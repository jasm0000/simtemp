# **DESIGN – Virtual Temperature Sensor (simtemp)**
## **1. Overview**
The nxp_simtemp project implements a **virtual temperature sensor** as a Linux kernel module paired with a **C++ command-line tool**.\
The purpose is to simulate a hardware sensor with realistic timing, asynchronous data delivery, and alert events when a temperature threshold is crossed.

The design follows the principle of simplicity and modularity: a single, well-structured driver (nxp_simtemp.c) with clear separation between **data generation**, **SysFS configuration**, and **character-device I/O**.\
The user-space CLI provides full access to configuration and testing without additional dependencies.

## **2. System Architecture**
### **2.1 Block Description**
**Kernel space:**

- **Sample Generator:** periodic workqueue producing temperature values based on the selected mode (normal, noisy, or ramp).
- **Global FIFO Buffer:** stores recent samples; supports blocking and non-blocking reads.
- **Event Notifier:** signals readers through poll() when a new sample or threshold crossing occurs.
- **SysFS Interface:** exposes configurable parameters.
- **Character Device (/dev/simtemp):** provides binary sample access for user applications.

**User space:**

- **CLI (C++):** communicates with /dev/simtemp and SysFS.
  - Supports --once, --follow, configuration commands, and --test automated validation.
- **Scripts:** automate compilation and demonstration.

| User Space           |
| -------------------- |
| simtempCLI           |
| - once/follow        |
| - test mode          |
| - sysfs writes       |
| +------------------+ |
      |
      v
+---------------------+
| Kernel Space        |
| nxp_simtemp.ko      |
| - /dev/simtemp      |
| - sysfs interface   |
| - wait queue + FIFO |
| - periodic worker   |
+---------------------+

The kernel and user-space layers interact via:

- **SysFS writes:** configuration path (sampling rate, mode, threshold).
- **Character device read:** data path (binary 16-byte records).
- **Poll events:** alert path (threshold crossing).

## **3. Interaction Flow**
1. User executes the CLI or shell commands to configure parameters via SysFS.
2. The kernel timer/workqueue triggers periodic sample generation.
3. Each new sample is pushed into the FIFO queue.
4. If the new temperature crosses the threshold, a special event flag is set and waiting readers are notified via poll().
5. The CLI retrieves samples using read() and decodes the 16-byte record.

This approach allows multiple independent user programs to read from the same queue (competing consumers), ensuring realistic device semantics.

## **4. Locking and Synchronization**
**Spinlocks** are used to protect shared structures that may be accessed in interrupt or timer context:

- Circular buffer indices (cbHead, cbTail)
- Shared flags (hasAlert, newSample)

The show functions in SysFS use spin_lock_irqsave / spin_unlock_irqrestore because they can be invoked from contexts that cannot sleep.
The store functions use the regular spin_lock / spin_unlock, as they are triggered from user space where sleeping is allowed.

Wait queues (wait_event_interruptible) are used for blocking reads.
When new data arrives or an alert occurs, the queue is awakened safely under the protection of the spinlock.

**Spinlocks** are the proper concurrency protection mecanisms at Kernel space beacause **Mutexes** send the thread to sleep when the lock condition happens, and in Kernel space that is much more critical than in user space, that's why spinlocks does not send the thread to sleep, instead it performs an active wait until condition gets unlocked, that's why spinlocks are more critical and should be used in very short locked sections to minimize the lock time.

## **5. API Design and Trade-Offs**
### **SysFS**

- All configuration and statistics were implemented through SysFS because:
- Using SysFS for configuration and poll() for alerts provides a clean separation between **control** and **data** paths.

## **6. Device Tree Mapping**
A minimal compatible binding is supported:

compatible = "nxp,simtemp";

Default properties (sampling period, threshold) are defined internally for cases where the DT is absent, such as x86 virtual testing.

The driver includes an of_match_table with the compatible string "nxp,simtemp".
On real i.MX or QEMU targets, it would bind automatically when this node is present in the device tree.
For environments without a populated DT, the module registers a fallback platform_device so it can operate standalone for testing.

Although the implementation currently creates a local platform device for simplicity, the same driver could bind automatically to a DT node on an i.MX or QEMU-ARM64 system by adding the snippet below:

simtemp0: simtemp@0 {
    compatible = "nxp,simtemp";
    sampling-ms = <100>;
    threshold-mC = <45000>;
    status = "okay";
};


## **7. Data Handling and Binary Format**
Each sample is represented by a 16-byte packed structure:

u64 timestamp_ns
s32 temp_mC
u32 flags   // bit0 = NEW_SAMPLE, bit1 = THRESHOLD

This was implemented using the following structure:
struct simtempRecord16
{
   u64 timestamp_ns;   /* monotonic ns */
   s32 temp_mC;        /* milli °C */
   u32 flags;          /* bit0=NEW_SAMPLE, bit1=THRESHOLD */
} __attribute__((packed));

To ensure endianess and sizes preserved (__attribute__((packed));) when sending the data to the consumer.

The driver uses copy_to_user() to safely transfer records.
Partial reads are supported if the user reads fewer than 16 bytes, the remaining bytes are served on the next read.

The CLI automatically detects binary format and decodes it for display (on initial debugging versions, ASCII format was returned to be able to debug using linux CAT since CLI was not still implemented).

## **8. Design Choices**
**8.1 Global FIFO Queue**\
All readers share a single queue. This simplifies synchronization and ensures sample integrity. Once a record is consumed, it is no longer available to other readers—mimicking a real hardware buffer.

**8.2 Modes of Operation**

- *Normal:* quasi-static nominal value with random fluctuation of +/-100 mC
- *Noisy:* random fluctuation of +/-2000 mC around the nominal value 
- *Ramp:* continuous increase until a threshold, then reset.

This allows testing both steady and dynamic conditions.

**8.3 Alert Strategy**
Threshold alerts are triggered any time that threshold_mC value is between previous and current generated samples.
The flag bit is set in the sample record and poll() wakes up any waiting process with POLLPRI.

## **9. Scaling and Performance Considerations**
At typical sampling periods (≥50 ms), the driver is fully reliable.\
If sampling were increased toward 10 kHz, the following would become limiting factors:

- Workqueue scheduling latency.
- Spinlock contention on the FIFO.
- Memory throughput for frequent copy_to_user() operations.

Potential mitigations:

- Switch to high-resolution timers (hrtimer).
- Use a lock-free circular buffer.
- Move event signaling to atomic flags.

## **10. User-Space Integration**
The CLI application was designed to be minimal and robust:

- Reads binary records directly and converts them to human-readable text.
- Automatically retries partial reads.
- Provides blocking and non-blocking modes.
- Includes an integrated **Test Mode** that verifies threshold alerts automatically.

This guarantees full compliance with the acceptance criteria without external tools.

## **11. Future Improvements and what would I do with more time**
- Investigate how to connect to a real ADC driver in a real ARM64/QEMU hardware.
- Add DT overlay for ARM64/QEMU demo.
- Implement dedicated ioctl interface for batch configuration.
- Add GUI (Qt or Tkinter) for visualization.
- Extend the CLI --test mode to validate noise and ramp accuracy.
- Implement an optional persistent “stats” file for runtime metrics.

## **12. Summary**
The nxp_simtemp design meets all mandatory requirements of the NXP Systems Software Engineer Challenge:

- Out-of-tree kernel driver with sysfs, pollable read, and clean teardown.
- C++ CLI with blocking, non-blocking, follow, and test modes.
- Build and demo scripts.
- Documented design choices, synchronization, and extensibility.

The overall goal was to deliver a **functional, well-structured, and realistic simulation** of a Linux sensor driver, emphasizing correctness and maintainability.

