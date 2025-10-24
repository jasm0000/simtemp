# **README – Virtual Temperature Sensor (simtemp)**

## **1. Project usage basic instructions**

- Goto Section: **5. Build and Execution Steps** for specific install instructions.
- Goto Section: **6. Demo Video and Repository Link** for video and repository links.

## **2. Repository Structure**

simtemp/
├── kernel/
│   ├── simtemp.c
│   └── Makefile
├── user/
│   └── cli/
│       ├── main.cpp
│       └── Makefile
├── scripts/
│   ├── build.sh
│   └── run_demo.sh
└── docs/
`    `├── README.md
`    `├── DESIGN.md
`    `└── TESTPLAN.md

kernel/ – Linux miscdevice driver sources and Makefile.
user/cli/ – C++ CLI application and Makefile.
scripts/ – Minimal helper scripts for building and demo.
docs/ – All Markdown documentation (README, design, and tests).

## **3. Functional Description**
### **3.1 The Kernel Module (nxp_simtemp.ko)**
The driver registers as a **miscdevice** under /dev/simtemp.
It periodically generates temperature samples and raises a **threshold alert** when the value crosses a programmable limit.
The driver provides both **character-device access** and **SysFS configuration files** located under:

/sys/class/misc/simtemp/
#### **SysFS Attributes**

|**File name**|**Access**|**Description**                                    |
|sampling_ms  |RW        |Sampling period in milliseconds                    |
|mode         |RW        |Operating mode: normal, noisy, or ramp             |
|threshold_mC |RW        |Temperature threshold in milli-degrees Celsius     |
|stats        |RO        |Internal counters (samples, alerts, overruns, etc.)|

The module uses a **global FIFO queue** for generated samples.
All readers (CLI, cat, etc.) compete for the same data, meaning each sample is consumed once by the first reader.
This design ensures thread-safe and deterministic behavior without sample duplication.

### **3.2 Device Read Format**
Each read from /dev/simtemp produces a **16-byte binary record**:

|**Field**   |**Type**|**Description**                            |
| :-:        | :-:    | :-:                                       |
|timestamp_ns|u64     |Timestamp in nanoseconds (monotonic clock) |
|temp_mC     |s32     |Temperature in milli-degrees Celsius       |
|flags       |u32     |Bit0 = NEW_SAMPLE, Bit1 = THRESHOLD alert  |

The driver can also return **ASCII text** when debug output is enabled.
The CLI automatically detects the format and decodes it appropriately.

Partial reads are supported: if fewer than 16 bytes are available, the CLI will attempt to accumulate until the record is complete or gracefully fall back to text mode.

### **3.3 The CLI Application (simtempCli)**
The CLI, written in C++, allows complete interaction with the driver and its SysFS interface.

Supported commands:

./simtempCli --once              [--nonblock] [--timeout MS]

./simtempCli --follow            [--nonblock] [--timeout MS] [--reopen]

./simtempCli --set-sampling MS

./simtempCli --set-mode {normal|noisy|ramp}

./simtempCli --set-threshold mC

./simtempCli --show-all

./simtempCli --test --period MS --threshold mC

./simtempCli --wait-alert       [--timeout MS]n";

#### **Command Summary**
- **--once**: Reads one sample and prints it (blocking by default).
  With --nonblock, use --timeout 0 to avoid waiting.
- **--follow**: Continuously reads and prints new samples in real time.
  With --reopen, it reopens the device automatically on EOF (cat-style behavior).
- **--set-sampling**, **--set-mode**, **--set-threshold**: Write SysFS parameters.
- **--show-all**: Reads all SysFS parameters and the stats file.
- **--test**: Runs an automated test verifying the threshold alert via POLLPRI.
- **--wait-alert**: This mode allows the CLI to wait exclusively for a threshold-crossing event reported by the kernel driver.
While other modes such as --once or --follow react to any new temperature sample (normal data-ready events, POLLIN), the --wait-alert command blocks until the driver signals a POLLPRI event — which occurs only when the simulated temperature crosses the configured threshold.

#### **Binary Sample Reading**
The CLI reads both binary and text output.
Binary records are decoded to human-readable form:

t_ms=2.15171e+08 temp=44.982C alert=1
(time is in exponential notation to save visual space in the output console, but can be adapted to any other specific need)

### **3.4 Test Mode**
The --test option provides an automated functional validation:

1. Sets sampling and threshold values via SysFS.
2. Opens /dev/simtemp.
3. Waits for a POLLPRI event (threshold crossing) within **2×period + 500 ms**.
4. Reports **PASS** or **FAIL** to the console.

This ensures the alert mechanism and poll() handling are working correctly.

## **4. Shell Scripts**
### **4.1 build.sh**
A simple helper to compile both kernel and user components.

Steps performed:

1. Runs make inside kernel/ to build simtemp.ko.
2. Runs make inside user/cli/ to build simtempCli.
3. Displays a success message if both targets build correctly.

Example:

./build.sh

### **4.2 run_demo.sh**
A minimal automation script to demonstrate the required six tests.
It loads the driver, runs a sequence of operations, and then unloads it.

Sequence:

1. Load kernel module.
2. Configure sampling, mode, and threshold.
3. Perform single and continuous reads.
4. Trigger a threshold alert.
5. Run automated test (--test).
6. Unload the module.

The script is intentionally simple and uses basic shell commands, reflecting a user-level understanding rather than an advanced scripting framework.

## **5. Build and Execution Steps**
### **Prerequisites**
Install:
sudo apt install build-essential linux-headers-$(uname -r)

Repository main folder (simtemp) shall be copied to home directory  ~/

### **Build**
cd ~/simtemp/scripts
./build.sh
### **Load / Unload Driver**
sudo insmod ../kernel/simtemp.ko

sudo rmmod simtemp
cd ..
### **Run CLI Examples**
./user/cli/simtempCli --show-all

./user/cli/simtempCli --set-sampling 500

./user/cli/simtempCli --set-mode ramp

./user/cli/simtempCli --set-threshold 45000

./user/cli/simtempCli --once --timeout 1000

./user/cli/simtempCli --follow --timeout 5000 --reopen

./user/cli/simtempCli --test --period 1000 --threshold 45000

## **6. Demo Video and Repository Link**
As required by the challenge instructions, the final submission includes:

- **public GitHub repository link** containing all source code and documentation:
    https://github.com/jasm0000/simtemp

- **short demo video (4 minutes)** showing:
  - Module compilation and loading
  - SysFS configuration
  - Sample reading
  - Threshold alert
  - Automated test
  - Clean module removal
  Link: https://drive.google.com/file/d/1jRSOxgL-DzlPLWg17i5EvUwMpWusiNO-/view?usp=drive_link
    
## **7. Notes and Known Behavior**
- The driver uses a **global FIFO**: multiple readers compete for samples.
- The CLI’s --once --nonblock mode is effectively non-blocking only if used with --timeout 0.
- SysFS writes require root privileges.
- The ramp mode starts from an internal reference value rather than the last sample; this was intentionally kept simple to ensure deterministic behavior during testing.

## **8. License and Author**
License: GPLv2
Author: Jorge Sánchez, 2025
Contact: jorgsanc@yahoo.com

## **9. Related Documentation**
- **DESIGN.md** – Kernel structure, synchronization strategy, and CLI design.
- **TESTSPLAN.md** – Detailed description of the six required tests and their expected outcomes.
- **AI_NOTES.md** – Some of the prompts made to AI as part of the investigation required to develop the project.

