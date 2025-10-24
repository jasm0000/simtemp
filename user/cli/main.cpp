/********************************************************************************/
/*                                                                              */
/*  Project:     Linux kernel + user-space CLI challenge project                */
/*  Module:      main.cpp                                                       */
/*  Description: Command Line Interface user space consumer                     */
/*  Author:      Jorge A. Sánchez                                               */
/*  Date:        24 / OCT / 2025                                                */
/*                                                                              */
/********************************************************************************/

// simtemp/user/cli/main.cpp
#include <iostream>
#include <string>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <poll.h>
#include <stdint.h>
#include <algorithm>

using namespace std;

/****************************************************************************/
/****************************************************************************/
/****************************** GLOBAL CONSTANTS ****************************/
/****************************************************************************/
/****************************************************************************/

static const char* devPath  = "/dev/simtemp";
static const char* sysfsDir = "/sys/class/misc/simtemp";

enum COMMAND_TYPE
{
   CMD_NONE,
   CMD_ONCE,
   CMD_FOLLOW,
   CMD_SET_SAMPLING,
   CMD_SET_MODE,
   CMD_SET_THRESHOLD,
   CMD_SHOW_ALL,
   CMD_TEST,
   CMD_WAIT_ALERT
};

/****************************************************************************/
/****************************************************************************/
/****************************** TYPE DEFINITIONS ****************************/
/****************************************************************************/
/****************************************************************************/



struct __attribute__((packed)) SimtempRecord16
{
   uint64_t timestamp_ns;  /* monotonic ns */
   int32_t  temp_mC;       /* milli °C */
   uint32_t flags;         /* bit0=NEW_SAMPLE, bit1=THRESHOLD */
};

/* Make human-readable output from a decoded 16B record. */
#include <iomanip>
#include <iostream>

#include <iomanip>
#include <iostream>

static void printSampleHuman(const SimtempRecord16& r) 
{ 
   double t_ms = static_cast<double>(r.timestamp_ns) / 1e6; 
   double t_C = static_cast<double>(r.temp_mC) / 1000.0; 
   unsigned alert = (r.flags & (1u << 1)) ? 1u : 0u; 
   cout << "t_ms = " << t_ms 
        << " temp = " << t_C 
        << "C Thresh. alert = " 
        << alert << "\n"; 
}
/* enum to report how a read attempt finished. */
enum READ_RESULT
{
   RR_OK = 0,   /* printed something successfully (binary or ASCII) */
   RR_EOF,      /* got EOF while trying to read */
   RR_ERR       /* fatal read error */
};



/****************************************************************************/
/****************************************************************************/
/******************************** PRINT HELP ********************************/
/****************************************************************************/
/****************************************************************************/

static void printHelp()
{
   cout << "simtempCLI (Temperature Simulator - CLI, Version 1.0)\n";
   cout << "Usage:\n";
   cout << "   ./simtempCLI --once              [--nonblock] [--timeout MS]\n";
   cout << "   ./simtempCLI --follow            [--nonblock] [--timeout MS] [--reopen]\n";
   cout << "   ./simtempCLI --set-sampling MS\n";
   cout << "   ./simtempCLI --set-mode {normal|noisy|ramp}\n";
   cout << "   ./simtempCLI --set-threshold mC\n";
   cout << "   ./simtempCLI --show-all\n";
   cout << "   ./simtempCLI --test --period MS\n";
   cout << "   ./simtempCli --wait-alert [--timeout MS]\n";
   cout << "\nNotes:\n";
   cout << " - --set-sampling writes /sys/class/misc/simtemp/sampling_ms\n";
   cout << " - --set-mode writes /sys/class/misc/simtemp/mode\n";
   cout << " - --set-threshold writes /sys/class/misc/simtemp/threshold_mC\n";
   cout << " - --show-all reads sampling_ms, mode, threshold_mC and stats\n";
   cout << " - --timeout applies to --once and --follow; use -1 for infinite wait\n";
   cout << " - --reopen performs reopen at EOF, in cat command style\n";
   cout << " - --test: forces mode=ramp, uses --period MS, auto-sets threshold and waits for POLLPRI (<= ~2*period)\n";
   cout << " - --wait-alert blocks until a threshold alert arrives (POLLPRI) or timeout\n";
   cout << " - You can combine --set-xxx with --once/--follow (set first, then read)\n";
}

/****************************************************************************/
/******************************* SYSFS HELPERS ******************************/
/****************************************************************************/

// Write a simple text value into a sysfs file (e.g., sampling_ms)
static bool writeTextFile(const string& path, const string& value)
{
   ofstream f(path);
   if (!f.is_open())
   {
      cerr << "ERROR: could not open " << path << " for writing ("
           << strerror(errno) << ")\n";
      if (errno == EACCES)
      {
         std::cerr << "Hint: run with sudo, e.g.:\n"
                  << "  sudo ./simtempCLI --set-sampling " << value << "\n";
      }
      return false;
   }


   f << value << std::endl;

   if (!f.good())
   {
      cerr << "ERROR: failed to write to " << path << "\n";
      return false;
   }
   return true;
}

// Set sampling period in milliseconds via sysfs
static bool setSamplingMs(unsigned ms)
{
   string path = string(sysfsDir) + "/sampling_ms";
   return writeTextFile(path, to_string(ms));
}

// Set operating mode via sysfs (expects: normal|noisy|ramp)
static bool setMode(const string& mode)
{
   if (mode != "normal" && mode != "noisy" && mode != "ramp")
   {
      cerr << "ERROR: invalid mode: " << mode << " (use normal|noisy|ramp)\n";
      return false;
   }

   string path = string(sysfsDir) + "/mode";
   return writeTextFile(path, mode);
}

// Set threshold in milli-degrees Celsius via sysfs (e.g., 45000 = 45.000 C)
static bool setThresholdmC(int mC)
{
   string path = string(sysfsDir) + "/threshold_mC";
   return writeTextFile(path, to_string(mC));
}

static bool readTextFile(const string& path, string& out)
{
   ifstream f(path);
   if (!f.is_open())
   {
      cerr << "ERROR: could not open " << path << " for reading ("
           << strerror(errno) << ")\n";
      return false;
   }

   string line;
   out.clear();
   while (std::getline(f, line))
   {
      out += line;
      out.push_back('\n');
   }

   if (!f.good() && !f.eof())
   {
      cerr << "ERROR: failed to read from " << path << "\n";
      return false;
   }

   return true;
}


/* One single CLI option to read all SYSFS parameters */
static bool showAll()
{
   bool ok = true;
   string v;

   cout << "---- simtemp sysfs ----\n";

   v.clear();
   if (readTextFile(string(sysfsDir) + "/sampling_ms", v))
   {
      cout << "sampling_ms:\n   " << v;
   }
   else
   {
      ok = false;
   }

   v.clear();
   if (readTextFile(string(sysfsDir) + "/mode", v))
   {
      cout << "mode:\n   " << v;
   }
   else
   {
      ok = false;
   }

   v.clear();
   if (readTextFile(string(sysfsDir) + "/threshold_mC", v))
   {
      cout << "threshold_mC:\n   " << v;
   }
   else
   {
      ok = false;
   }

   v.clear();
   if (readTextFile(string(sysfsDir) + "/stats", v))
   {
      cout << "stats:\n" << v;
   }
   else
   {
      ok = false;
   }

   cout.flush();
   return ok;
}


/****************************************************************************/
/********************** 16-BYTE BINARY SAMPLE RECORD READ *******************/
/****************************************************************************/


/* 
 * Try to accumulate exactly 16 bytes and decode the binary record.
 * - If exactly 16 bytes collected: decode and print one line: RR_OK
 * - If less than 16 (short read, driver still ASCII, etc) then
 *     fall back to printing as text using a small buffer: RR_OK (unless 0 and EOF)
 * - If we see n == 0 before collecting anything: RR_EOF.
 * - If we hit a fatal error: RR_ERR.
 * This helper does NOT call poll(), caller guarantees readiness
 */
static enum READ_RESULT readBinaryOrTextOnce(int fd)
{
   alignas(SimtempRecord16) unsigned char recBuf[sizeof(SimtempRecord16)] = {};
   // static_assert(sizeof(SimtempRecord16) == 16, "SimtempRecord16 must be 16 bytes");

   size_t have = 0;

   while (have < sizeof(recBuf))
   {
      ssize_t n = ::read(fd, recBuf + have, sizeof(recBuf) - have);
      if (n < 0)
      {
         if (errno == EINTR)
         {
            continue; /* interrupted by signal, just retry */
         }
         if (errno == EAGAIN || errno == EWOULDBLOCK)
         {
            /* no more data now; stop trying to complete the record */
            break;
         }
         cerr << "ERROR: read failed (" << strerror(errno) << ")\n";
         return RR_ERR;
      }
      if (n == 0)
      {
         /* EOF while accumulating; if we had 0 so far, treat as EOF */
         if (have == 0)
         {
            return RR_EOF;
         }
         /* if we had some bytes, treat as incomplete and fall back to text below */
         break;
      }
      have += static_cast<size_t>(n);
   }

   if (have == sizeof(recBuf))
   {
      SimtempRecord16 rec{};
      // memcpy(&rec, recBuf, sizeof(rec));
      std::copy_n(recBuf, sizeof(recBuf), reinterpret_cast<unsigned char*>(&rec));


      cout << "Read from driver: ";
      printSampleHuman(rec);
      cout.flush();
      return RR_OK;
   }

   /* Fallback to text: print whatever we have as raw text (plus try to extend it) */
   {
      char buf[256] = {0};
      size_t used = 0;

      if (have > 0)
      {
         size_t copy = have;
         if (copy > sizeof(buf) - 1) copy = sizeof(buf) - 1;
         std::copy_n(recBuf, copy, buf); 
         used = copy;
      }

      if (used < sizeof(buf) - 1)
      {
         ssize_t m = ::read(fd, buf + used, (sizeof(buf) - 1) - used);
         if (m > 0)
         {
            used += static_cast<size_t>(m);
         }
         else if (m == 0 && used == 0)
         {
            /* real EOF and nothing to print */
            return RR_EOF;
         }
         else if (m < 0)
         {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            {
               cerr << "ERROR: read (fallback) failed (" << strerror(errno) << ")\n";
               return RR_ERR;
            }
         }
      }

      if (used > 0)
      {
         cout << "Read from driver (" << used << " bytes):\n";
         cout.write(buf, used);
         if (buf[used - 1] != '\n')
         {
            cout << "\n";
         }
         cout.flush();
      }
      /* nothing printable */
      return RR_OK;
   }
}

/****************************************************************************/
/********************** HELPER FOR 16B READING ******************************/
/****************************************************************************/

/* Reads exactly one 16-byte record with a blocking poll first.
 * Returns true and fills out if successful, false on error/timeout.
 */
static bool readOneBinarySampleBlocking(int fd, int timeoutMs, SimtempRecord16 &out)
{
   struct pollfd pfd;
   pfd.fd = fd;
   pfd.events = POLLIN | POLLRDNORM | POLLPRI;

   int pr = poll(&pfd, 1, timeoutMs);
   if (pr < 0)
   {
      cerr << "ERROR: poll in test read (" << strerror(errno) << ")\n";
      return false;
   }
   if (pr == 0)
   {
      cerr << "TEST: poll timeout waiting for sample\n";
      return false;
   }

   size_t have = 0;
   unsigned char *ptr = reinterpret_cast<unsigned char*>(&out);
   while (have < sizeof(out))
   {
      ssize_t n = ::read(fd, ptr + have, sizeof(out) - have);
      if (n < 0)
      {
         if (errno == EINTR) continue;
         if (errno == EAGAIN || errno == EWOULDBLOCK) break;
         cerr << "ERROR: read in test (" << strerror(errno) << ")\n";
         return false;
      }
      if (n == 0)
      {
         cerr << "TEST: EOF while reading sample\n";
         return false;
      }
      have += (size_t)n;
   }

   if (have != sizeof(out))
   {
      cerr << "TEST: short read (" << have << " bytes)\n";
      return false;
   }
   return true;
}


/****************************************************************************/
/****************************************************************************/
/******************************* TEST MODE **********************************/
/****************************************************************************/
/****************************************************************************/

/* Test:
 * 1) set mode=ramp and sampling_ms=periodMs
 * 2) open /dev/simtemp and read one sample S
 * 3) set threshold_mC = S + 1
 * 4) wait for POLLPRI within <= 2*periodMs + 500 ms
 */
static bool runTestMode(int periodMs)
{
   if (periodMs <= 0)
   {
      cerr << "ERROR: --test requires a positive --period\n";
      return false;
   }

   // 1) Configure sysfs (force ramp mode first)
   if (!setMode(string("ramp")))
   {
      cerr << "ERROR: failed to set mode=ramp during --test\n";
      return false;
   }
   if (!setSamplingMs((unsigned)periodMs))
   {
      cerr << "ERROR: failed to set sampling_ms during --test\n";
      return false;
   }

   // 2) Open device and read one binary sample S
   int fd = open(devPath, O_RDONLY);
   if (fd < 0)
   {
      cerr << "ERROR: Could not open " << devPath << " (" << strerror(errno) << ")\n";
      return false;
   }

   SimtempRecord16 first{};
   int t1 = max( (2 * periodMs) + 500, 1500 );  // 1500ms covers 1000ms default

   if (!readOneBinarySampleBlocking(fd, t1, first))
   {
      close(fd);
      return false;
   }

   // 3) threshold = S + 1 mC (forces a crossing next tick in ramp mode)
   int autoThresh = first.temp_mC + 1;
   if (!setThresholdmC(autoThresh))
   {
      cerr << "ERROR: failed to set threshold_mC during --test\n";
      close(fd);
      return false;
   }

   // 4) Wait for POLLPRI within <= 2*period + 500 ms
   int totalTimeoutMs = (2 * periodMs) + 500;
   struct pollfd pfd;
   pfd.fd = fd;
   pfd.events = POLLPRI | POLLIN | POLLRDNORM;

   int ret = poll(&pfd, 1, totalTimeoutMs);
   if (ret < 0)
   {
      cerr << "ERROR: poll failed in --test (" << strerror(errno) << ")\n";
      close(fd);
      return false;
   }
   if (ret == 0)
   {
      cerr << "[TEST] FAIL: no POLLPRI within " << totalTimeoutMs << " ms\n";
      close(fd);
      return false;
   }

   if ((pfd.revents & POLLPRI) == 0)
   {
      cerr << "[TEST] FAIL: event arrived but not POLLPRI (revents=0x"
           << std::hex << pfd.revents << std::dec << ")\n";
      close(fd);
      return false;
   }

   // Consume one read to clear condition
   char buf[32] = {0};
   buf[0] = read(fd, buf, sizeof(buf));

   cout << "[TEST] PASS: threshold alert detected via POLLPRI (ramp)\n";
   close(fd);
   return true;
}


/****************************************************************************/
/****************************************************************************/
/********************************* ONCE *************************************/
/****************************************************************************/
/****************************************************************************/

// Reads one single sample from /dev/simtemp
static bool readOnce(bool nonblock, int timeoutMs)
{
   int flags = O_RDONLY;
   if (nonblock)
   {
      flags |= O_NONBLOCK;
   }

   int fd = open(devPath, flags);
   if (fd < 0)
   {
      cerr << "ERROR: Could not open " << devPath << " (" << strerror(errno) << ")\n";
      return false;
   }

   // Using poll to wait for data from driver
   struct pollfd pfd;
   pfd.fd = fd;
   pfd.events = POLLIN | POLLRDNORM | POLLPRI;
   //pfd.events = POLLPRI;



   int ret = poll(&pfd, 1, timeoutMs);
   if (ret < 0)
   {
      cerr << "ERROR: poll failed (" << strerror(errno) << ")\n";
      close(fd);
      return false;
   }
   if (0 == ret)
   {
      cerr << "TIMEOUT: no data received in " << timeoutMs << " ms\n";
      close(fd);
      return false;
   }

   /* perform a single binary-or-text read */
   enum READ_RESULT rr = readBinaryOrTextOnce(fd);
   if (rr == RR_ERR)
   {
      close(fd);
      return false;
   }
   if (rr == RR_EOF)
   {
      cerr << "EOF from device\n";
      close(fd);
      return false;
   }

   close(fd);
   return true;
}

/****************************************************************************/
/****************************************************************************/
/*********************************** FOLLOW *********************************/
/****************************************************************************/
/****************************************************************************/

// Follow mode:
// - Supports timeout (--timeout).
// - Differentiates POLLIN/POLLRDNORM (data) vs POLLPRI (alert).
// - Supports reopen in cat command style.
static bool readFollow(bool nonblock, int timeoutMs, bool reopenOnEof)
{
   int flags = O_RDONLY;
   if (nonblock)
   {
      flags |= O_NONBLOCK;
   }

   int fd = open(devPath, flags);
   if (fd < 0)
   {
      cerr << "ERROR: Could not open " << devPath << " (" << strerror(errno) << ")\n";
      return false;
   }

   struct pollfd pfd;
   pfd.fd = fd;
   pfd.events = POLLIN | POLLRDNORM | POLLPRI;

   for (;;)
   {
      int ret = poll(&pfd, 1, timeoutMs);
      if (ret < 0)
      {
         if (errno == EINTR)
         {
            continue;
         }
         cerr << "ERROR: poll failed (" << strerror(errno) << ")\n";
         close(fd);
         return false;
      }

      if (ret == 0)
      {
         cerr << "TIMEOUT: no data received in " << timeoutMs << " ms\n";
         break;
      }

      if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
      {
         cerr << "Device not readable anymore (revents=0x"
              << std::hex << pfd.revents << std::dec << "). Exiting follow.\n";
         break;
      }

      // Data available: read and print (binary 16B or fallback to text)
      if ((pfd.revents & POLLPRI) != 0)
      {
         enum READ_RESULT rr = readBinaryOrTextOnce(fd);
         if (rr == RR_ERR)
         {
            close(fd);
            return false;
         }
         if (rr == RR_EOF)
         {
            cerr << "EOF after POLLPRI\n";
            if (reopenOnEof)
            {
               close(fd);
               fd = open(devPath, flags);
               if (fd < 0)
               {
                  cerr << "ERROR: reopen failed (" << strerror(errno) << ")\n";
                  return false;
               }
               pfd.fd = fd;
               continue;
            }
            break;
         }
         continue;
      }

      if ((pfd.revents & POLLIN) != 0 || (pfd.revents & POLLRDNORM) != 0)
      {
         enum READ_RESULT rr = readBinaryOrTextOnce(fd);
         if (rr == RR_ERR)
         {
            close(fd);
            return false;
         }
         if (rr == RR_EOF)
         {
            cerr << "EOF from device\n";
            if (reopenOnEof)
            {
               close(fd);
               fd = open(devPath, flags);
               if (fd < 0)
               {
                  cerr << "ERROR: reopen failed (" << strerror(errno) << ")\n";
                  return false;
               }
               pfd.fd = fd;
               continue;
            }
            break;
         }
      }
   }

   close(fd);
   return true;
}


/****************************************************************************/
/****************************************************************************/
/*****************************  WAIT ALERT  *********************************/
/****************************************************************************/
/****************************************************************************/

// Waits only for POLLPRI from /dev/simtemp.
// - Returns true if an alert arrives within timeoutMs.
// - Returns false on timeout or error.
// - Does not consume normal data; only reacts to POLLPRI.
static bool waitAlertOnly(int timeoutMs)
{
   int fd = open(devPath, O_RDONLY);
   if (fd < 0)
   {
      cerr << "ERROR: Could not open " << devPath << " (" << strerror(errno) << ")\n";
      return false;
   }

   struct pollfd pfd;
   pfd.fd = fd;
   pfd.events = POLLPRI;   // we only care about threshold alerts here

   int ret = poll(&pfd, 1, timeoutMs);
   if (ret < 0)
   {
      if (errno == EINTR)
      {
         // interrupted by signal; treat as failure for this helper
         cerr << "ERROR: poll interrupted (" << strerror(errno) << ")\n";
      }
      else
      {
         cerr << "ERROR: poll failed (" << strerror(errno) << ")\n";
      }
      close(fd);
      return false;
   }
   if (ret == 0)
   {
      cerr << "TIMEOUT: no alert received in " << timeoutMs << " ms\n";
      close(fd);
      return false;
   }

   if ((pfd.revents & POLLPRI) == 0)
   {
      // Some other event happened; for this mode, we treat it as failure.
      cerr << "Unexpected event (revents=0x"
           << std::hex << pfd.revents << std::dec << ") without POLLPRI\n";
      close(fd);
      return false;
   }

   // Optional: one read to clear the condition (format agnostic).
   // Not strictly required; safe to leave as best-effort.
   char buf[32] = {0};
   ssize_t n = read(fd, buf, sizeof(buf));
   (void)n;

   cout << "[wait-alert] Threshold alert detected (POLLPRI)\n";
   close(fd);
   return true;
}


/****************************************************************************/
/****************************************************************************/
/********************************  M A I N  *********************************/
/****************************************************************************/
/****************************************************************************/

int main(int argc, char** argv)
{
   int timeoutMs    = -1;
   bool nonblock    = false;
   bool reopenOnEof = false;
   unsigned samplingMs = 0;
   string mode;
   int thresholdmC = 0;
   int testPeriodMs = -1;

   enum COMMAND_TYPE command = CMD_NONE;

   /*  PARSE INPUT PARAMETERS */
   for (int i = 1; i < argc; ++i)
   {
      string arg = argv[i];
      if (arg == "--help" || arg == "--h")
      {
         printHelp();
         return 0;
      }
      else if (arg == "--once")
      {
         command = CMD_ONCE;
      }
      else if (arg == "--follow")
      {
         command = CMD_FOLLOW;
      }
      else if (arg == "--nonblock")
      {
         nonblock = true;
      }
      else if (arg == "--timeout" && i + 1 < argc)
      {
         timeoutMs = stoi(argv[++i]);
      }
      else if (arg == "--reopen")
      {
         reopenOnEof = true;
      }
      else if (arg == "--set-sampling" && i + 1 < argc)
      {
         command = CMD_SET_SAMPLING;
         samplingMs = static_cast<unsigned>(stoul(argv[++i]));
      }
      else if (arg == "--set-mode" && i + 1 < argc)
      {
         command = CMD_SET_MODE;
         mode = argv[++i];
      }
      else if (arg == "--set-threshold" && i + 1 < argc)
      {
         command = CMD_SET_THRESHOLD;
         thresholdmC = stoi(argv[++i]);
      }
      else if (arg == "--show-all")
      {
         command = CMD_SHOW_ALL;
      }
      else if (arg == "--test")
      {
         command = CMD_TEST;
      }
      else if (arg == "--period" && i + 1 < argc)
      {
         testPeriodMs = stoi(argv[++i]);
      }
      else if (arg == "--wait-alert")
      {
         command = CMD_WAIT_ALERT;
      }
      else
      {
         cerr << "Unknown parameters: " << arg << endl;
         printHelp();
         return 1;
      }
   }

   // Dispatch command
   switch (command)
   {
      case CMD_ONCE:
         if (!readOnce(nonblock, timeoutMs))
            return 1;
         return 0;

      case CMD_FOLLOW:
         if (!readFollow(nonblock, timeoutMs, reopenOnEof))
            return 1;
         return 0;

      case CMD_SET_SAMPLING:
         if (!setSamplingMs(samplingMs))
            return 1;
         return 0;

      case CMD_SET_MODE:
         if (!setMode(mode))
            return 1;
         return 0;

      case CMD_SET_THRESHOLD:
         if (!setThresholdmC(thresholdmC))
            return 1;
         return 0;

      case CMD_SHOW_ALL:
         if (!showAll())
            return 1;
         return 0;

      case CMD_TEST:
         if (testPeriodMs <= 0)
         {
            cerr << "Usage: --test --period MS\n";
            return 1;
         }
         if (!runTestMode(testPeriodMs))
         {
            return 1;
         }
         return 0;

      case CMD_WAIT_ALERT:
         if (!waitAlertOnly(timeoutMs))
         {
            return 1;
         }
         return 0;

         case CMD_NONE:
         break;
   }

   printHelp();
   return 0;
}
