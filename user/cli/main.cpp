// simtemp/user/cli/main.cpp
#include <iostream>
#include <string>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <poll.h>

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
   CMD_SHOW_ALL
};

/****************************************************************************/
/****************************************************************************/
/****************************** GLOBAL VARIABLES ****************************/
/****************************************************************************/
/****************************************************************************/



/****************************************************************************/
/****************************************************************************/
/******************************** PRINT HELP ********************************/
/****************************************************************************/
/****************************************************************************/

static void printHelp()
{
   cout << "simtempCLI (hello)\n";
   cout << "Usage:\n";
   cout << "   ./simtempCli --once              [--nonblock] [--timeout MS]\n";
   cout << "   ./simtempCli --follow            [--nonblock] [--timeout MS] [--reopen]\n";
   cout << "   ./simtempCli --set-sampling MS\n";
   cout << "   ./simtempCli --set-mode {normal|noisy|ramp}\n";
   cout << "   ./simtempCli --set-threshold mC\n";
   cout << "   ./simtempCli --show-all\n";
   cout << "\nNotes:\n";
   cout << " - --set-sampling writes /sys/class/misc/simtemp/sampling_ms\n";
   cout << " - --set-mode writes /sys/class/misc/simtemp/mode\n";
   cout << " - --set-threshold writes /sys/class/misc/simtemp/threshold_mC\n";
   cout << " - --show-all reads sampling_ms, mode, threshold_mC and stats\n";
   cout << " - --timeout applies to --once and --follow; use -1 for infinite wait\n";
   cout << " - --reopen  performs reopen at eof, in cat command style\n";
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
                  << "  sudo ./simtempCli --set-sampling " << value << "\n";
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
/****************************************************************************/
/********************************* ONCE *************************************/
/****************************************************************************/
/****************************************************************************/

// Reads a single sample from /dev/simtemp
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

   // Reading what driver has to return
   char buffer[128] = {0};
   ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1);
   if (bytesRead < 0)
   {
      cerr << "ERROR: read failed (" << strerror(errno) << ")\n";
      close(fd);
      return false;
   }

   cout << "Read from driver (" << bytesRead << " bytes):\n";
   cout << buffer << endl;

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

      // Data available: read and print
      if ((pfd.revents & POLLPRI) != 0)
      {
         char buffer[256] = {0};
         ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1);
         if (bytesRead < 0)
         {
            if (errno == EAGAIN || errno == EWOULDBLOCK) { continue; }
            cerr << "ERROR: read (POLLPRI) failed (" << strerror(errno) << ")\n";
            close(fd);
            return false;
         }
         if (bytesRead == 0)
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

         cerr << "[ALERT] poll: POLLPRI\n";
         cout.write(buffer, bytesRead);
         cout.flush();
         continue;
      }

      if ((pfd.revents & POLLIN) != 0 || (pfd.revents & POLLRDNORM) != 0)
      {
         char buffer[256] = {0};
         ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1);
         if (bytesRead < 0)
         {
            if (errno == EAGAIN || errno == EWOULDBLOCK) { continue; }
            cerr << "ERROR: read failed (" << strerror(errno) << ")\n";
            close(fd);
            return false;
         }
         if (bytesRead == 0)
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

         // Print raw text
         cout.write(buffer, bytesRead);
         if (buffer[bytesRead - 1] != '\n')
         {
            // cout << "\n";
         }
         cout.flush();
      }
   }

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

      case CMD_NONE:
         break;
   }

   printHelp();
   return 0;
}
