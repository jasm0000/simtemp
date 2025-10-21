// simtemp/user/cli/main.cpp
#include <iostream>
#include <string>
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

static const char* devPath = "/dev/simtemp";

enum commandType
{
   CMD_NONE,
   CMD_ONCE
};


/****************************************************************************/
/****************************************************************************/
/****************************** GLOBAL VARIABLES ****************************/
/****************************************************************************/
/****************************************************************************/



/****************************************************************************/
/******************************** PRINT HELP ********************************/
/****************************************************************************/

static void printHelp()
{
   std::cout << "simtempCLI (hello)\n";
   std::cout << "Usage:\n";
   std::cout << "   ./simtempCli --once [--nonblock] [--timeout MS]\n";
}

/****************************************************************************/
/********************************  ONCE  *********************************/
/****************************************************************************/

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
   //pfd.events = POLLIN | POLLRDNORM | POLLPRI;
   pfd.events = POLLPRI;



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
      cerr << "ERROR: read  failed (" << strerror(errno) << ")\n";
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
/********************************  M A I N  *********************************/
/****************************************************************************/
/****************************************************************************/


int main(int argc, char** argv)
{
   int timeoutMs = -1;
   bool nonblock = false;

   enum commandType command = CMD_NONE;

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
      else if (arg == "--nonblock")
      {
         nonblock = true;
      }
      else if (arg == "--timeout" && i + 1 < argc)
      {
         timeoutMs = stoi(argv[++i]);
      }
      else
      {
         cerr << "Unknown parameters: " << arg << endl;
         printHelp();
         return 1;
      }
   }

   switch (command)
   {
      case CMD_ONCE:
         if (!readOnce(nonblock, timeoutMs))
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
