**AI NOTES – Virtual Temperature Sensor (simtemp)**
## **Following are some of the promts used to solve some of the questions and roadblocks found during the development of the project**

1. I have this new project to work on, and I need to structure it. The first part I'll need is your help figuring out what tools I need to install to configure my entire setup and get started. Help me with links to where I can download a virtual machine with Ubuntu, and in Ubuntu, help me with the commands in Linux I need to download, install the compiler, git, and other tools to compile C code in Kernel Space and C++ code in User Space, and create my repository.
2. Give me an overview of how sysfs is structured and what libraries and function sets are used to read and write parameters.
3. How do I register my driver in kernel space? Give me a general explanation of how to implement it, including libraries and function sets.
4. Give me an overview of how Device Tree is structured and what libraries and function sets are used to read DT parameters.
5. How do I generate a random (or pseudo-random) number in Linux? With which library?
6. With what library and set of functions can I implement a mutex to create exclusive concurrency areas in kernel space?
7. With what library and set of functions do I use spinlocks in the Linux kernel?
8. How can I use delayed_work in the kernel to generate samples at regular intervals?
9. How can I use the Linux cat command to read my driver's read function?
10. How can I put a function to sleep in kernel space? With what library and set of functions? To make blocking calls, I use the function to wake up when a new sample is available.
11. With what Linux command can I verify if my driver is already allocated in memory?
12. From C++ in user space, how can I poll my driver? With what library and set of functions?
13. How do you recommend recording the video requested in the md file?

## **How did I validate the produced outputs?**

Due to the nature of project the outputs to those prompts were self-verifiable, I mean, in all of the cases it was information that were verified instantly when trying to implement or apply the responded information because they would be GO/NO GO, so the validation was just to implement, test, compile, run commands, etc., and the results would be very quick to be confirmed.
