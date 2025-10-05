#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>


/************************ MACROS DEFINITIONS **************************/

#define VERSION "b001"


/************************ GLOBAL VARIABLES ****************************/

char arreglito[20] = "0123456789ABCDEFG\n";
struct delayed_work readSampleTimer;

/*********************************************************************/


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jorge Sanchez");
MODULE_DESCRIPTION("miscdevice /dev/simtemp");
MODULE_VERSION(VERSION);




/*********************************************************************/
/********************  TIMER FUNCTIONS *******************************/
/*********************************************************************/

/********************  TIMER RESET *******************************/

static void timerReset(void)
{
    schedule_delayed_work(&readSampleTimer, 2 * HZ);
}

/******************** TIMER CALL BACK ****************************/

static void readSampleTimerCallback(struct work_struct *work)
{
    static char simCnt = 0;
    printk(KERN_INFO "Timer ejecutado, muestra numero: %i\n",simCnt++);
    timerReset();
}

/******************** TIMER INIT ********************************/

static void timerInit(void)
{
    INIT_DELAYED_WORK(&readSampleTimer, readSampleTimerCallback);
    timerReset();
}

/******************* TIMER DE INIT ******************************/

static void timerDeInit(void)
{
    cancel_delayed_work_sync(&readSampleTimer);
}

/*********************************************************************/
/*********************************************************************/
/********************  KERNEL DEVICE SETUP ***************************/
/*********************************************************************/
/*********************************************************************/


/******************* DEVICE READ OPERATION **********************/


static ssize_t simtempRead(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    return -EAGAIN;
}

/******************* DEVICE OPEN OPERATION ***************************/

static int simtempOpen(struct inode *ino, struct file *f)
{
    pr_info("simtemp: open\n");
    return 0;
}

/******************* DEVICE RELEASE OPERATION ************************/


static int simtempRelease(struct inode *ino, struct file *f)
{
    pr_info("simtemp: release\n");
    return 0;
}

// Tabla de operaciones
static const struct file_operations simtempFOps = {
    .owner   = THIS_MODULE,
    .read    = simtempRead,
    .open    = simtempOpen,
    .release = simtempRelease,
};

// miscdevice crea /dev/simtemp automáticamente
static struct miscdevice simtempMiscDev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "Temperature simulator",
    .fops  = &simtempFOps,
    .mode  = 0666,     //  Read write permissions for everybody
};

static int __init simtempInit(void)
{
    int retVal = misc_register(&simtempMiscDev);
    if (retVal) 
    {
        pr_err("simtemp: misc_register failed (%d)\n", retVal);
        return retVal;
    }
    timerInit();
    pr_info("simtemp: Loaded. /dev/simtemp Ready "VERSION"\n");
    return 0;
}

static void __exit simtempExit(void)
{
    timerDeInit();
    misc_deregister(&simtempMiscDev);
    pr_info("simtemp: Unloaded.\n");
}

module_init(simtempInit);
module_exit(simtempExit);
