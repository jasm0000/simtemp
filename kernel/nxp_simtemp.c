#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/types.h>



/************************* MACRO DEFINITIONS **************************/

#define VERSION "b001"
#define tmprStubbedTemp 14000
#define TMPR_SMPL_CIRC_BUFF_SIZE 14000
#define TRUE  1
#define FALSE 0
#define u64 __u64
#define s32 __s32
#define u32 __u32

#define MASK_NEW_SMPL 1       // NEW SAMPLE FLAG MASK




/*********************************************************************/
/********************  TYPES DEFINITIONS *****************************/
/*********************************************************************/

struct simtempSample {
    u64 timestampNS;   // monotonic timestamp
    s32 tempMC;        // milli-degree Celsius (e.g., 44123 = 44.123 °C)
    u32 flags;         // bit0=NEW_SAMPLE, bit1=THRESHOLD_CROSSED (extend as needed)
} __attribute__((packed));


/*********************************************************************/
/********************  FUNCTION PROTOTYPES ***************************/
/*********************************************************************/

static s32 tmprSampleReadADC(void);
static ssize_t simtempRead(struct file *f, char __user *buf, size_t len, loff_t *off);
static int simtempOpen(struct inode *ino, struct file *f);
static int simtempRelease(struct inode *ino, struct file *f);



/**********************************************************************/
/************************ GLOBAL VARIABLES ****************************/
/**********************************************************************/

struct delayed_work readSampleTimer;
// Device operations table
static const struct file_operations simtempFOps = {
    .owner   = THIS_MODULE,
    .read    = simtempRead,
    .open    = simtempOpen,
    .release = simtempRelease,
};

static struct miscdevice simtempMiscDev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "simtemp",
    .fops  = &simtempFOps,
    .mode  = 0666,     //  Read write permissions for everybody
};


s32 tmprBase = tmprStubbedTemp;
struct simtempSample tmprSmplCircBuff[TMPR_SMPL_CIRC_BUFF_SIZE] = {0};   // Samples circular buffer
spinlock_t tmprSmplBuffLock;                            // Spinlock to make exclusie areas 
                                                        // when producing and consuming last sample
char simCnt = 0;
s32 tmprNewSample = 0xFFFFFFFF;
struct simtempSample sampleNew;
struct simtempSample tmprLastSample;
char newSampleFlag = FALSE;


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
    timerReset();
    tmprNewSample = tmprSampleReadADC();
    sampleNew.tempMC = tmprNewSample;
    sampleNew.timestampNS = ktime_get_ns();
    sampleNew.flags = 1;

    pr_info("Timer ejecutado, muestra numero: %i = %i, at: %llu\n",simCnt++, tmprNewSample, sampleNew.timestampNS);
    //if (cbHead)
    //tmprSmplCircBuff[cbHead++] = sample;

    spin_lock(&tmprSmplBuffLock);
    tmprLastSample = sampleNew;
    spin_unlock(&tmprSmplBuffLock);

    
}

/******************** TIMER INIT ********************************/

static void timerInit(void)
{
    INIT_DELAYED_WORK(&readSampleTimer, readSampleTimerCallback);
    spin_lock_init(&tmprSmplBuffLock);
    timerReset();
}

/******************* TIMER DE INIT ******************************/

static void timerDeInit(void)
{
    cancel_delayed_work_sync(&readSampleTimer);
}

/*********************************************************************/
/*********************************************************************/
/*************** TEMPERATURE SAMPLES HANDLING ************************/
/*********************************************************************/
/*********************************************************************/

static s32 tmprSampleReadADC(void)
{
    // u32 r = prandom_u32() % 1001;
    s32 r = get_random_u32() % 1001;
    return tmprBase + r;

}

/*********************************************************************/
/*********************************************************************/
/********************  KERNEL DEVICE SETUP ***************************/
/*********************************************************************/
/*********************************************************************/


/******************* DEVICE READ OPERATION **********************/


static ssize_t simtempRead(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    ssize_t retVal = 0;
    #define OUTPUT_SIZE 40
    char auxStr[OUTPUT_SIZE] = {0};
    unsigned long notCopied;

    spin_lock(&tmprSmplBuffLock);
    if (MASK_NEW_SMPL == (tmprLastSample.flags & MASK_NEW_SMPL))
    {   
        retVal = sizeof(tmprLastSample);
    }
    tmprLastSample = sampleNew;
    tmprLastSample.flags = tmprLastSample.flags & (!(MASK_NEW_SMPL));
    spin_unlock(&tmprSmplBuffLock);
    
    snprintf(auxStr, sizeof(auxStr), "%d, at: %llu\n", tmprLastSample.tempMC, tmprLastSample.timestampNS);
    if (retVal != 0)
    {
        retVal = OUTPUT_SIZE;
//        copy_to_user(buf, &tmprLastSample, sizeof(tmprSmplBuffLock));
        notCopied = copy_to_user(buf, auxStr, OUTPUT_SIZE);
        if (notCopied)
        {
            return -EFAULT;
        }
        
    }
    return retVal;
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

/******************* DEVICE INIT OPERATION ************************/


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
