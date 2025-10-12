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
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>


/**********************************************************************/
/************************* MACRO DEFINITIONS **************************/
/**********************************************************************/

#define VERSION "b001"
#define TMP_STUBBED_TEMPERATURE 14000
#define TMPR_SMPL_CIRC_BUFF_SIZE 20
#define TRUE  1
#define FALSE 0
#define u64 __u64
#define s32 __s32
#define u32 __u32

#define MASK_NEW_SMPL 1       // NEW SAMPLE FLAG MASK


// DT macros
#define SIMTEMP_COMPATIBLE           "nxp,simtemp"
#define SIMTEMP_DEFAULT_SAMPLING_MS  3000U
#define SIMTEMP_DEFAULT_THRESH_mC    45000


/*********************************************************************/
/*********************  TYPE DEFINITIONS *****************************/
/*********************************************************************/

struct simtempSample 
{
   u64 timestampNS;   // monotonic timestamp
   s32 tempMC;        // milli-degree Celsius (e.g., 44123 = 44.123 °C)
   u32 flags;         // bit0=NEW_SAMPLE, bit1=THRESHOLD_CROSSED (extend as needed)
   char consecutive;
} __attribute__((packed));

struct simtempDev     // DT input parameters type
{
   struct device       *dev;
   unsigned int         sampling_ms;
   int                  threshold_mC;
};

static const struct of_device_id simtemp_of_match[] = 
{
   { .compatible = SIMTEMP_COMPATIBLE },
   { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, simtemp_of_match);



/**********************************************************************/
/************************ GLOBAL VARIABLES ****************************/
/**********************************************************************/

struct delayed_work readSampleTimer;
/*
static struct miscdevice simtempMiscDev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "simtemp",
    .fops  = &simtempFOps,
    .mode  = 0666,     //  Read write permissions for everybody
};
*/


s32 tmprBase = TMP_STUBBED_TEMPERATURE;
struct simtempSample tmprSmplCircBuff[TMPR_SMPL_CIRC_BUFF_SIZE] = {0};   // Samples circular buffer
spinlock_t tmprSmplBuffLock;                            // Spinlock to make exclusie areas 
                                                        // when producing and consuming last sample
char simCnt = 0;
s32 tmprNewSample = 0xFFFFFFFF;
struct simtempSample sampleNew;
struct simtempSample tmprLastSample;
char newSampleFlag = FALSE;


u32 cbHead = 0;
u32 cbTail = 0;

u32 globalParam_SmplRate = SIMTEMP_DEFAULT_SAMPLING_MS;
int globalParam_tmprTreshold = SIMTEMP_DEFAULT_THRESH_mC;



/*********************************************************************/
/********************  FUNCTION PROTOTYPES ***************************/
/*********************************************************************/

static s32 tmprSampleReadADC(void);
static ssize_t simtempRead(struct file *f, char __user *buf, size_t len, loff_t *off);
static int simtempOpen(struct inode *ino, struct file *f);
static int simtempRelease(struct inode *ino, struct file *f);
static void simtempInitFromDT(struct simtempDev *sd);


// Device operations table
static const struct file_operations simtempFOps = {
    .owner   = THIS_MODULE,
    .read    = simtempRead,
    .open    = simtempOpen,
    .release = simtempRelease,
};



/*********************************************************************/
/******************** DEVICE TREE ************************************/
/*********************************************************************/



static void simtemp_parse_dt(struct simtempDev *sd, struct device *dev)
{
   struct device_node *np = dev->of_node;
   u32 u;

   /* Defaults */
   sd->sampling_ms  = SIMTEMP_DEFAULT_SAMPLING_MS;
   sd->threshold_mC = SIMTEMP_DEFAULT_THRESH_mC;

   if (!np)
   {
      pr_info("PARSE DT: Using defaults\n");
      return;
   }

   if (!of_property_read_u32(np, "sampling-ms", &u))
   {
      sd->sampling_ms = u;
   }

   if (!of_property_read_u32(np, "threshold-mC", &u))
   {
      sd->threshold_mC = (int)u;
   }
      
}

static int simtemp_probe(struct platform_device *pdev)
{
   struct simtempDev *sd;
   // int ret = 0;

   sd = devm_kzalloc(&pdev->dev, sizeof(*sd), GFP_KERNEL);
   if (!sd)
   {
      return -ENOMEM;
   }
      

   sd->dev = &pdev->dev;
   platform_set_drvdata(pdev, sd);

   /* Lee properties desde DT (o defaults si no hay of_node) */
   simtemp_parse_dt(sd, &pdev->dev);

   simtempInitFromDT(sd);    

    // ret = simtemp_core_init(sd);
    // if (ret) return ret;

    dev_info(&pdev->dev, "simtemp probed: sampling_ms=%u, threshold_mC=%d\n",
             sd->sampling_ms, sd->threshold_mC);
    return 0;
}

static void simtemp_remove(struct platform_device *pdev)
{
   // struct simtempDev *sd = platform_get_drvdata(pdev);

   // simtemp_core_exit(sd);

   dev_info(&pdev->dev, "simtemp removed\n");
}

static struct platform_driver simtemp_platform_driver = {
    .probe  = simtemp_probe,
    .remove = simtemp_remove,
    .driver = {
        .name           = "nxp_simtemp",
        .of_match_table = simtemp_of_match,
    },
};

/* Fallback: crea un platform_device local si no venimos de DT */
static struct platform_device *simtemp_pdev_fallback;


/*********************************************************************/


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jorge Sanchez");
MODULE_DESCRIPTION("miscdevice /dev/simtemp");
MODULE_VERSION(VERSION);

void simtempInitFromDT(struct simtempDev *sd)
{
   globalParam_SmplRate = sd->sampling_ms;
   globalParam_tmprTreshold = sd->threshold_mC;
}


/*********************************************************************/
/********************  TIMER FUNCTIONS *******************************/
/*********************************************************************/

/********************  TIMER RESET *******************************/

static void timerReset(void)
{
   //schedule_delayed_work(&readSampleTimer, 2 * HZ);
    schedule_delayed_work(&readSampleTimer, msecs_to_jiffies((unsigned long)globalParam_SmplRate));
}

/******************** TIMER CALL BACK ****************************/

static void readSampleTimerCallback(struct work_struct *work)
{
   timerReset();
   tmprNewSample = tmprSampleReadADC();
   sampleNew.tempMC = tmprNewSample;
   sampleNew.timestampNS = ktime_get_ns();
   sampleNew.flags = 1;
   sampleNew.consecutive = simCnt;
   s32 lostSample = -1;

   pr_info("Timer ejecutado, muestra numero: %i = %i, at: %llu\n",simCnt++, tmprNewSample, sampleNew.timestampNS);
    
   // head always point to the next position to be written in the circ buffer
   spin_lock(&tmprSmplBuffLock);
   tmprSmplCircBuff[cbHead] = sampleNew;  // write new sample in next position in circ buffer
   if (cbTail == ((cbHead + 1) % TMPR_SMPL_CIRC_BUFF_SIZE))
   {  // if tail is next to head, means that we will start to loose samples
      // because whole circ buffer is not yet read by consumer
      lostSample = tmprSmplCircBuff[cbTail].consecutive;
      cbTail = ((cbHead + 2) % TMPR_SMPL_CIRC_BUFF_SIZE);
   }
   cbHead = (cbHead + 1) % TMPR_SMPL_CIRC_BUFF_SIZE;
   spin_unlock(&tmprSmplBuffLock);
    
   if (lostSample != -1)
      pr_info("Se perdió la %i \n",lostSample);


   /*
   spin_lock(&tmprSmplBuffLock);
   tmprLastSample = sampleNew;
   spin_unlock(&tmprSmplBuffLock);
   */
    
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
   #define OUTPUT_SIZE 40
   ssize_t retVal = OUTPUT_SIZE;
   char auxStr[OUTPUT_SIZE] = {0};
   unsigned long notCopied;



   // struct simtempSample auxSample;

   spin_lock(&tmprSmplBuffLock);
   if (cbTail == cbHead)
   {
      spin_unlock(&tmprSmplBuffLock);
      return 0;
   }
   tmprLastSample = tmprSmplCircBuff[cbTail];
   cbTail = ((cbTail + 1) % TMPR_SMPL_CIRC_BUFF_SIZE);
   spin_unlock(&tmprSmplBuffLock);


   


   /*

   spin_lock(&tmprSmplBuffLock);
   if (MASK_NEW_SMPL == (tmprLastSample.flags & MASK_NEW_SMPL))
   {   
      retVal = sizeof(tmprLastSample);
   }
   tmprLastSample = sampleNew;
   tmprLastSample.flags = tmprLastSample.flags & (!(MASK_NEW_SMPL));
   spin_unlock(&tmprSmplBuffLock);

   */
    
   snprintf(auxStr, sizeof(auxStr), "%i %d, at: %llu\n", tmprLastSample.consecutive, tmprLastSample.tempMC, tmprLastSample.timestampNS);
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
   /*
   int retVal = misc_register(&simtempMiscDev);
    if (retVal) 
    {
        pr_err("simtemp: misc_register failed (%d)\n", retVal);
        return retVal;
    }

    */


   pr_info("INIT: Iniciando\n");

   int ret;

   ret = platform_driver_register(&simtemp_platform_driver);
   if (ret)
   {
      pr_info("INIT: falló ret = platform_driver_register(&simtemp_platform_driver); fin de INIT\n");
      return ret;
   }


    /* Fallback: crea un platform_device solo si NO hay DT poblado */
    if (!of_have_populated_dt()) 
    {
        pr_info("INIT: no hay DT poblado → creando fallback platform_device\n");
        simtemp_pdev_fallback = platform_device_register_simple("nxp_simtemp", PLATFORM_DEVID_AUTO, NULL, 0);
        if (IS_ERR(simtemp_pdev_fallback)) 
        {
            pr_info("INIT: No se pudo crear fallback, se cancela todo.\n");
            ret = PTR_ERR(simtemp_pdev_fallback);
            platform_driver_unregister(&simtemp_platform_driver);
            return ret;
        }
    } 
    else 
    {
        pr_info("INIT: DT sí está poblado → NO se crea fallback\n");
    }

   timerInit();
   pr_info("INIT: simtemp: platform driver registered\n");
   return 0;
}

static void __exit simtempExit(void)
{
    timerDeInit();
    if (simtemp_pdev_fallback && !IS_ERR(simtemp_pdev_fallback)) {
        platform_device_unregister(simtemp_pdev_fallback);
        simtemp_pdev_fallback = NULL;
    }
    platform_driver_unregister(&simtemp_platform_driver);
    pr_info("EXIT: platform driver unregistered\n");

/*
    misc_deregister(&simtempMiscDev);
    pr_info("simtemp: Unloaded.\n");

*/
}

module_init(simtempInit);
module_exit(simtempExit);
