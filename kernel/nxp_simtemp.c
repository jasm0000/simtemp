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
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched/signal.h>  /* TASK_INTERRUPTIBLE helpers */

/**********************************************************************/
/************************* MACRO DEFINITIONS **************************/
/**********************************************************************/

#define VERSION "b001"
#define TMPR_SMPL_CIRC_BUFF_SIZE     20

#define SIMTEMP_THRESH_MIN_mC   (-40000)
#define SIMTEMP_THRESH_MAX_mC   (125000)


#define TRUE  1
#define FALSE 0
#define u64 __u64
#define s32 __s32
#define u32 __u32
#define u16 __u16

#define MASK_NEW_SMPL 1       // NEW SAMPLE FLAG MASK


// DT macros
#define SIMTEMP_COMPATIBLE           "nxp,simtemp"
#define SIMTEMP_SAMPLING_MS_DEFAULT  3000U
#define SIMTEMP_SAMPLING_MS_MAX      60000U
#define SIMTEMP_SAMPLING_MS_MIN      5U
#define SIMTEMP_THRESH_DEFAULT    45000

#define GLOBAL_MODE_NONBLOCKING  0    // Global returning sample mode non blocking call
#define GLOBAL_MODE_BLOCKING     1    // Global returning sampple mode blocking call

#define NOISE_RANGE_NORMAL 10
#define NOISE_RANGE_NOISY  3000
#define RAMP_RANGE         10000
#define RAMP_STEP          10



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jorge Sanchez");
MODULE_DESCRIPTION("miscdevice /dev/simtemp");
MODULE_VERSION(VERSION);


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
   char                 mode;
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
s32 tmprBase = SIMTEMP_THRESH_DEFAULT;
struct simtempSample tmprSmplCircBuff[TMPR_SMPL_CIRC_BUFF_SIZE] = {0};   // Samples circular buffer
spinlock_t tmprSmplBuffLock;                            // Spinlock to make exclusie areas 
                                                        // when producing and consuming last sample
spinlock_t sysfsLock;
char simCnt = 0;
s32 tmprNewSample = 0xFFFFFFFF;
struct simtempSample sampleNew;
struct simtempSample tmprLastSample;
char newSampleFlag = FALSE;


u32 cbHead = 0;
u32 cbTail = 0;

u32 globalParam_SmplRate = SIMTEMP_SAMPLING_MS_DEFAULT;
int globalParam_tmprTreshold = SIMTEMP_THRESH_DEFAULT;

char globalMode = GLOBAL_MODE_NONBLOCKING;

static DECLARE_WAIT_QUEUE_HEAD(simtempWait);

// static bool thresholdEvent = FALSE;

u16 noiseRange = NOISE_RANGE_NORMAL;
u16 rampSample;



/*********************************************************************/
/********************  FUNCTION PROTOTYPES ***************************/
/*********************************************************************/

static s32 tmprSampleReadADC(void);
static ssize_t simtempRead(struct file *f, char __user *buf, size_t len, loff_t *off);
static int simtempOpen(struct inode *ino, struct file *f);
static int simtempRelease(struct inode *ino, struct file *f);
static void simtempInitFromDT(struct simtempDev *sd);
static __poll_t simtempPoll(struct file *file, poll_table *wait);

ssize_t samplingMsStore(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count);


ssize_t samplingMsShow(struct device *dev,
                              struct device_attribute *attr,
                              char *buf);
void resetGlobalModeParams(void);




// Device operations table
static const struct file_operations simtempFOps = 
{
   .owner   = THIS_MODULE,
   .read    = simtempRead,
   .open    = simtempOpen,
   .release = simtempRelease,
   .poll  = simtempPoll,
};



static struct miscdevice simtempMiscDev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "simtemp",
    .fops  = &simtempFOps,
    .mode  = 0666,     //  Read write permissions for everybody
};



/*********************************************************************/
/******************************* SYSFS *******************************/
/*********************************************************************/

enum simtempMode
{
   SIMTEMP_MODE_NORMAL = 0,
   SIMTEMP_MODE_NOISY,
   SIMTEMP_MODE_RAMP
};


static int globalParam_Threshold = 45000;
static enum simtempMode globalParam_Mode = SIMTEMP_MODE_NORMAL;

/* Stats simples (RO) */
struct simtempStats
{
   u64 produced;
   u64 delivered;
   u64 overruns;
   u64 alerts;
};
static struct simtempStats globalStats;

static ssize_t thresholdMcShow(struct device *dev,
                               struct device_attribute *attr,
                               char *buf)
{
   unsigned long flags;
   int v;

   spin_lock_irqsave(&sysfsLock, flags);
   v = globalParam_Threshold;
   spin_unlock_irqrestore(&sysfsLock, flags);

   return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}

static ssize_t thresholdMcStore(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf,
                                size_t count)
{
   int val;
   int oldVal;
   int ret = kstrtoint(buf, 0, &val);
   if (ret)
   {
      return ret;
   }
      

   if (val < SIMTEMP_THRESH_MIN_mC || val > SIMTEMP_THRESH_MAX_mC)
   {
      return -EINVAL;
   }

   spin_lock(&sysfsLock);
   oldVal = globalParam_Threshold;
   globalParam_Threshold = val;
   spin_unlock(&sysfsLock);

   pr_info("SYSFS: threshold_mC was updated from %d to %d\n", oldVal, globalParam_Threshold);
   return count;
}

/* Archivo sysfs: threshold_mC */
static struct device_attribute thresholdMcAttr =
   __ATTR(threshold_mC, 0664, thresholdMcShow, thresholdMcStore);


static const char * modeToStr(enum simtempMode m)
{
   switch (m)
   {
      case SIMTEMP_MODE_NORMAL: return "normal";
      case SIMTEMP_MODE_NOISY:  return "noisy";
      case SIMTEMP_MODE_RAMP:   return "ramp";
      default:                return "normal";
   }
}

static int modeFromStr(const char *buf, enum simtempMode *out)
{
   if (sysfs_streq(buf, "normal")) { *out = SIMTEMP_MODE_NORMAL; return 0; }
   if (sysfs_streq(buf, "noisy"))  { *out = SIMTEMP_MODE_NOISY;  return 0; }
   if (sysfs_streq(buf, "ramp"))   { *out = SIMTEMP_MODE_RAMP;   return 0; }
   return -EINVAL;
}

static ssize_t modeShow(struct device *dev,
                        struct device_attribute *attr,
                        char *buf)
{
   unsigned long flags;
   enum simtempMode m;

   spin_lock_irqsave(&sysfsLock, flags);
   m = globalParam_Mode;
   spin_unlock_irqrestore(&sysfsLock, flags);

   return scnprintf(buf, PAGE_SIZE, "%s\n", modeToStr(m));
}

static ssize_t modeStore(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf,
                         size_t count)
{
   enum simtempMode m;
   int ret = modeFromStr(buf, &m);
   if (ret)
      return ret;

   spin_lock(&sysfsLock);
   globalParam_Mode = m;
   spin_unlock(&sysfsLock);

   pr_info("SYSFS: mode set to %s\n", modeToStr(m));
   return count;
}

/* Archivo sysfs: mode */
static struct device_attribute modeAttr =
   __ATTR(mode, 0664, modeShow, modeStore);


static ssize_t statsShow(struct device *dev,
                         struct device_attribute *attr,
                         char *buf)
{
   unsigned long flags;
   struct simtempStats s;

   spin_lock_irqsave(&sysfsLock, flags);
   s = globalStats;
   spin_unlock_irqrestore(&sysfsLock, flags);

   return scnprintf(buf, PAGE_SIZE,
                    "produced=%llu\n"
                    "delivered=%llu\n"
                    "overruns=%llu\n"
                    "alerts=%llu\n",
                    s.produced, s.delivered, s.overruns, s.alerts);
}

/* Archivo sysfs: stats (RO) */
static struct device_attribute statsAttr =
   __ATTR(stats, 0444, statsShow, NULL);






ssize_t samplingMsShow(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{

   unsigned long flags;
   u32 v;

   spin_lock_irqsave(&sysfsLock, flags);
   v = globalParam_SmplRate;
   spin_unlock_irqrestore(&sysfsLock, flags);

   return scnprintf(buf, PAGE_SIZE, "%u\n", v);

}

ssize_t samplingMsStore(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
   unsigned int val;
   u32 oldVal;
   int ret = kstrtouint(buf, 0, &val);
   if (ret)
      return ret;

   // Validate limits
   if (val < SIMTEMP_SAMPLING_MS_MIN || val > SIMTEMP_SAMPLING_MS_MAX)
   {
      return -EINVAL;
   }
      
   oldVal = globalParam_SmplRate;
   spin_lock(&sysfsLock);
   globalParam_SmplRate = val;
   spin_unlock(&sysfsLock);

   pr_info("SYSFS: Sample rate was updated from %u to %u\n", oldVal, globalParam_SmplRate);

   return count;
}



static struct device_attribute samplingMsAttr =
   __ATTR(sampling_ms, 0664, samplingMsShow, samplingMsStore);




/* Lista de atributos sysfs del driver */
static struct attribute *simtempAttrs[] =
{
   &samplingMsAttr.attr,
   &thresholdMcAttr.attr,
   &modeAttr.attr,
   &statsAttr.attr,
   NULL,
};

static const struct attribute_group simtempAttrGroup =
{
   .attrs = simtempAttrs,
};


/*********************************************************************/
/******************** DEVICE TREE ************************************/
/*********************************************************************/



static void simtemp_parse_dt(struct simtempDev *sd, struct device *dev)
{
   struct device_node *np = dev->of_node;
   u32 u;

   /* Defaults */
   sd->sampling_ms  = SIMTEMP_SAMPLING_MS_DEFAULT;
   sd->threshold_mC = SIMTEMP_THRESH_DEFAULT;
   sd->mode = SIMTEMP_MODE_NORMAL;
   sd->mode = SIMTEMP_MODE_RAMP;

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

static struct platform_driver simtemp_platform_driver = 
{
   .probe  = simtemp_probe,
   .remove = simtemp_remove,
   .driver = 
   {
      .name           = "nxp_simtemp",
      .of_match_table = simtemp_of_match,
   },
};

/* Fallback: crea un platform_device local si no venimos de DT */
static struct platform_device *simtemp_pdev_fallback;


void simtempInitFromDT(struct simtempDev *sd)
{
   globalParam_SmplRate = sd->sampling_ms;
   globalParam_tmprTreshold = sd->threshold_mC;
   globalParam_Mode = sd->mode;
}



/*********************************************************************/


/*********************************************************************/
/********************  TIMER FUNCTIONS *******************************/
/*********************************************************************/

/********************  TIMER RESET *******************************/

static void timerReset(void)
{
   //schedule_delayed_work(&readSampleTimer, 2 * HZ);

   u32 smplRate_Aux;
   spin_lock(&sysfsLock);
   smplRate_Aux = globalParam_SmplRate; // Using an aux variable to reduce the exclusive area time
   spin_unlock(&sysfsLock);

   schedule_delayed_work(&readSampleTimer, msecs_to_jiffies((unsigned long)smplRate_Aux));
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
   newSampleFlag = TRUE;
   spin_unlock(&tmprSmplBuffLock);

   wake_up_interruptible(&simtempWait);

    
   if (lostSample != -1)
      pr_info("Se perdió la %i \n",lostSample);


   /*
   spin_lock(&tmprSmplBuffLock);
   tmprLastSample = sampleNew;
   spin_unlock(&tmprSmplBuffLock);
   */
    
}


/*********************************************************************/
/*********************************************************************/
/****************************** POLL *********************************/
/*********************************************************************/
/*********************************************************************/


static __poll_t simtempPoll(struct file *file, poll_table *wait)
{
   __poll_t mask = 0;

   /* Registrar esta wait queue con el subsistema de poll/epoll */
   poll_wait(file, &simtempWait, wait);

   /* Reportar qué eventos están listos */
   spin_lock(&tmprSmplBuffLock);

   /* Datos listos para leer → POLLIN | POLLRDNORM */
   if (cbTail != cbHead)
      mask |= POLLIN | POLLRDNORM;

   /* Umbral cruzado POLLPRI (out-of-band) */
   /*
   if (thresholdEvent)
      mask |= POLLPRI;

      */

   spin_unlock(&tmprSmplBuffLock);

   return mask;
}


/******************** TIMER INIT ********************************/

static void timerInit(void)
{
   INIT_DELAYED_WORK(&readSampleTimer, readSampleTimerCallback);
   spin_lock_init(&tmprSmplBuffLock);
   timerReset();


   rampSample = globalParam_Threshold - RAMP_RANGE;
}

static void sysfsInit(void)
{
   spin_lock_init(&sysfsLock);
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

enum simtempMode auxMode;
int auxThreshold;

static s32 tmprSampleReadADC(void)
{
   spin_lock(&sysfsLock);
   auxMode = globalParam_Mode;
   auxThreshold = globalParam_Threshold;
   spin_unlock(&sysfsLock);

   switch (auxMode)
   {
      case SIMTEMP_MODE_RAMP:
         if (rampSample > auxThreshold + RAMP_RANGE)
         {
            rampSample = auxThreshold - RAMP_RANGE;
         }
         rampSample += RAMP_STEP;
         return rampSample;
      case SIMTEMP_MODE_NOISY:
         return tmprBase + (get_random_u32() % (NOISE_RANGE_NOISY+1));
      case SIMTEMP_MODE_NORMAL:
      default:
         return tmprBase + (get_random_u32() % (NOISE_RANGE_NORMAL+1));
   }

}


void resetGlobalModeParams(void)
{
   noiseRange = NOISE_RANGE_NORMAL;
   rampSample = globalParam_Threshold - RAMP_RANGE;
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

   spin_lock(&tmprSmplBuffLock);
   if (cbTail == cbHead)
   {
      spin_unlock(&tmprSmplBuffLock);
      if (f->f_flags & O_NONBLOCK)
      {
         return -EAGAIN;
      }

      int wret = wait_event_interruptible(simtempWait, cbTail != cbHead);
      if (wret)   /* -ERESTARTSYS si recibe señal */
      {
         return wret;
      }
      /* Resume execution since there are new available sample(s) */
      spin_lock(&tmprSmplBuffLock);
   }
   tmprLastSample = tmprSmplCircBuff[cbTail];
   cbTail = ((cbTail + 1) % TMPR_SMPL_CIRC_BUFF_SIZE);
   if (cbTail == cbHead)
   {
      newSampleFlag = FALSE;
   }
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
   
   int retVal = misc_register(&simtempMiscDev);
   if (retVal) 
   {
      pr_err("simtemp: misc_register failed (%d)\n", retVal);
      return retVal;
   }


   /* Cuelga el grupo completo en: /sys/class/misc/simtemp/ */
   retVal = sysfs_create_group(&simtempMiscDev.this_device->kobj, &simtempAttrGroup);
   if (retVal)
   {
      pr_err("simtemp: sysfs_create_group() failed (%d)\n", retVal);
      misc_deregister(&simtempMiscDev);
      return retVal;
   }

   pr_info("INIT: Iniciando\n");

   retVal = platform_driver_register(&simtemp_platform_driver);
   if (retVal)
   {
      pr_info("INIT: falló ret = platform_driver_register(&simtemp_platform_driver); fin de INIT\n");
      return retVal;
   }


   /* Fallback: crea un platform_device solo si NO hay DT poblado */
   if (!of_have_populated_dt()) 
   {
      pr_info("INIT: no hay DT poblado → creando fallback platform_device\n");
      simtemp_pdev_fallback = platform_device_register_simple("nxp_simtemp", PLATFORM_DEVID_AUTO, NULL, 0);
      if (IS_ERR(simtemp_pdev_fallback)) 
      {
         pr_info("INIT: No se pudo crear fallback, se cancela todo.\n");
         retVal = PTR_ERR(simtemp_pdev_fallback);
         platform_driver_unregister(&simtemp_platform_driver);
         return retVal;
      }
   } 
   else 
   {
      pr_info("INIT: DT sí está poblado → NO se crea fallback\n");
   }
   timerInit();
   sysfsInit();
   pr_info("INIT: simtemp: platform driver registered\n");
   return 0;
}


/******************* DEVICE EXIT OPERATION ************************/

static void __exit simtempExit(void)
{
   timerDeInit();
   if (simtemp_pdev_fallback && !IS_ERR(simtemp_pdev_fallback)) {
      platform_device_unregister(simtemp_pdev_fallback);
      simtemp_pdev_fallback = NULL;
   }
   platform_driver_unregister(&simtemp_platform_driver);
   pr_info("EXIT: platform driver unregistered\n");


   if (simtempMiscDev.this_device)
   {
      sysfs_remove_group(&simtempMiscDev.this_device->kobj, &simtempAttrGroup);
   }

   misc_deregister(&simtempMiscDev);
   pr_info("simtemp: Unloaded.\n");
}

module_init(simtempInit);
module_exit(simtempExit);
