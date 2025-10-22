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
#include <linux/sched/signal.h>
#include <linux/slab.h>

/**********************************************************************/
/************************* MACRO DEFINITIONS **************************/
/**********************************************************************/

#define VERSION "b001"



#define SIMTEMP_MODE_DEFAULT         SIMTEMP_MODE_RAMP
#define SIMTEMP_THRESH_DEFAULT       45000
#define SIMTEMP_SAMPLING_MS_DEFAULT  1000U

#define NOISE_RANGE_NORMAL 100
#define NOISE_RANGE_NOISY  2000

#define SIMTEMP_NOISE_BASE SIMTEMP_THRESH_DEFAULT


#define SIMTEMP_MODE_NORMAL  0
#define SIMTEMP_MODE_NOISY   1
#define SIMTEMP_MODE_RAMP    2




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
#define SIMTEMP_SAMPLING_MS_MAX      60000U
#define SIMTEMP_SAMPLING_MS_MIN      5U

#define GLOBAL_MODE_NONBLOCKING  0    // Global returning sample mode non blocking call
#define GLOBAL_MODE_BLOCKING     1    // Global returning sampple mode blocking call

#define RAMP_RANGE         10000
#define RAMP_STEP          100

#define FLAGS_NEW_SAMPLE      0x0001
#define FLAGS_THRESHOLD_CROSS 0x0002



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jorge Sanchez");
MODULE_DESCRIPTION("miscdevice /dev/simtemp");
MODULE_VERSION(VERSION);


/*********************************************************************/
/*********************  TYPE DEFINITIONS *****************************/
/*********************************************************************/

struct simtempRecord16
{
   u64 timestamp_ns;   /* monotonic ns */
   s32 temp_mC;        /* milli °C */
   u32 flags;          /* bit0=NEW_SAMPLE, bit1=THRESHOLD */
} __attribute__((packed));

// per-file descriptor context to support short reads safely.
// It stores a frozen 16B record and the copy offset across read() calls.
struct simtempFctx
{
   struct simtempRecord16 out;
   size_t outOff;      /* how many bytes have been returned so far */
   bool   outValid;    /* there is a record pending completion */
};

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
s32 tmprBase = SIMTEMP_NOISE_BASE;
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

u16 rampSample;


bool thresholdEvent;
bool threshEventAux;
bool notFirstSample = false;  // Control flag to aviod triggering a false threshold event in the first time a sample is generated
s32 tmprPrevSample;
s32 globalParam_ThresholdAux;
char auxMode;
int auxThreshold;
int noiseRangeAux;
s32 tmprBaseAux;



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

int noiseRange;   
/* noiseRange variable will always hold the half of the noise range 
   value configured depending on the mode, noisy or normal */


/*********************************************************************/
/*********************************************************************/
/******************************* SYSFS *******************************/
/*********************************************************************/
/*********************************************************************/

static int globalParam_Threshold = 45000;
static char globalParam_Mode = SIMTEMP_MODE_NORMAL;

/* Stats simples (RO) */
struct simtempStats
{
   u64 produced;     // Number of samples produced
   u64 delivered;    // Number of samples consumed by user
   u64 overruns;     // Number of times a sample was lost because of buffer full before reading last sample
   u64 alerts;       // Number of times temperature threshold was crossed
};
static struct simtempStats globalStats = {0};

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


static inline void updateNoiseParams(void)   // Declared inline for efficiency purposes in the critical time calls
{
   if (SIMTEMP_MODE_NORMAL == globalParam_Mode)
   {
      noiseRange = NOISE_RANGE_NORMAL / 2;
   }
   if (SIMTEMP_MODE_NOISY == globalParam_Mode)
   {
      noiseRange = NOISE_RANGE_NOISY / 2;
   }
   tmprBase = globalParam_Threshold +  noiseRange - (noiseRange / 10);
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
   updateNoiseParams();
   // tmprBase = val +  noiseRange - (noiseRange / 10);
   spin_unlock(&sysfsLock);

   pr_info("SYSFS: threshold_mC was updated from %d to %d\n", oldVal, globalParam_Threshold);
   pr_info("SYSFS: tmprBase: %d, noiseRange: %d, noiseRange / 10 = %d\n", 
                   tmprBase, 
                   noiseRange,
                   noiseRange / 10);
   return count;
}

/* Archivo sysfs: threshold_mC */
static struct device_attribute thresholdMcAttr =
   __ATTR(threshold_mC, 0664, thresholdMcShow, thresholdMcStore);





static const char * modeToStr(char m)
{
   switch (m)
   {
      case SIMTEMP_MODE_NORMAL: return "normal";
      case SIMTEMP_MODE_NOISY:  return "noisy";
      case SIMTEMP_MODE_RAMP:   return "ramp";
      default:                return "normal";
   }
}

static int modeFromStr(const char *buf, char*out)
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
   char m;

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
   char m;
   int ret = modeFromStr(buf, &m);
   if (ret)
      return ret;

   pr_info("SYSFS: mode set to %s\n", modeToStr(m));

   spin_lock(&sysfsLock);
   globalParam_Mode = m;

   /*
   if (SIMTEMP_MODE_NORMAL == globalParam_Mode)
   {
      noiseRange = NOISE_RANGE_NORMAL / 2;
   }
   if (SIMTEMP_MODE_NOISY == globalParam_Mode)
   {
      noiseRange = NOISE_RANGE_NOISY / 2;
   }
   */

   updateNoiseParams();

   spin_unlock(&sysfsLock);

   pr_info("SYSFS: Noise range = %d\n", noiseRange);

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
                    "   produced  = %llu\n"
                    "   delivered = %llu\n"
                    "   overruns  = %llu\n"
                    "   alerts    = %llu\n",
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
   sd->mode = SIMTEMP_MODE_DEFAULT;

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

static inline bool detectThresholdCross(s32 prev, s32 curr, s32 threshold)
{
   return ((prev < threshold) != (curr < threshold));
}

static void readSampleTimerCallback(struct work_struct *work)
{
   timerReset();
   tmprNewSample = tmprSampleReadADC();
   sampleNew.tempMC = tmprNewSample;
   sampleNew.timestampNS = ktime_get_ns();
   sampleNew.flags = FLAGS_NEW_SAMPLE;
   sampleNew.consecutive = simCnt;
   s32 lostSample = -1;
   
   spin_lock(&sysfsLock);
   globalParam_ThresholdAux = globalParam_Threshold;
   globalStats.produced++;                     // STATS: Increase amount of produced samples
   spin_unlock(&sysfsLock);


   threshEventAux = notFirstSample & detectThresholdCross(tmprPrevSample, tmprNewSample, globalParam_ThresholdAux);
   if (threshEventAux)
   {
      sampleNew.flags |= FLAGS_THRESHOLD_CROSS;
      spin_lock(&sysfsLock);
      globalStats.alerts++;                     // STATS: Increase amount of times temperature threshold was crossed
      spin_unlock(&sysfsLock);
}
   notFirstSample = true;
   tmprPrevSample = tmprNewSample;


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
   thresholdEvent |= threshEventAux;
   pr_info("TMR: thresholdEvent = %d\n",thresholdEvent);
   spin_unlock(&tmprSmplBuffLock);

   wake_up_interruptible(&simtempWait);

    
   if (lostSample != -1)
   {
      spin_lock(&sysfsLock);
      globalStats.overruns++;                     // STATS: Increase amount of lost samples
      spin_unlock(&sysfsLock);
      pr_info("Se perdió la %i \n",lostSample);

   }


   /*
   spin_lock(&tmprSmplBuffLock);
   tmprLastSample = sampleNew;
   spin_unlock(&tmprSmplBuffLock);
   */
    
}


/*********************************************************************/
/*********************************************************************/
/*************** TEMPERATURE SAMPLES HANDLING ************************/
/*********************************************************************/
/*********************************************************************/

static inline int randomSignedInt(int range)
{
   return ((int)(get_random_u32() % (2 * range + 1))) - range;
}




static s32 tmprSampleReadADC(void)
{
   spin_lock(&sysfsLock);
   auxMode = globalParam_Mode;
   auxThreshold = globalParam_Threshold;
   noiseRangeAux = noiseRange;
   tmprBaseAux = tmprBase;
   spin_unlock(&sysfsLock);

   switch (auxMode)
   {
      case SIMTEMP_MODE_RAMP:
         if (rampSample > tmprBaseAux + RAMP_RANGE)
         {
            rampSample = tmprBaseAux - RAMP_RANGE;
         }
         rampSample += RAMP_STEP;
         return rampSample;
      case SIMTEMP_MODE_NOISY:
      case SIMTEMP_MODE_NORMAL:
      default:
         return tmprBase + randomSignedInt(noiseRangeAux);
   }

}



/****************************************************************/
/******************* DEVICE READ OPERATION **********************/
/****************************************************************/


/******************* DEVICE OPEN OPERATION ***************************/

// open() allocates a small per-FD context to support short reads.
static int simtempOpen(struct inode *ino, struct file *f)
{
   pr_info("simtemp: open\n");

   /* allocate per-FD context (16B record + offsets) */
   struct simtempFctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
   if (!ctx)
      return -ENOMEM;
   ctx->outOff = 0;
   ctx->outValid = false;
   f->private_data = ctx;
   return 0;
}


/******************* DEVICE RELEASE OPERATION ************************/


static int simtempRelease(struct inode *ino, struct file *f)
{
    pr_info("simtemp: release\n");
    if (f->private_data)
    {
       // free the per-FD context on close, without altering your logs
       kfree(f->private_data);
       f->private_data = NULL;
    }
    return 0;
}


static ssize_t simtempRead(struct file *f, char __user *buf, size_t len, loff_t *off)
{

   // Retrieve per-FD context (for 16B record + short reads)
   // If there is a pending partial record, we should continue copying
   // it BEFORE consuming a new sample from the circular buffer
   struct simtempFctx *ctx = (struct simtempFctx *)f->private_data;
   size_t need = sizeof(struct simtempRecord16);
   if (!ctx)
      return -EINVAL;

   // If there is an unfinished 16B record, continue copying it now.
   if (ctx->outValid)
   {
      size_t toCopy = need - ctx->outOff;
      if (toCopy > len) 
      toCopy = len;
      if (copy_to_user(buf, ((u8 *)&ctx->out) + ctx->outOff, toCopy))
         return -EFAULT;

      ctx->outOff += toCopy;
      if (ctx->outOff == need)
      {
         // finished delivering the 16B record
         ctx->outValid = FALSE;
         ctx->outOff = 0;
      }
      return toCopy;
   }

   spin_lock(&tmprSmplBuffLock);
   if (cbTail == cbHead)  // If no new samples availble to consume
   {
      spin_unlock(&tmprSmplBuffLock);
      if (f->f_flags & O_NONBLOCK)
      {  // Return -EAGAIN if this call was a NONBLOCKING one
         return -EAGAIN;
      }

      /* Waits until there is a new sample available */
      int wret = wait_event_interruptible(simtempWait, cbTail != cbHead);
      if (wret)   /* Return -ERESTARTSYS if an interruption signal is received */
      {           /* like Ctr-C, kill, etc*/
         return wret;
      }
      /* Resume execution since there are new available sample(s) */
      spin_lock(&tmprSmplBuffLock);
   }
   tmprLastSample = tmprSmplCircBuff[cbTail];
   pr_info("READ: tmprLastSample.flags = %d\n", tmprLastSample.flags);
   if ((tmprLastSample.flags & FLAGS_THRESHOLD_CROSS) == FLAGS_THRESHOLD_CROSS)
   {
      thresholdEvent = false;
      pr_info("READ: thresholdEvent = 0\n");
   }
   
   cbTail = ((cbTail + 1) % TMPR_SMPL_CIRC_BUFF_SIZE);
   if (cbTail == cbHead)
   {
      newSampleFlag = FALSE;
   }
   spin_unlock(&tmprSmplBuffLock);
   
   spin_lock(&sysfsLock);
   globalStats.delivered++;                     // STATS: Increase amount of consumed samples
   spin_unlock(&sysfsLock);


   // freeze a 16B record for this FD (to support short reads).
   // we build the exact ABI record from your tmprLastSample.
   size_t toCopy;

   ctx->out.timestamp_ns = tmprLastSample.timestampNS;
   ctx->out.temp_mC      = tmprLastSample.tempMC;
   ctx->out.flags        = tmprLastSample.flags;

   ctx->outOff   = 0;
   ctx->outValid = TRUE;

   /* If the caller passed len=0 we just prepare the record and return 0. */
   if (len == 0)
      return 0;

   /* Deliver the first chunk of the 16B record now. The rest (if any)
      * will be delivered in subsequent read() calls without re-consuming
      * the circular buffer */
   toCopy = need;
   if (toCopy > len) toCopy = len;

   if (copy_to_user(buf, ((u8 *)&ctx->out) + ctx->outOff, toCopy))
      return -EFAULT;

   ctx->outOff += toCopy;
   if (ctx->outOff == need)
   {
      ctx->outValid = FALSE;
      ctx->outOff = 0;
   }
   return toCopy;


   
#if 0
   /* Debugg code returning ascii format */

   #define OUTPUT_SIZE 60
   ssize_t retVal = OUTPUT_SIZE;
   char auxStr[OUTPUT_SIZE] = {0};
   unsigned long notCopied;

   bool auxFlag1 = ((tmprLastSample.flags & FLAGS_NEW_SAMPLE) == FLAGS_NEW_SAMPLE);
   bool auxFlag2 = ((tmprLastSample.flags & FLAGS_THRESHOLD_CROSS) == FLAGS_THRESHOLD_CROSS);

   snprintf(auxStr, sizeof(auxStr), "%3d %5d, at: %llu, NF=%d, TF=%d\n", 
            tmprLastSample.consecutive, 
            tmprLastSample.tempMC, 
            tmprLastSample.timestampNS,
            auxFlag1,
            auxFlag2);
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
#endif
}



void resetGlobalModeParams(void)
{
   rampSample = tmprBase - RAMP_RANGE;
}

/*********************************************************************/
/*********************************************************************/
/****************************** POLL *********************************/
/*********************************************************************/
/*********************************************************************/


static __poll_t simtempPoll(struct file *file, poll_table *wait)
{
   __poll_t mask = 0;

   /* Register this wait queue whith poll */
   poll_wait(file, &simtempWait, wait);

   /* Reportar qué eventos están listos */
   spin_lock(&tmprSmplBuffLock);

   /* Datos listos para leer → POLLIN | POLLRDNORM */
   if (cbTail != cbHead)
      mask |= POLLIN | POLLRDNORM;

   /* Crossing threshold POLLPRI  */
   
   if (thresholdEvent)
   {
      pr_info("POLL: thresholdEvent = 1\n");
      mask |= POLLPRI;
      //thresholdEvent = false;
   }
   spin_unlock(&tmprSmplBuffLock);

   return mask;
}


/******************** TIMER INIT ********************************/

static void timerInit(void)
{
   INIT_DELAYED_WORK(&readSampleTimer, readSampleTimerCallback);
   spin_lock_init(&tmprSmplBuffLock);
   timerReset();
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
/********************  KERNEL DEVICE SETUP ***************************/
/*********************************************************************/
/*********************************************************************/


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
   updateNoiseParams();
   rampSample = globalParam_Threshold - RAMP_RANGE;
   pr_info("INIT: simtemp: platform driver registered\n");
   pr_info("INIT: globalParam_tmprTreshold = %d\n",globalParam_tmprTreshold);
   pr_info("INIT: tmprBase                 = %d\n",tmprBase);
   pr_info("INIT: noiseRange               = %d\n",noiseRange);

   return 0;
}


/******************* DEVICE EXIT OPERATION ************************/

static void __exit simtempExit(void)
{
   wake_up_interruptible_all(&simtempWait);
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
