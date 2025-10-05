#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jorge Sanchez");
MODULE_DESCRIPTION("miscdevice /dev/simtemp");
MODULE_VERSION("0.1");

// --- Operaciones mínimas del char device ---

char arreglito[20] = "0123456789ABCDEFG\n";

struct delayed_work readSampleTimer;


static void timerReset(void)
{
    schedule_delayed_work(&readSampleTimer, 2 * HZ);
}

static void readSampleTimerCallback(struct work_struct *work)
{
    static char simCnt = 0;
    printk(KERN_INFO "Timer ejecutado, generando muestra..., %i\n",simCnt++);
    timerReset();
}



static void timerInit(void)
{
    INIT_DELAYED_WORK(&readSampleTimer, readSampleTimerCallback);
    timerReset();
}



static void timerDeInit(void)
{
    cancel_delayed_work_sync(&readSampleTimer);
}



static ssize_t simtemp_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    // Por ahora no hay datos; regresamos "intenta de nuevo"
    copy_to_user(buf, arreglito, 17);

    return 17;

    buf[0] = arreglito[0];
    buf[1] = arreglito[1];
    buf[2] = arreglito[2];
    buf[3] = arreglito[3];
    buf[4] = arreglito[4];
    buf[5] = arreglito[5];
    buf[6] = arreglito[6];
    buf[7] = arreglito[7];
    buf[8] = arreglito[8];
    buf[9] = arreglito[9];
    buf[10] = arreglito[10];
    buf[11] = arreglito[011];
    buf[12] = arreglito[012];
    buf[13] = arreglito[013];
    buf[14] = arreglito[014];
    buf[15] = arreglito[015];
    buf[16] = arreglito[016];
    buf[17] = arreglito[017];

//    return -EAGAIN;
}

static int simtemp_open(struct inode *ino, struct file *f)
{
    pr_info("simtemp: open\n");
    return 0;
}

static int simtemp_release(struct inode *ino, struct file *f)
{
    pr_info("simtemp: release\n");
    return 0;
}

// Tabla de operaciones
static const struct file_operations simtemp_fops = {
    .owner   = THIS_MODULE,
    .read    = simtemp_read,
    .open    = simtemp_open,
    .release = simtemp_release,
};

// miscdevice crea /dev/simtemp automáticamente
static struct miscdevice simtemp_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "simtemp",
    .fops  = &simtemp_fops,
    .mode  = 0666,
};

static int __init simtemp_init(void)
{
    int ret = misc_register(&simtemp_miscdev);
    if (ret) {
        pr_err("simtemp: misc_register falló (%d)\n", ret);
        return ret;
    }
    timerInit();
    pr_info("simtemp: cargado. /dev/simtemp listo (v0.1)\n");
    return 0;
}

static void __exit simtemp_exit(void)
{
    timerDeInit();
    misc_deregister(&simtemp_miscdev);
    pr_info("simtemp: descargado.\n");
}

module_init(simtemp_init);
module_exit(simtemp_exit);
