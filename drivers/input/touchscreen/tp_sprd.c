#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/bug.h>
#include <linux/input.h>
#include <linux/slab.h>

#include <linux/ctype.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/adi_hal_internal.h>
#include <mach/irqs.h>
#include <mach/regs_ana.h>
#include <mach/bits.h>

//#define TP_DEBUG
#ifdef  TP_DEBUG
#define TP_PRINT  printk
#else
#define TP_PRINT(...)
#endif

#define SCI_PASSERT(condition, format...)  \
	do {		\
		if(!(condition)) { \
			pr_err("function :%s\r\n", __FUNCTION__);\
			BUG();	\
		} \
	}while(0)

#define TPC_REG_BASE                (SPRD_MISC_BASE+0x280)
#define TPC_CTRL                    (TPC_REG_BASE + 0x0000)
#define TPC_SAMPLE_CTRL0            (TPC_REG_BASE + 0x0004)
#define TPC_SAMPLE_CTRL1            (TPC_REG_BASE + 0x0008)
#define TPC_BOUNCE_CTRL             (TPC_REG_BASE + 0x000C)
#define TPC_FILTER_CTRL             (TPC_REG_BASE + 0x0010)
#define TPC_CALC_CTRL               (TPC_REG_BASE + 0x0014)
#define TPC_CALC_X_COEF_A           (TPC_REG_BASE + 0x0018)
#define TPC_CALC_X_COEF_B           (TPC_REG_BASE + 0x001C)
#define TPC_CALC_Y_COEF_A           (TPC_REG_BASE + 0x0020)
#define TPC_CALC_Y_COEF_B           (TPC_REG_BASE + 0x0024)
#define TPC_INT_EN                  (TPC_REG_BASE + 0x0028)
#define TPC_INT_STS                 (TPC_REG_BASE + 0x002C)
#define TPC_INT_RAW                 (TPC_REG_BASE + 0x0030)
#define TPC_INT_CLR                 (TPC_REG_BASE + 0x0034)
#define TPC_BUF_CTRL                (TPC_REG_BASE + 0x0038)
#define TPC_X_DATA                  (TPC_REG_BASE + 0x003C)
#define TPC_Y_DATA                  (TPC_REG_BASE + 0x0040)
#define TPC_Z_DATA                  (TPC_REG_BASE + 0x0044)

//TPC_CTRL BIT map
#define TPC_STOP_BIT                BIT_5
#define TPC_RUN_BIT                 BIT_4
#define TPC_TPC_MODE_BIT            BIT_3
#define TPC_PEN_REQ_POL_BIT         BIT_1
#define TPC_EN_BIT                  BIT_0

#define TPC_PRESCALE_OFFSET         0x08
#define TPC_PRESCALE_MSK            (0xFF << TPC_PRESCALE_OFFSET)

//TPC_SAMPLE_CTRL0 BIT map
#define TPC_SAMPLE_INTERVAL_OFFSET    0
#define TPC_SAMPLE_INTERVAL_MSK      (0xFF << TPC_SAMPLE_INTERVAL_OFFSET)
#define TPC_POINT_INTERVAL_OFFSET     8
#define TPC_POINT_INTERVAL_MSK       (0xFF << TPC_POINT_INTERVAL_OFFSET)

//TPC_SAMPLE_CTRL1 BIT map
#define TPC_DATA_INTERVAL_OFFSET     0
#define TPC_DATA_INTERVAL_MSK        (0xFFF << TPC_DATA_INTERVAL_OFFSET)
#define TPC_SAMPLE_NUM_OFFSET        12
#define TPC_SAMPLE_NUM_MSK           (0xF << TPC_SAMPLE_NUM_OFFSET)

//TPC_BOUNCE_CTRL BIT map
#define TPC_DEBOUNCE_EN_BIT         BIT_0
#define TPC_DEBOUNCE_NUM_OFFSET     1
#define TPC_DEBOUNCE_NUM_MSK        (0xFF << TPC_DEBOUNCE_NUM_OFFSET)


//TPC_FILTER_CTRL BIT map
#define TPC_FILTER_EN_BIT           BIT_0
#define TPC_FILTER_MODE_BIT         BIT_1
#define TPC_FILTER_MODE_OFFSET      2
#define TPC_FILTER_MODE_MSK         (0xF << TPC_FILTER_MODE_OFFSET)

//TPC_BUF_CTRL BIT map
#define TPC_BUF_EMP_BIT         BIT_5
#define TPC_BUF_FULL            BIT_4
#define TPC_BUF_LENGTH_OFFSET   0
#define TPC_BUF_LENGTH_MSK      (0xF << TPC_BUF_LENGTH_OFFSET)

#define TPC_DONE_IRQ_MSK_BIT    BIT_2
#define TPC_UP_IRQ_MSK_BIT      BIT_1
#define TPC_DOWN_IRQ_MSK_BIT    BIT_0

#define ADC_REG_BASE                (SPRD_MISC_BASE+0x300)
#define ADC_CTRL                    (ADC_REG_BASE + 0x0000)
#define ADC_CS                      (ADC_REG_BASE + 0x0004)
#define ADC_TPC_CH_CTRL             (ADC_REG_BASE + 0x0008)
#define ADC_DAT                     (ADC_REG_BASE + 0x00C)
#define ADC_INT_EN                  (ADC_REG_BASE + 0x0010)
#define ADC_INT_CLR                 (ADC_REG_BASE + 0x0014)
#define ADC_INT_STAT                (ADC_REG_BASE + 0x0018)
#define ADC_INT_SRC                 (ADC_REG_BASE + 0x001C)

///ADC_CTRL
#define ADC_STATUS_BIT                      BIT_4
#define ADC_TPC_CH_ON_BIT                   BIT_2
#define SW_CH_ON_BIT                        BIT_1
#define ADC_EN_BIT                          BIT_0

///ADC_CS bit map
#define ADC_SCALE_BIT                       BIT_4
#define ADC_CS_BIT_MSK                      0x0F

#define ADC_SCALE_3V       0
#define ADC_SCALE_1V2      1

#define TPC_BUF_LENGTH      4
#define TPC_DELTA_DATA      15
#define CLEAR_TPC_INT(msk) \
    do{ \
        ANA_REG_SET(TPC_INT_CLR, msk);\
        while(ANA_REG_GET(TPC_INT_RAW) & msk);    \
    }while(0)

#define X_MIN	0
#define X_MAX	0x3ff
#define Y_MIN	0
#define Y_MAX	0x3ff

#define TPC_CHANNEL_X       2
#define TPC_CHANNEL_Y       3
#define ADC_CH_MAX_NUM      8

#define SCI_TRUE	1
#define SCI_FALSE	0

int a= 35337;
int b= -305;
int c= -8622664;
int d= -105;
int e= 43086;
int f= -8445728;
int g= 65536;

typedef union
{
    struct
    {
        int16_t y;
        int16_t x;
    } data;
    int32_t dwValue;
} TPC_DATA_U;

struct sprd_tp{
	struct input_dev *input;
	
	unsigned int irq;
	unsigned int is_first_down;

	struct timer_list timer;
	TPC_DATA_U tp_data;
};
static void ADC_Init(void)
{
//8810	TODO
	ANA_REG_OR (ANA_AGEN, (BIT_5 | BIT_13 | BIT_14)); //AUXAD controller APB clock enable
//	ANA_REG_OR (ANA_CLK_CTL, ACLK_CTL_AUXAD_EN | ACLK_CTL_AUXADC_EN);//enable AUXAD clock generation
//	ANA_REG_OR (ADC_CTRL, ADC_EN_BIT);//ADC module enable

}
static void ADC_SetScale (uint32_t scale)
{
    if (scale == ADC_SCALE_1V2)
    {
        //Set ADC small scalel
        ANA_REG_AND (ADC_CS, ~ADC_SCALE_BIT);
    }
    else if ( (scale == ADC_SCALE_3V))
    {
        //Set ADC large scalel
        ANA_REG_OR (ADC_CS, ADC_SCALE_BIT);
    }
}
static void ADC_SetCs (uint32_t source)
{
    SCI_PASSERT( (source < ADC_CH_MAX_NUM), ("error: source =%d",source));
    ANA_REG_MSK_OR (ADC_CS, source, ADC_CS_BIT_MSK);
   
}
 void ADC_OpenTPC (void)
{
    //Open TPC channel
    ANA_REG_OR (ADC_CTRL, ADC_TPC_CH_ON_BIT);

    TP_PRINT( "ADC:ADC_OpenTPC\n");

}
 void ADC_CloseTPC (void)
{
    //Close TPC channel
    ANA_REG_AND (ADC_CTRL, ~ADC_TPC_CH_ON_BIT);

    TP_PRINT( "ADC:ADC_CloseTPC\n");
}
static void tp_init(void)
{
	ANA_REG_OR (ANA_AGEN, BIT_12 | BIT_4);//Touch panel controller APB clock enable

   	 //Enable TPC module
    ANA_REG_OR (TPC_CTRL, TPC_EN_BIT);
    //Config pen request polarity
    ANA_REG_AND (TPC_CTRL, ~TPC_PEN_REQ_POL_BIT);

    //Config tpc mode
    ANA_REG_AND ( TPC_CTRL, ~TPC_TPC_MODE_BIT);

    //500KHz
    ANA_REG_MSK_OR (TPC_CTRL, (0x0D<< TPC_PRESCALE_OFFSET), TPC_PRESCALE_MSK);//prescale

    //Config ADC channel
    ADC_SetScale (ADC_SCALE_3V);

    //Config TPC sample properity
    ANA_REG_SET (TPC_SAMPLE_CTRL0,
                (0x10 << TPC_SAMPLE_INTERVAL_OFFSET) | (0x30 <<TPC_POINT_INTERVAL_OFFSET));//(0x80 << 8) | 0x14
    ANA_REG_SET (TPC_SAMPLE_CTRL1,
                 (0x200 << TPC_DATA_INTERVAL_OFFSET) | (4 <<TPC_SAMPLE_NUM_OFFSET));// (4 << 12)| 0x400

    //Config TPC filter properity
    //Config TPC debounce properity
    ANA_REG_SET (TPC_BOUNCE_CTRL,
                 TPC_DEBOUNCE_EN_BIT | (5 << TPC_DEBOUNCE_NUM_OFFSET));
    //Config TPC buffer length
    ANA_REG_MSK_OR (TPC_BUF_CTRL, (TPC_BUF_LENGTH << TPC_BUF_LENGTH_OFFSET), TPC_BUF_LENGTH_MSK);
    //Clear TPC interrupt
    CLEAR_TPC_INT ( (TPC_DONE_IRQ_MSK_BIT | TPC_UP_IRQ_MSK_BIT |TPC_DOWN_IRQ_MSK_BIT));

}
static inline void tp_run (void)
{
    //Enable TPC module
    ANA_REG_OR (TPC_CTRL, TPC_RUN_BIT);

    CLEAR_TPC_INT (TPC_DONE_IRQ_MSK_BIT);

    //Set ADC cs to TPC_CHANNEL_X
    ADC_SetCs (TPC_CHANNEL_X);

    //Set ADC scale
    ADC_SetScale (ADC_SCALE_3V);

    //Open ADC channel
    ADC_OpenTPC();

}
static inline void tp_stop (void)
{
    //Disable TPC module
    ANA_REG_OR (TPC_CTRL, TPC_STOP_BIT);
    //Close ADC channel
    ADC_CloseTPC();

}
static int tp_fetch_data (TPC_DATA_U *tp_data)
{
    uint16_t i;
    uint16_t buf_status = 0;
    TPC_DATA_U pre_data,cur_data;    
    int delta_x, delta_y;
    int result = SCI_TRUE;

    buf_status = ANA_REG_GET (TPC_BUF_CTRL);

    //TPC data buffer is empty
    if (! (buf_status & TPC_BUF_FULL))
    {
        TP_PRINT ("func[%s]: buffer not full!buf_status=0x%x\n",__FUNCTION__,buf_status);
        return SCI_FALSE;
    }

    pre_data.dwValue = 0;

    //Fecth data from TPC data buffer
    for (i=0; i<TPC_BUF_LENGTH; i++)
    {
        //Get data from buffer
        cur_data.data.x  = ANA_REG_GET (TPC_X_DATA);
        cur_data.data.y  = ANA_REG_GET (TPC_Y_DATA);
        
        if(cur_data.data.x <= X_MIN || cur_data.data.y >= Y_MAX)
                return SCI_FALSE;
            

        TP_PRINT("x=%d,y=%d\n",cur_data.data.x,cur_data.data.y);
        if (pre_data.dwValue != 0)
        {
            delta_x = pre_data.data.x - cur_data.data.x;
            delta_x = (delta_x > 0) ? delta_x : (- delta_x);
            delta_y = pre_data.data.y - cur_data.data.y;
            delta_y = (delta_y > 0) ? delta_y : (- delta_y);

            //TP_PRINT("delta_x=%d,delta_y=%d\n",delta_x,delta_y);
            
            if ( (delta_x + delta_y) >= TPC_DELTA_DATA)
            {
                //TP_PRINT ("func[%s]: fetch data failed !delta_x = %d delta_y = %d\n",__FUNCTION__,delta_x, delta_y);
                result  = SCI_FALSE;
            }
        }

        pre_data.dwValue = cur_data.dwValue;

    }

    //Get the last data
    tp_data->dwValue = cur_data.dwValue;

    return result;

}

static irqreturn_t tp_irq(int irq, void *dev_id)
{
	struct sprd_tp *tp=(struct sprd_tp *)dev_id;
	uint32_t int_status;
    int32_t xd,yd;
    int32_t xl,yl;
    
   	TP_PRINT ("TP: enter tp_irq!\n");
	
	int_status = ANA_REG_GET (TPC_INT_STS);

    //Pen Down interrtup
    if (int_status & TPC_DOWN_IRQ_MSK_BIT)
    {
        TP_PRINT("DOWN\n");
        tp->is_first_down = 1;

        //Clear down interrupt*/
        CLEAR_TPC_INT (TPC_DOWN_IRQ_MSK_BIT); 
        //Run TPC
        tp_run();

        //enable done interrupt
        ANA_REG_OR (TPC_INT_EN, TPC_DONE_IRQ_MSK_BIT);
		
        return IRQ_HANDLED;
    }
    //Pen Up interrupt
    else if (int_status & TPC_UP_IRQ_MSK_BIT)
    {
        TP_PRINT("UP\n");
        
        //disable interrupt
        ANA_REG_AND (TPC_INT_EN, ~TPC_DONE_IRQ_MSK_BIT);
        //Clear up interrupt*/
        CLEAR_TPC_INT ( (TPC_UP_IRQ_MSK_BIT|TPC_DONE_IRQ_MSK_BIT));
        //Stop TPC
        tp_stop();

	    input_report_key(tp->input, BTN_TOUCH, 0);
	    input_report_abs(tp->input, ABS_PRESSURE, 0);
	    input_sync(tp->input);
    }
    //Sample Done interrupt
    else if (int_status & TPC_DONE_IRQ_MSK_BIT)
    {
            TP_PRINT("DONE\n");
            //trace_printk("DONE\n");
            //Clear done interrupt*/
            CLEAR_TPC_INT (TPC_DONE_IRQ_MSK_BIT);

            //Get data from TP buffer
            if (tp_fetch_data(&(tp->tp_data))) {
                    //TP_PRINT("fetch data finish\n");
                    //xd=765-tp->tp_data.data.x;
                    //yd=816-tp->tp_data.data.y;
                    xd=X_MAX-tp->tp_data.data.x;
                    yd=Y_MAX-tp->tp_data.data.y;
                    if(0 == a+b+c+d+e+f+g){
                      input_report_abs(tp->input, ABS_X, xd);
                      input_report_abs(tp->input, ABS_Y, yd);
                    }else{
                      xl=(a*xd+b*yd+c)/g;
                      yl=(d*xd+e*yd+f)/g;
                      xl=(xl+20)*3;
                      yl=(yl+15)*2;
                      TP_PRINT("xd=%d,yd=%d\n",xd,yd);
                      TP_PRINT("xl=%d,yl=%d\n",xl,yl);
                      input_report_abs(tp->input, ABS_X, xl);
                      input_report_abs(tp->input, ABS_Y, yl);
                    }
                    input_report_abs(tp->input, ABS_PRESSURE,1);
                    input_report_key(tp->input, BTN_TOUCH, 1);
                    input_sync(tp->input);
                    //TP_PRINT("report finish\n");

            }
            else{
                    TP_PRINT("func[%s]: done interrupt rise,but can not fetch data!\n",__FUNCTION__);
                    //ANA_REG_AND (TPC_INT_EN, ~TPC_DONE_IRQ_MSK_BIT);
                    return IRQ_HANDLED;
            }
            //ANA_REG_AND (TPC_INT_EN, ~TPC_DONE_IRQ_MSK_BIT);
    }
    //Error occured
    else{
        printk("func[%s]: uncundefined irq\n",__FUNCTION__);
    }
	
	return IRQ_HANDLED;
}
#if 0
static int tp_proc_read(char *page, char **start, off_t off, int count, 
	int *eof, void *data)
{
	int len;
    if(off >0){
            *eof=1;
            return 0;
    }
	len = sprintf(page, "%d %d %d %d %d %d %d\n",a,b,c,d,e,f,g);
	return len;
}

#define PARA_LEN	7
static int tp_proc_write(struct file *file, const char __user *buf, 
	unsigned long len, void *data)
{
	char tp_buf[len + 1];
	long para[PARA_LEN];
	int num;
	char *endp,*startp;

	memset(tp_buf, 0, len + 1);

	if (copy_from_user(tp_buf, buf, len))
		return -EFAULT;
	
	startp = endp =tp_buf;
	
	num=0;
	do{
		para[num++] =simple_strtol(startp, &endp, 10);
		
		if(endp){
            endp++;	
		    startp=endp;
        }
		
	}while( num< 7 );
    
	a= (int)para[0];
    b= (int)para[1];
	c =(int)para[2];
    d= (int)para[3];
    e= (int)para[4];
    f= (int)para[5];
    g= (int)para[6];

	return 0;

}

static int tp_create_proc(void)
{
	struct proc_dir_entry *tp_entry;

	tp_entry = create_proc_entry("tp_info", 0666, NULL);  //creat /proc/tp_info
	if (!tp_entry) {
		printk(KERN_INFO"can not create tp proc entry\n");
		return -ENOMEM;
	}
	
	tp_entry->read_proc = tp_proc_read;
	tp_entry->write_proc = tp_proc_write;

	return 0;
}

static void tp_remove_proc(void)
{
	remove_proc_entry("tp_info", NULL);  //remove /proc/tp_info

}
#endif
static ssize_t tp_sysfs_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int len;
    len = sprintf(buf, "%d %d %d %d %d %d %d\n",a,b,c,d,e,f,g);
	return len;
}
#define PARA_LEN	7
static ssize_t tp_sysfs_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	long para[PARA_LEN];
	int num;
	char *endp,*startp;

	startp = endp = buf;
	
	num=0;
	while(*startp && num < 7){
		
		para[num++] =simple_strtol(startp, &endp, 10);		
		if(endp){
           endp++;	
		   startp=endp;
        }		
	}
    
	a= (int)para[0];
    b= (int)para[1];
	c =(int)para[2];
    d= (int)para[3];
    e= (int)para[4];
    f= (int)para[5];
    g= (int)para[6];

	return size;
}

static DEVICE_ATTR(tp, 0666, tp_sysfs_show, tp_sysfs_store); 
/*
 * The functions for inserting/removing us as a module.
 */
static int __init sprd_tp_probe(struct platform_device *pdev)
{
	struct sprd_tp *tp;
	struct input_dev *input_dev;
	int ret;
	
	tp = kzalloc(sizeof(struct sprd_tp), GFP_KERNEL);
	if (!tp){ 
		printk("func[%s]:kzalloc failed!\n",__FUNCTION__);
		return  -ENOMEM;
	}
	
	tp->irq = IRQ_ANA_TPC_INT;	

	TP_PRINT("before enter input_allocate_device\n");
    input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		goto err_alloc;
	}
    
	tp->input=input_dev;
	tp->input->name = "sprd_touch_screen";
	tp->input->phys = "TP";
	tp->input->id.bustype = BUS_HOST;
	tp->input->id.vendor = 0x8800;
	tp->input->id.product = 0xCAFE;
	tp->input->id.version = 0x0001;
	tp->input->dev.parent = &pdev->dev;

	tp->input->evbit[0] = tp->input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	tp->input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(tp->input, ABS_X, X_MIN, X_MAX, 0, 0);
	input_set_abs_params(tp->input, ABS_Y, Y_MIN, Y_MAX, 0, 0);
	input_set_abs_params(tp->input, ABS_PRESSURE, 0, 1, 0, 0);
	/*init adc*/
	ADC_Init();
	/*init tpc*/
	tp_init();

	TP_PRINT("before enter request_irq\n");
	ret = request_irq(tp->irq,tp_irq, IRQF_DISABLED, "touch_screen", tp);
	if (ret != 0) {
		printk("func[%s]: Could not allocate tp IRQ!\n",__FUNCTION__);
		ret =  -EIO;
		goto err_irq;
	}
    
	TP_PRINT("before enter input_register_device\n");
	/* All went ok, so register to the input system */
	ret = input_register_device(tp->input);	
	if(ret) {
		printk("func[%s]: Could not register input device(touchscreen)!\n",__FUNCTION__);
		ret = -EIO;
		goto err_reg;
	}
	platform_set_drvdata(pdev, tp);
#if 0
	if(tp_create_proc())
		printk("create touch panel proc file failed!\n");
#endif
 	if(device_create_file(&tp->input->dev, &dev_attr_tp)) {
 		printk("create touch panel sysfs file failed!\n");
 	}


      ANA_REG_OR(TPC_INT_EN,(TPC_UP_IRQ_MSK_BIT |TPC_DOWN_IRQ_MSK_BIT));	

	return 0;

err_reg:
	input_free_device(tp->input);	
err_irq:
	free_irq(tp->irq, tp);	
err_alloc:
	kfree(tp);

	return ret;
}

static int sprd_tp_remove(struct platform_device *pdev)
{
	struct sprd_tp *tp = platform_get_drvdata(pdev);
	
	platform_set_drvdata(pdev, NULL);
	
	input_unregister_device(tp->input);
	
	free_irq(tp->irq, tp);
	
	kfree(tp);
#if 0
	tp_remove_proc();
#endif
	device_remove_file(&tp->input->dev, &dev_attr_tp);

	return 0;
}


static struct platform_driver sprd_tp_driver = {
       .probe          = sprd_tp_probe,
       .remove        = sprd_tp_remove,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= "sprd-tp",
	},
};

static int __init tp_sprd_init(void)
{
	printk(KERN_INFO"TP:SC8800G Touch Panel driver $Revision:1.0 $\n");
	
	return platform_driver_register(&sprd_tp_driver);	
}

static void __exit tp_sprd_exit(void)
{
	printk(KERN_INFO "TP:SC8800G Touch Panel remove!\n");
	platform_driver_unregister(&sprd_tp_driver);
}

module_init(tp_sprd_init);
module_exit(tp_sprd_exit);

MODULE_DESCRIPTION("SC8800 Touch Panel driver");
MODULE_AUTHOR("Ke Wang, <ke.wang@spreadtrum.com>");
MODULE_LICENSE("GPL");
