/*
*
*   file: ws2812_drv.c
*   date: 2024-09-03
*   notice:
*		The ws2812 driver is suitable for RK356x, RK3588 and RV1106.
*   	The ws2812 driver can only control a signle RGB LED at a time.
*		For RV1106: WS2812 connected to pwm2m0 (gpio0a1), will blink green twice on boot.
*
*	usage:
*		1、Before compilation, you need to enable the corresponding macro definition based on the target chip.
*
*		2、APP example:

		#include <stdio.h>
		#include <string.h>
		#include <sys/types.h>
		#include <sys/stat.h>
		#include <fcntl.h>
		#include <unistd.h>
		#include <stdlib.h>

		struct ws2812_mes {
			unsigned int gpiochip;      // GPIO chip number for the data pin
			unsigned int gpionum;       // GPIO number for the data pin
			unsigned int lednum;        // The ordinal number of the LED on the strip to be controlled, starting from 1
			unsigned char color[3];     // color[0]:color[1]:color[2] R:G:B      
		};

		int main(int argc, char **argv)
		{
			struct ws2812_mes ws2812;
			int fd;

			ws2812.gpiochip = 3;       
			ws2812.gpionum  = 26;
			ws2812.lednum   = 1;
			ws2812.color[0] = 0xff;
			ws2812.color[1] = 0x00;
			ws2812.color[2] = 0x00;

			fd = open("/dev/ws2812", O_RDWR);

			write(fd, &ws2812, sizeof(struct ws2812_mes));

			close(fd);

			return 0;
		}
*
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>

#define DEV_NAME            			"ws2812"
#define LED_NUM_MAX						(20)
//#define DEBUG

// Enable the corresponding macro definition based on the target chip
//#define RK356x
//#define RK3588
#define RV1106

#ifdef RK356x
/* RK356x GPIO BASE */
#define GPIO0_BASE_ADDR					UL(0xFDD60000)
#define GPIO1_BASE_ADDR					UL(0xFE740000)
#define GPIO2_BASE_ADDR					UL(0xFE750000)
#define GPIO3_BASE_ADDR					UL(0xFE760000)
#define GPIO4_BASE_ADDR					UL(0xFE770000)
#endif
#ifdef RK3588
/* RK3588 GPIO BASE */
#define GPIO0_BASE_ADDR					UL(0xFD8A0000)
#define GPIO1_BASE_ADDR					UL(0xFEC20000)
#define GPIO2_BASE_ADDR					UL(0xFEC30000)
#define GPIO3_BASE_ADDR					UL(0xFEC40000)
#define GPIO4_BASE_ADDR					UL(0xFEC50000)
#endif
#ifdef RV1106
/* RV1106 GPIO BASE */
#define GPIO0_BASE_ADDR					UL(0xFF4D0000)
#define GPIO1_BASE_ADDR					UL(0xFF4E0000)
#define GPIO2_BASE_ADDR					UL(0xFF4F0000)
#define GPIO3_BASE_ADDR					UL(0xFF500000)
#define GPIO4_BASE_ADDR					UL(0xFF510000)
#endif

/* GPIO Level REG OFFSET */
#define GPIO_SWPORT_DR_L_OFFSET			(0x0000)
#define GPIO_SWPORT_DR_H_OFFSET			(0x0004)
/* GPIO Direction REG OFFSET */
#define GPIO_SWPORT_DDR_L_OFFSET		(0x0008)
#define GPIO_SWPORT_DDR_H_OFFSET		(0x000C)

static volatile unsigned int *GPIO_DIR_REG;
static volatile unsigned int *GPIO_LEVEL_REG;

/* ws2812_mes 结构体定义（用于用户空间通信） */
struct ws2812_mes {
    unsigned int gpiochip;      		// GPIO chip number for the data pin
    unsigned int gpionum;       		// GPIO number for the data pin
    unsigned int lednum;        		// The ordinal number of the LED on the strip to be controlled, starting from 1
    unsigned char color[3];     		// color[0]:color[1]:color[2] R:G:B     
};

static int major = 0;
static struct class *ws2812_class;
static int bit;
static unsigned int temp;

/* 开机闪烁配置：gpio0a1 (gpiochip=0, gpionum=1) */
#define BOOT_BLINK_GPIOCHIP		0
#define BOOT_BLINK_GPIONUM		1
#define BOOT_BLINK_LEDNUM		1
#define BOOT_BLINK_GREEN_VALUE	0xFF
#define BOOT_BLINK_DELAY_MS		200	/* 闪烁间隔200ms */

/**
 * ws2812_drv_open - 打开设备文件
 */
static int ws2812_drv_open(struct inode *node, struct file *file)
{
	return 0;
}

/**
 * ws2812_reset - 发送复位信号
 * 根据数据手册，复位码需要低电平时间大于50μs（典型值>280μs）
 */
static void ws2812_reset(void)
{
	/* Reset: Pull low for > 280us */
	temp &= (~(1 << bit));
	*GPIO_LEVEL_REG = temp;
	udelay(300);
}

/**
 * ws2812_write_frame_0 - 发送'0'码
 * 根据数据手册：
 * T0H: 0.25μs - 0.55μs (典型值 0.4μs)
 * T0L: 0.7μs - 1.0μs (典型值 0.85μs)
 */
static void ws2812_write_frame_0(void)
{
	/* T0H: Pull high for 0.25μs ~ 0.55μs (typical 0.4μs) */
	temp |= 1 << bit;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;

	/* T0L: Pull low for 0.7μs ~ 1.0μs (typical 0.85μs) */
	temp &= (~(1 << bit));
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
}

/**
 * ws2812_write_frame_1 - 发送'1'码
 * 根据数据手册：
 * T1H: 0.65μs - 0.95μs (典型值 0.8μs)
 * T1L: 0.3μs - 0.6μs (典型值 0.45μs)
 */
static void ws2812_write_frame_1(void)
{
	/* T1H: Pull high for 0.65μs ~ 0.95μs (typical 0.8μs) */
	temp |= 1 << bit;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;

	/* T1L: Pull low for 0.3μs ~ 0.6μs (typical 0.45μs) */
	temp &= (~(1 << bit));
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
	*GPIO_LEVEL_REG = temp;
}

/**
 * ws2812_write_byte - 发送一个字节数据
 * @byte: 要发送的字节数据
 * 从最高位开始发送，根据每一位的值调用相应的帧函数
 */
static void ws2812_write_byte(unsigned char byte)
{
	int i = 0;

	for(i = 0; i < 8; i++)
	{
		if((byte << i) & 0x80)
			ws2812_write_frame_1();
	  	else
			ws2812_write_frame_0();
	}
}

static ssize_t ws2812_drv_write(struct file *filp, const char __user * buf, size_t count, loff_t * ppos)
{
	int err;
	struct ws2812_mes ws2812_usr;
	int step;
	int i = 1;

	err = copy_from_user(&ws2812_usr, buf, sizeof(ws2812_usr));
	if(err != 0)
	{
		printk(KERN_ERR"get ws2812 struct err!\n");
		return err;
	}

#ifdef DEBUG
	printk("ws2812_usr.gpiochip : %d\n", ws2812_usr.gpiochip);
	printk("ws2812_usr.gpionum : %d\n", ws2812_usr.gpionum);
	printk("ws2812_usr.lednum : %d\n", ws2812_usr.lednum);
	printk("ws2812_usr.color[0] : %d\n", ws2812_usr.color[0]);
	printk("ws2812_usr.color[1] : %d\n", ws2812_usr.color[1]);
	printk("ws2812_usr.color[2] : %d\n", ws2812_usr.color[2]);
#endif

	if(ws2812_usr.gpiochip < 0 || ws2812_usr.gpiochip > 4)
	{
		printk(KERN_ERR"ws2812.gpiochip must >= 0 && <= 4\n");
		return -1;
	}

	if(ws2812_usr.gpionum < 0 || ws2812_usr.gpionum > 31)
	{
		printk(KERN_ERR"ws2812.gpionum must >= 0 && <= 31\n");
		return -1;
	}

	if(ws2812_usr.lednum < 1 || ws2812_usr.lednum > LED_NUM_MAX)
	{
		printk(KERN_ERR"ws2812.lednum must >= 1 && <= %d\n", LED_NUM_MAX);
		return -1;
	}
	
	/* step: 0-15 uses GPIO_SWPORT_DR_L_OFFSET, 16-31 uses GPIO_SWPORT_DR_H_OFFSET */
	step = ws2812_usr.gpionum / 16;
	if(ws2812_usr.gpionum < 16)
	{
		/* GPIO 0-15 use L register */
		if(ws2812_usr.gpiochip == 0)
		{
			GPIO_DIR_REG	= ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DDR_L_OFFSET, 4);
			GPIO_LEVEL_REG 	= ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DR_L_OFFSET, 4);
		}
		else
		{
#ifdef RV1106
			/* RV1106 GPIO address mapping */
			GPIO_DIR_REG	= ioremap(GPIO0_BASE_ADDR + (0x10000 * ws2812_usr.gpiochip) + GPIO_SWPORT_DDR_L_OFFSET, 4);
			GPIO_LEVEL_REG	= ioremap(GPIO0_BASE_ADDR + (0x10000 * ws2812_usr.gpiochip) + GPIO_SWPORT_DR_L_OFFSET, 4);
#else
			GPIO_DIR_REG	= ioremap(GPIO1_BASE_ADDR + (0x10000*(ws2812_usr.gpiochip-1)) + GPIO_SWPORT_DDR_L_OFFSET, 4);
			GPIO_LEVEL_REG	= ioremap(GPIO1_BASE_ADDR + (0x10000*(ws2812_usr.gpiochip-1)) + GPIO_SWPORT_DR_L_OFFSET, 4);
#endif
		}
	}
	else
	{
		/* GPIO 16-31 use H register */
		if(ws2812_usr.gpiochip == 0)
		{
			GPIO_DIR_REG	= ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DDR_H_OFFSET, 4);
			GPIO_LEVEL_REG 	= ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DR_H_OFFSET, 4);
		}
		else
		{
#ifdef RV1106
			/* RV1106 GPIO address mapping */
			GPIO_DIR_REG	= ioremap(GPIO0_BASE_ADDR + (0x10000 * ws2812_usr.gpiochip) + GPIO_SWPORT_DDR_H_OFFSET, 4);
			GPIO_LEVEL_REG	= ioremap(GPIO0_BASE_ADDR + (0x10000 * ws2812_usr.gpiochip) + GPIO_SWPORT_DR_H_OFFSET, 4);
#else
			GPIO_DIR_REG	= ioremap(GPIO1_BASE_ADDR + (0x10000*(ws2812_usr.gpiochip-1)) + GPIO_SWPORT_DDR_H_OFFSET, 4);
			GPIO_LEVEL_REG	= ioremap(GPIO1_BASE_ADDR + (0x10000*(ws2812_usr.gpiochip-1)) + GPIO_SWPORT_DR_H_OFFSET, 4);
#endif
		}
	}

#ifdef DEBUG
	printk("GPIO_DIR_REG : 0x%p\n", GPIO_DIR_REG);
	printk("GPIO_LEVEL_REG : 0x%p\n", GPIO_LEVEL_REG);
#endif
	
	if(GPIO_LEVEL_REG == NULL || GPIO_DIR_REG == NULL)
	{
		printk(KERN_ERR"GPIO_LEVEL_REG or GPIO_DIR_REG is NULL\n");
		return -1;
	}

	/* Calculate bit position: 0-15 for L register, 0-15 for H register */
	bit = ws2812_usr.gpionum % 16;

	/* Set GPIO Multiplexing */
	/* In the relevant multiplexing registers, they are all configured as GPIO mode by default, so the GPIO multiplexing configuration is omitted here */

	/* Set GPIO Mode to Output */
	temp = *GPIO_DIR_REG;
	temp |= 1 << (16 + bit);
	temp |= 1 << bit;
	*GPIO_DIR_REG = temp;

	/* Set initial GPIO level to high */
	temp = *GPIO_LEVEL_REG;
	temp |= 1 << (16 + bit);
	temp |= 1 << bit;
	*GPIO_LEVEL_REG = temp;

	ws2812_reset();
	
	for(i = 1; i < ws2812_usr.lednum; i++)
	{
		ws2812_write_byte(0x00);		
		ws2812_write_byte(0x00);		
		ws2812_write_byte(0x00);		
	}
	ws2812_write_byte(ws2812_usr.color[1]);		// color G
	ws2812_write_byte(ws2812_usr.color[0]);		// color R
	ws2812_write_byte(ws2812_usr.color[2]);		// color B

	ws2812_reset();

#ifdef DEBUG
	printk("ws2812 write over!\n");
#endif

	return 0;
}

static int ws2812_drv_close(struct inode *node, struct file *file)
{	
	iounmap(GPIO_LEVEL_REG);
	iounmap(GPIO_DIR_REG);

	GPIO_LEVEL_REG = NULL;
	GPIO_DIR_REG = NULL;

	return 0;
}

static struct file_operations ws2812_fops = {
	.owner = THIS_MODULE,
	.open = ws2812_drv_open,
	.release = ws2812_drv_close,
	.write = ws2812_drv_write,
};

/**
 * ws2812_boot_blink - 开机闪烁两次绿色
 * 在模块加载时自动执行，闪烁两次绿色后熄灭
 */
static void ws2812_boot_blink(void)
{
	struct ws2812_mes ws2812_blink;
	int step;
	volatile unsigned int *blink_gpio_dir_reg = NULL;
	volatile unsigned int *blink_gpio_level_reg = NULL;
	unsigned int blink_temp;
	int blink_bit;
	int i;

	/* 配置闪烁参数：gpio0a1，绿色 */
	ws2812_blink.gpiochip = BOOT_BLINK_GPIOCHIP;
	ws2812_blink.gpionum = BOOT_BLINK_GPIONUM;
	ws2812_blink.lednum = BOOT_BLINK_LEDNUM;
	ws2812_blink.color[0] = 0x00;	/* R = 0 */
	ws2812_blink.color[1] = BOOT_BLINK_GREEN_VALUE;	/* G = 0xFF */
	ws2812_blink.color[2] = 0x00;	/* B = 0 */

	/* 映射GPIO寄存器 */
	step = ws2812_blink.gpionum / 16;
	if(ws2812_blink.gpionum < 16)
	{
		blink_gpio_dir_reg = ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DDR_L_OFFSET, 4);
		blink_gpio_level_reg = ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DR_L_OFFSET, 4);
	}
	else
	{
		blink_gpio_dir_reg = ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DDR_H_OFFSET, 4);
		blink_gpio_level_reg = ioremap(GPIO0_BASE_ADDR + GPIO_SWPORT_DR_H_OFFSET, 4);
	}

	if(blink_gpio_level_reg == NULL || blink_gpio_dir_reg == NULL)
	{
		printk(KERN_ERR"ws2812 boot blink: GPIO register map failed!\n");
		return;
	}

	blink_bit = ws2812_blink.gpionum % 16;

	/* 设置GPIO为输出模式 */
	blink_temp = *blink_gpio_dir_reg;
	blink_temp |= 1 << (16 + blink_bit);
	blink_temp |= 1 << blink_bit;
	*blink_gpio_dir_reg = blink_temp;

	/* 设置初始GPIO电平为高 */
	blink_temp = *blink_gpio_level_reg;
	blink_temp |= 1 << (16 + blink_bit);
	blink_temp |= 1 << blink_bit;
	*blink_gpio_level_reg = blink_temp;

	/* 保存原始GPIO寄存器指针，临时使用 */
	GPIO_DIR_REG = blink_gpio_dir_reg;
	GPIO_LEVEL_REG = blink_gpio_level_reg;
	bit = blink_bit;
	temp = blink_temp;

	/* 闪烁两次绿色 */
	for(i = 0; i < 2; i++)
	{
		/* 发送复位信号 */
		ws2812_reset();
		
		/* 发送绿色数据（GRB顺序：G=0xFF, R=0x00, B=0x00） */
		ws2812_write_byte(ws2812_blink.color[1]);	/* G */
		ws2812_write_byte(ws2812_blink.color[0]);	/* R */
		ws2812_write_byte(ws2812_blink.color[2]);	/* B */
		
		/* 发送复位信号锁定颜色 */
		ws2812_reset();
		
		/* 等待200ms */
		mdelay(BOOT_BLINK_DELAY_MS);
	}

	/* 熄灭LED：发送全0数据 */
	ws2812_reset();
	ws2812_write_byte(0x00);	/* G = 0 */
	ws2812_write_byte(0x00);	/* R = 0 */
	ws2812_write_byte(0x00);	/* B = 0 */
	ws2812_reset();

	/* 取消映射 */
	iounmap(blink_gpio_level_reg);
	iounmap(blink_gpio_dir_reg);
	
	/* 清空临时指针 */
	GPIO_LEVEL_REG = NULL;
	GPIO_DIR_REG = NULL;

	printk(KERN_INFO"ws2812 boot blink: Green LED blinked twice and turned off\n");
}

/**
 * ws2812_init - 模块初始化函数
 * 注册字符设备并执行开机闪烁
 */
static __init int ws2812_init(void)
{
	printk(KERN_INFO"Load the ws2812 module successfully!\n");

	major = register_chrdev(0, "rk_ws2812", &ws2812_fops);  

	ws2812_class = class_create(THIS_MODULE, "rk_ws2812_class");
	if (IS_ERR(ws2812_class)) {
		printk(KERN_ERR"%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
		unregister_chrdev(major, "rk_ws2812");
		return PTR_ERR(ws2812_class);
	}

	device_create(ws2812_class, NULL, MKDEV(major, 0), NULL, "ws2812");

#ifdef RV1106
	/* RV1106: 开机闪烁两次绿色 */
	ws2812_boot_blink();
#endif

	return 0;
}
module_init(ws2812_init);

static __exit void ws2812_exit(void)
{
	printk(KERN_INFO"the ws2812 module has been remove!\n");

	device_destroy(ws2812_class, MKDEV(major, 0));
	class_destroy(ws2812_class);
	unregister_chrdev(major, "rk_ws2812");

	if(GPIO_LEVEL_REG != NULL)
		iounmap(GPIO_LEVEL_REG);
	if(GPIO_DIR_REG != NULL)
		iounmap(GPIO_DIR_REG);
	GPIO_LEVEL_REG = NULL;
	GPIO_DIR_REG = NULL;
}
module_exit(ws2812_exit);

MODULE_AUTHOR("Cohen0415");
MODULE_LICENSE("GPL");
