/*
	De hoan thien 1 driver su dung sysfs can thuc hien nhung cong viec sau:
	-  Dang ki 1 driver: su dung module_init() -> platform_driver_register()
	-  Binding driver do voi device tuong ung: thong qua .probe operation
	-  Tao sysfs de giao tiep voi tang user

	Ben phia device:
	-  Khai bao trong arch/arm/plat-s5p4418/nanopi2/device.c:
	-  struct platform_device led_sysfs_device gom nhung resource nao
	-  Dang ki no bang ham platform_device_register() trong nxp_board_devices_register()
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "led-sysfs"


/* Probe Function */
static int led_sysfs_probe(struct platform_device *pdev)
{

}

/* Remove Function */
static int led_sysfs_remove(struct platform_device *pdev)
{

}

static struct platform_driver sample_driver = {
	.probe		= led_sysfs_probe,
	.remove		= led_sysfs_remove,
	.driver		= {
		.name   = DRIVER_NAME,
		.owner 	= THIS_MODULE,
		//.of_match_table = of_led_sysfs_match,
	},
};

static int __init khoi_tao_led(void)
{	
	printk(KERN_ALERT"\n ------------ DungLX: khoi tao led driver ---------------");
	platform_driver_register(&sample_driver);
	return 0;
}

static void __exit huy_led(void)
{
	printk(KERN_ALERT"\n ------------ DungLX: huy led driver ---------------");
	platform_driver_unregister(&sample_driver);
	return;
}

module_init(khoi_tao_led);
module_exit(huy_led);

MODULE_AUTHOR("Dung Le Xuan <dunglx@bkav.com>");
MODULE_DESCRIPTION("LEDS SYSFS Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:led-sysfs");
