/*
 *  bbk_boards_version.c
 *
 * Copyright (C) 2016 Vivo, Inc.
 * Author: WangLe <wangle@vivo.com.cn>
 *
 * Description: cali the hardware board version and show it
 *
 */
#include <linux/module.h>
#include <linux/kernel.h> 
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/bbk_drivers_info.h>

#define TAG "BOARD_VERSION"

struct boards_version_data {
	unsigned gpio_nums;
	unsigned gpios[12];
	char board_version[12];
	char model_value[128];
};

struct boards_version_data *bv_data;

static int boards_version_gpio_state(int gpio_num)
{
	return gpio_get_value(gpio_num);
}

static ssize_t board_version_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, bv_data->board_version);
}

char *get_board_version(void)
{
	return bv_data->board_version;
}

EXPORT_SYMBOL_GPL(get_board_version);


static ssize_t model_value_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, bv_data->model_value);
}

static struct debug_sysfs_entry board_version = 
		__ATTR(board_version, S_IRUGO, board_version_show, NULL);
static struct debug_sysfs_entry model_value = 
		__ATTR(model_value, S_IRUGO, model_value_show, NULL);


static int boards_version_parse_dt(struct device *dev, struct boards_version_data *bvdata)
{
	struct device_node *np = dev->of_node;
	int gpio, i;
	if (of_property_read_u32(np, "gpio_nums", &bvdata->gpio_nums)) {
		printk(KERN_ERR "%s: boards version gpio nums(%d) is not valid \n", TAG, bvdata->gpio_nums);
		return -1;
	}
	
	if (of_property_read_u32_array(np, "gpios", bvdata->gpios, bvdata->gpio_nums)) {
		printk(KERN_ERR "%s: boards version gpios get fail \n", TAG);
		return -1;
	}

	gpio = of_get_named_gpio(np, "gpios_start", 0);
	if (!gpio_is_valid(gpio)) {
		printk(KERN_ERR "%s: boards version gpios get fail \n", TAG);
		return -1;
	}

	for (i = 0; i < bvdata->gpio_nums-1; i++) {
		bvdata->gpios[i+1] = bvdata->gpios[i+1] - bvdata->gpios[0] + gpio;
	}
	bvdata->gpios[0] = gpio;

	return 0;
}

static void boards_version_set(struct boards_version_data *bvdata)
{
	unsigned gpio_nums = bvdata->gpio_nums;
	unsigned count = 0;
	unsigned i = 0;

	for (i = 0; i < gpio_nums; i++) {
		bvdata->board_version[i] = '0' + boards_version_gpio_state(bvdata->gpios[i]);
		count += sprintf(&bvdata->model_value[count], "GPIO-%d-%c,", bvdata->gpios[i], bvdata->board_version[i]);
	}
	bvdata->board_version[gpio_nums] = '\0';
	bvdata->model_value[count - 1] = '\0';
}

static int boards_version_probe(struct platform_device *pdev)
{

	struct boards_version_data *boards_version_data;
	int ret = 0;

	boards_version_data = kzalloc(sizeof(struct boards_version_data), GFP_KERNEL);
	if (!boards_version_data)
		return -ENOMEM;
		
	bv_data = boards_version_data;

	ret = boards_version_parse_dt(&pdev->dev, boards_version_data);
	if (ret < 0) {
		printk(KERN_ERR "%s: boards version parse dt fail \n", TAG);
		goto free_pdata;
	}

	boards_version_set(boards_version_data);
	
	ret = devs_create_sys_files(&board_version.attr);
	if (ret < 0) {
		printk(KERN_ERR "%s: board version sys files create error \n", TAG);
		goto free_pdata;
	}
	
	ret = devs_create_sys_files(&model_value.attr);
	if (ret < 0) {
		printk(KERN_ERR "%s: model value sys files create error \n", TAG);
		goto free_pdata;
	}

	return 0;

free_pdata:
	kfree(boards_version_data);
	return ret;
}

static int boards_version_remove(struct platform_device *pdev)
{
	struct boards_version_data *boards_version_data = platform_get_drvdata(pdev);

	kfree(boards_version_data);

	return 0;
}

#ifdef CONFIG_OF              
static struct of_device_id board_match_table[] = {
    { .compatible = "board-version",},
    {},
};
#endif 

static struct platform_driver boards_version_driver = {
    .probe      = boards_version_probe,
    .remove     = boards_version_remove,
    .driver     = {
		.name   = "board-version",       
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = board_match_table,
#endif
	},
};

static int __init boards_version_init(void)
{
	return platform_driver_register(&boards_version_driver);
}

static void __exit boards_version_exit(void)
{
	platform_driver_unregister(&boards_version_driver);
}

arch_initcall(boards_version_init);
module_exit(boards_version_exit);

MODULE_AUTHOR("WangLe <wangle@vivo.com.cn>");
MODULE_DESCRIPTION("Hardware Boards Version");
MODULE_LICENSE("GPL");

