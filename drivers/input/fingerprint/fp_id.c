#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>

#include "fp_id.h"

#define MAX_TIMES		7

struct kobject kobj;

int get_fp_id(void);
static int fp_id = -1;
static int count_ground;//fp_id pin connect to ground
static int count_suspend;//fp_id pin suspend
const char *fp_project_name;
//static struct platform_device *fp_id_pdev = NULL;

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)

struct fp_id_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj, struct attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj, struct attribute *attr,  const char *buf, size_t count);
};

static ssize_t fp_id_int_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	/*int fp_frame_id = -1;
	if(FPC_BASE < fp_id && GOODIX_BASE > fp_id){
		fp_frame_id = FPC_FRAME_ID;
	}
	else if(GOODIX_BASE < fp_id){
		fp_frame_id = GOODIX_FRAME_ID;
	}
	printk("fp_id_int get_fp_id=%d, fp_frame_id=%d, count_ground=%d, count_suspend=%d.\n", get_fp_id(), fp_frame_id, count_ground, count_suspend);
	return sprintf(buf, "%d\n", fp_frame_id);*/
	char *fp_frame_id = "default";
	if (FPC_BASE < fp_id && GOODIX_BASE > fp_id) {
		fp_frame_id = "default";
	} else if (GOODIX_BASE < fp_id) {
		if (fp_id == GOODIX_GF5126M) {
			fp_frame_id = "goodix_5126m";
		} else if (fp_id == GOODIX_GF5216C) {
			fp_frame_id = "goodix_5216c";
		} else if (fp_id == GOODIX_GF5269) {
			fp_frame_id = "goodix_5269";
		} else if (fp_id == GOODIX_GF3208) {
			fp_frame_id = "goodix_3208";
		} else if (fp_id == GOODIX_GF318M) {
			fp_frame_id = "goodix_318m";
		} else if (fp_id == GOODIX_GF5288) {
			fp_frame_id = "goodix_5288";
		} else if (fp_id == SILEAD_GSL6165) {
			fp_frame_id = "silead_6165";
		}
	}
	printk("fp_id_int get_fp_id=%d, fp_frame_id=%s, count_ground=%d, count_suspend=%d.\n", get_fp_id(), fp_frame_id, count_ground, count_suspend);
	return sprintf(buf, "%s\n", fp_frame_id);
}

static ssize_t fp_id_int_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	/* nothing to do temply */
	printk("fp_id_int cannot be writed.\n");
	return 0;
}

static ssize_t fp_id_string_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	char *fp_frame_id = "NONE";
	if (FPC_BASE < fp_id && GOODIX_BASE > fp_id) {
		fp_frame_id = "NONE";
	} else if (GOODIX_BASE < fp_id) {
		if (fp_id == GOODIX_GF5126M) {
			fp_frame_id = "GOODIX_GF5126M";
		} else if (fp_id == GOODIX_GF5216C) {
			fp_frame_id = "GOODIX_GF5216C";
		} else if (fp_id == GOODIX_GF5269) {
			fp_frame_id = "GOODIX_GF5269";
		} else if (fp_id == GOODIX_GF3208) {
			fp_frame_id = "GOODIX_GF3208";
		} else if (fp_id == GOODIX_GF318M) {
			fp_frame_id = "GOODIX_GF318M";
		} else if (fp_id == GOODIX_GF5288) {
			fp_frame_id = "GOODIX_GF5288";
		} else if (fp_id == SILEAD_GSL6165) {
			fp_frame_id = "SILEAD_GSL6165";
		}
	}
	printk("fp_id_string get_fp_id=%d, fp_frame_id=%s, count_ground=%d, count_suspend=%d.\n", get_fp_id(), fp_frame_id, count_ground, count_suspend);
	return sprintf(buf, "%s\n", fp_frame_id);
}

static ssize_t fp_id_string_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	/* nothing to do temply */
	printk("fp_id_string cannot be writed.\n");
	return 0;
}
	
static struct fp_id_sysfs_entry fp_id_int =
	__ATTR(fp_id, 0644,
			fp_id_int_object_show, fp_id_int_object_store);
static struct fp_id_sysfs_entry fp_id_string =
	__ATTR(fp_id_string, 0644,
			fp_id_string_object_show, fp_id_string_object_store);
	
static struct attribute *our_own_sys_attrs[] = {
	&fp_id_int.attr,
	&fp_id_string.attr,
	NULL,
};

int get_fp_id(void)
{
	return fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t fp_id_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store) {
		ret = kobj_attr->store(k, kobj_attr, buf, count);
	}

	return ret;
}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
	.store = fp_id_object_store,
};

static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

static int fp_id_probe(struct platform_device *pdev)
{
	int ret;
	int board_id_gpio;
	int fp_gpio = -1;
	int id_state = -1;
	int i = 0;
	ret = kobject_init_and_add(&kobj, &fp_id_object_type, NULL, "fp_id");
	if (ret) {
		printk("%s: Create fp_id error!\n", __func__);
		return -EINVAL;
	}
	ret = of_property_read_string(pdev->dev.of_node, "vivo,project-name", &fp_project_name);
	if (ret) {
		printk("%s:vivo,project-name property do not find\n", __func__);
		fp_project_name = "default";
	}
	printk("%s:vivo,project-name = %s\n", __func__, fp_project_name);
	fp_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpios", 0);
	if (fp_gpio < 0) {
		printk("%s: get fp_id gpio failed!\n", __func__);
		return -EINVAL;
	}
	printk("%s:fp gpio: %d \n", __func__, fp_gpio);

	ret = devm_gpio_request(&pdev->dev, fp_gpio, "fp_id,gpios");
	if (ret)  {
		printk("%s: request fp_id gpio failed!\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < MAX_TIMES; i++) {
		mdelay(1);
		ret = gpio_get_value(fp_gpio);
		if (ret == 0) {
			count_ground++;
		} else {
			count_suspend++;
		}
	}
	id_state = (count_ground > count_suspend) ? 0:1;
	printk("%s id_state=%d.\n", __func__, id_state);
	if (!strncmp(fp_project_name, "PD1803", 6)) {
		fp_id = GOODIX_GF3208;
	} else if (!strncmp(fp_project_name, "PD1732", 6)) {
		struct device_node *of_node = of_find_compatible_node(NULL, NULL, "board-version");
		if (!of_node) {
			pr_err("of_node for GPIO171 is Null.\n");
			return -EINVAL;
		}
		board_id_gpio = of_get_named_gpio(of_node, "gpios_start", 0);
		pr_err("Get GPIO171 pin=%d.\n", board_id_gpio);
		gpio_direction_input(board_id_gpio);
		ret = gpio_get_value(board_id_gpio); /*have fingerprint when gpio is low*/
		printk("%s:board_id_gpio return ret = %d\n", __func__, ret);
		if (!ret) {
			if (0 == id_state) {
				fp_id = GOODIX_GF3208;
			} else {
				fp_id = GOODIX_GF318M;
			}
		} else {
			fp_id = -1;
		}
	}

	return 0;
}

static int
fp_id_remove(struct platform_device *pdev)
{
	printk("fp_id  remove.\n");
	kobject_del(&kobj);
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};
#endif 

static struct platform_driver fp_id_driver = {
    .probe      = fp_id_probe,
    .remove     = fp_id_remove,
    .driver = {
		.name   = "fp_id",
		.owner  = THIS_MODULE,
		.of_match_table = fp_id_match_table,
    },
};

static int __init fp_id_init(void)
{
    //fp_id_pdev = platform_device_register_simple("fp_id", 0, NULL, 0);
    return platform_driver_register(&fp_id_driver);
}
module_init(fp_id_init);

static void __exit fp_id_exit(void)
{
    platform_driver_unregister(&fp_id_driver);
    //platform_device_unregister(fp_id_pdev);
}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
