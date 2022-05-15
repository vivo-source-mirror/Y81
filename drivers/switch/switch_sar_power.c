/*
 *  drivers/switch/switch_sar_power.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/wakelock.h>
#define TAG "SAR_POWER"

struct sar_power_switch_data {
	struct switch_dev sdev;
	unsigned rf_detect_gpio;
	unsigned rf_detect_gpio_1;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	int irq_1;
	int enable;
	struct work_struct work;
	struct work_struct switch_work;
};

static unsigned long switch_enable = -1;
static bool sar_gpio_flag = true;

static void sar_power_irq_work(struct work_struct *work)
{
	int state;
	/* added for irq_1 */
	int state_1 = 0;
	struct sar_power_switch_data	*data =
		container_of(work, struct sar_power_switch_data, work);
	pr_info("[%s]:[%s] irq work\n", TAG, __func__);

	state = gpio_get_value(data->rf_detect_gpio);

	if (sar_gpio_flag) {
		state_1 = gpio_get_value(data->rf_detect_gpio_1);
		pr_info("[%s]:[%s] state is %d, state_1 is %d\n", TAG, __func__, state, state_1);
		state = state || state_1;
	}

	pr_info("[%s]:[%s] state is %d\n", TAG, __func__, state);
	switch_set_state(&data->sdev, state);
}

static irqreturn_t sar_power_irq_handler(int irq, void *dev_id)
{
	struct sar_power_switch_data *switch_data =
	    (struct sar_power_switch_data *)dev_id;

	pr_info("[%s]:[%s] irq had triggered\n", TAG, __func__);
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static void sar_power_switch_work(struct work_struct *switch_work)
{
	unsigned long switch_state = switch_enable;
	int ret = -1;
	struct sar_power_switch_data	*switch_data =
		container_of(switch_work, struct sar_power_switch_data, switch_work);
	pr_info("[%s]:[%s] sar power switch work\n", TAG, __func__);

	if (switch_state == 1) {
		if (switch_data->enable == 1) {
			return;
		}

		ret = request_irq (switch_data->irq, sar_power_irq_handler,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, switch_data->sdev.name, switch_data);
			/*IRQF_TRIGGER_FALLING|IRQF_ONESHOT, switch_data->sdev.name, switch_data);*/
		if (ret < 0) {
			pr_info("[%s]:[%s] request irq %d err %d\n", TAG, __func__, switch_data->irq, ret);
			return;
		}

		if (sar_gpio_flag) {
			ret = request_irq (switch_data->irq_1, sar_power_irq_handler,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, switch_data->sdev.name, switch_data);
			if (ret < 0) {
				pr_err("[%s]:[%s] request irq_1 %d err %d\n", TAG, __func__, switch_data->irq_1, ret);
				return;
			}
		}
		schedule_work(&switch_data->work);

		switch_data->enable = 1;
		pr_info("[%s]:[%s] switch enable is %d\n", TAG, __func__, switch_data->enable);
	} else {
		if (switch_data->enable == 0 || switch_data->enable == -1) {
			/* ensure disable state is 1 */
			switch_set_state(&switch_data->sdev, 1);
			return;
		}

		disable_irq(switch_data->irq);
		free_irq(switch_data->irq, switch_data);

		if (sar_gpio_flag) {
			disable_irq(switch_data->irq_1);
			free_irq(switch_data->irq_1, switch_data);
		}

		switch_set_state(&switch_data->sdev, 1);
		switch_data->enable = 0;
		pr_info("[%s]:[%s] switch enable is %d\n", TAG, __func__, switch_data->enable);
	}
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct sar_power_switch_data	*switch_data =
		container_of(sdev, struct sar_power_switch_data, sdev);
	const char *state;
	pr_info("[%s]:[%s] switch gpio print state\n", TAG, __func__);
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t length)
{
	unsigned long val;
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);
	struct sar_power_switch_data *switch_data =
		container_of(sdev, struct sar_power_switch_data, sdev);
	pr_info("[%s]:[%s] enable store\n", TAG, __func__);

    if (!attr || !dev || !buf)
		return -EINVAL;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch_enable = val;
	schedule_work(&switch_data->switch_work);

	return length;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);
	struct sar_power_switch_data *switch_data =
		container_of(sdev, struct sar_power_switch_data, sdev);
	pr_info("[%s]:[%s] enable show\n", TAG, __func__);

	return sprintf(buf, "enable=%d\n", switch_data->enable);
}

static DEVICE_ATTR(enable, S_IRWXU, enable_show, enable_store);

static int sar_power_switch_probe(struct platform_device *pdev)
{

	struct sar_power_switch_data *switch_data;
	int ret = 0;

	pr_err("[%s]:[%s] probe begin\n", TAG, __func__);
	switch_data = kzalloc(sizeof(struct sar_power_switch_data), GFP_KERNEL);
	if (!switch_data) {
		pr_err("[%s]:[%s] probe error, switch data is empty\n", TAG, __func__);
		return -ENOMEM;
	}

	pr_err("[%s]:[%s] probe 222\n", TAG, __func__);

	switch_data->sdev.name = "sar-power";
	switch_data->enable = -1;
	/*switch_data->rf_detect_gpio = of_get_named_gpio_flags(
				pdev->dev.of_node, "sar-power-rf-detect,gpios",0,&flags);*/
	switch_data->rf_detect_gpio = of_get_named_gpio(
				pdev->dev.of_node, "sar-power-rf-detect,gpios", 0);
	/*if(of_property_read_u32(pdev->dev.of_node, "sar-power-rf-detect,gpios",
		&(switch_data->rf_detect_gpio)))*/
	if (!gpio_is_valid(switch_data->rf_detect_gpio)) {
		pr_err("[%s]:[%s]: sar-power-rf-detect gpio get fail\n", TAG, __func__);
		return -1;
	}

	switch_data->rf_detect_gpio_1 = of_get_named_gpio(
				pdev->dev.of_node, "sar-power-rf-detect_1,gpios", 0);
	/*ret = of_property_read_u32(pdev->dev.of_node, "sar-power-rf-detect_1,gpios",
		&switch_data->rf_detect_gpio_1);*/
	if (!gpio_is_valid(switch_data->rf_detect_gpio_1)) {
		pr_err("[%s]:[%s] sar-power-fr-detect second gpio get fail, only use one\n", TAG, __func__);
		sar_gpio_flag = false;
	}


	pr_err("[%s]:[%s] sar-power-rf-detect gpio is %d, second gpio is %d\n", TAG, __func__, switch_data->rf_detect_gpio, switch_data->rf_detect_gpio_1);
	switch_data->sdev.print_state = switch_gpio_print_state;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	pr_err("[%s]:[%s] probe 333\n", TAG, __func__);
	ret = device_create_file(switch_data->sdev.dev, &dev_attr_enable);
	if (ret < 0)
		goto err_switch_dev_register;

	pr_err("[%s]:[%s] probe 444 %s %d\n", TAG, __func__, switch_data->sdev.name, gpio_is_valid(switch_data->rf_detect_gpio));
	ret = gpio_request(switch_data->rf_detect_gpio, switch_data->sdev.name);
	if (ret < 0) {
		pr_err("[%s]:[%s] probe gpio request err, ret = %d\n", TAG, __func__, ret);
		goto err_request_gpio;
	}

	pr_err("[%s]:[%s] probe 555\n", TAG, __func__);

	ret = gpio_direction_input(switch_data->rf_detect_gpio);
	if (ret < 0) {
		pr_err("[%s]:[%s] probe gpio direction input err, ret = %d\n", TAG, __func__, ret);
		goto err_set_gpio_input;
	}

	pr_info("[%s]:[%s] probe 666 \n", TAG, __func__);
	/*INIT_WORK(&switch_data->work, sar_power_irq_work);*/
	/*INIT_WORK(&switch_data->switch_work, sar_power_switch_work);*/

	switch_data->irq = gpio_to_irq(switch_data->rf_detect_gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		pr_err("[%s]:[%s] probe gpio to irq err, ret = %d\n", TAG, __func__, ret);
		goto err_detect_irq_num_failed;
	}

	if (sar_gpio_flag) {
		ret = gpio_request(switch_data->rf_detect_gpio_1, switch_data->sdev.name);
		if (ret < 0) {
			pr_err("[%s]:[%s] gpio_2: %d request error\n", TAG, __func__, switch_data->rf_detect_gpio_1);
			goto err_request_gpio;
		}

		ret = gpio_direction_input(switch_data->rf_detect_gpio_1);
		if (ret < 0) {
			pr_err("[%s]:[%s] gpio_2: %d set input error\n", TAG, __func__, switch_data->rf_detect_gpio_1);
			goto err_set_gpio_input;
		}
		switch_data->irq_1 = gpio_to_irq(switch_data->rf_detect_gpio_1);

		if (switch_data->irq_1 < 0) {
			ret = switch_data->irq_1;
			pr_err("[%s]:[%s] gpio_2 %d set irq error\n", TAG, __func__, switch_data->rf_detect_gpio_1);
			goto err_detect_irq_num_failed;
		}
	}

	INIT_WORK(&switch_data->work, sar_power_irq_work);
	INIT_WORK(&switch_data->switch_work, sar_power_switch_work);
	sar_power_irq_work(&switch_data->work);

	pr_info("[%s]:[%s] probe end.\n", TAG, __func__);

	return 0;

err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->rf_detect_gpio);
	pr_err("[%s]:[%s] probe err, gpio free\n", TAG, __func__);
err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
	pr_err("[%s]:[%s] probe err, switch dev unregister.\n", TAG, __func__);
err_switch_dev_register:
	pr_err("[%s]:[%s] probe err, kfree\n", TAG, __func__);
	kfree(switch_data);

	pr_info("[%s]:[%s] probe err return ret = %d\n", TAG, __func__, ret);
	return ret;
}

static int sar_power_switch_remove(struct platform_device *pdev)
{
	struct sar_power_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->rf_detect_gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);
	pr_info("[%s]:[%s] remove.\n", TAG, __func__);

	return 0;
}

/*#ifdef CONFIG_OF*/
static struct of_device_id sar_power_match_table[] = {
	{ .compatible = "sar-power",},
	{},
};
/*#endif*/

static struct platform_driver sar_power_switch_driver = {
	.probe		= sar_power_switch_probe,
	.remove		= sar_power_switch_remove,
	.driver		= {
		.name	= "sar-power",
		.owner	= THIS_MODULE,
/*#ifdef CONFIG_OF*/
		.of_match_table = sar_power_match_table,
/*#endif*/
	},
};

static int __init sar_power_switch_init(void)
{
	int ret = 0;
	pr_info("[%s]:[%s] 0ops init\n", TAG, __func__);
	ret = platform_driver_register(&sar_power_switch_driver);
	pr_info("[%s]:[%s] 0ops init end ret = %d\n", TAG, __func__, ret);
	return ret;
}

static void __exit sar_power_switch_exit(void)
{
	pr_info("[%s]:[%s] 0ops exit\n", TAG, __func__);
	platform_driver_unregister(&sar_power_switch_driver);
}

late_initcall(sar_power_switch_init);
/*module_init(sar_power_switch_init);*/
module_exit(sar_power_switch_exit);

MODULE_AUTHOR("Wangle wangle@vivo.com.cn");
MODULE_DESCRIPTION("SAR POWER Switch driver");
MODULE_LICENSE("GPL");
