/*
 * Adafruit power switch driver for Raspberry Pi
 *
 * Written by Sean Cross for Adafruit Industries
 *
 * Copyright (C) 2014 Adafruit Industries (www.adafruit.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define RPI_POWER_SWITCH_VERSION "1.7"
#define POWER_SWITCH_CLASS_NAME "rpi-power-switch"

#include <linux/module.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/workqueue.h>

enum button_mode {
	MODE_BUTTON = 0,
	MODE_SWITCH = 1,
};

/* Module Parameters */
static int gpio_pin = 22;
static int mode = MODE_BUTTON;
static int led_pin = 16;

/* This is the base state.  When this changes, do a shutdown. */
static int gpio_pol;

static void (*old_pm_power_off)(void);
static struct device *switch_dev;

/*
 * Bottom half of the power switch ISR.
 * We need to break this out here, as you can't run call_usermodehelper
 * from an interrupt context.
 * This function will actually Call /sbin/shutdown when the switch gets hit.
 */
static void initiate_shutdown(struct work_struct *work)
{
	int ret;
	char *cmd = "/sbin/shutdown";
	char *argv[] = {
		cmd,
		"-h",
		"now",
		NULL,
	};
	char *envp[] = {
		"HOME=/",
		"PATH=/sbin:/bin:/usr/sbin:/usr/bin",
		NULL,
	};

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	/* We only want this IRQ to fire once, ever. */
	free_irq(gpio_to_irq(gpio_pin), NULL);

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	/* Make sure the switch hasn't just bounced */
	if (mode == MODE_SWITCH && gpio_get_value(gpio_pin) != gpio_pol)
		return;

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	ret = call_usermodehelper(cmd, argv, envp, UMH_WAIT_PROC);

	pr_devel("returned %d\n", ret);
}

static struct delayed_work initiate_shutdown_work;

/*
 * This ISR gets called when the board is "off" and the switch changes.
 * It indicates we should start back up again, which means we need to
 * do a reboot.
 */
static irqreturn_t reboot_isr(int irqno, void *param)
{
	emergency_restart();
	return IRQ_HANDLED;
}

/*
 * Pulse the GPIO low for /duty/ cycles and then /high/ for 100-duty cycles.
 * Returns the number of usecs delayed.
 */
static int gpio_pulse(int gpio, int duty)
{
	int low;
	int high;
#define RATE 1

	if (duty < 0)
		duty = 0;
	if (duty > 100)
		duty = 100;
	low = duty;
	high = 100-duty;

	gpio_set_value(gpio, 0);
	udelay(RATE*low);

	gpio_set_value(gpio, 1);
	udelay(RATE*high);

	return (RATE*low)+(RATE*high);
}

/*
 * Give an indication that it's safe to turn off the board.  Pulse the LED
 * in a kind of "breathing" pattern, so the user knows that it's
 * "powered down".
 */
static int do_breathing_forever(int gpio)
{
	int err;
	int usecs;

	err = gpio_request(gpio, "LED light");
	if (err < 0) {
		pr_err("Unable to request GPIO");
		return 0;
	}
	gpio_direction_output(gpio, gpio_get_value(gpio));

	while (1) {
		/*
		 * We want four seconds:
		 *   - One second of ramp-up
		 *   - One second of ramp-down
		 *   - Two seconds of low
		 */
		for (usecs=0; usecs < 800000; )
			usecs += gpio_pulse(gpio, ((usecs*9)/80000)+10);

		for (usecs=0; usecs < 800000; )
			usecs += gpio_pulse(gpio, 100-((usecs*9)/80000));

		for (usecs=0; usecs < 800000; )
			usecs += gpio_pulse(gpio, 10);

		for (usecs=0; usecs < 800000; )
			usecs += gpio_pulse(gpio, 10);
	}
	return 0;
}

/*
 * Our shutdown function.  Execution will stay here until the switch is
 * flipped.
 * NOTE: The default power_off function sends a message to the GPU via
 * a mailbox message to shut down most parts of the core.  Since we don't
 * have any documentation on the mailbox message formats, we will leave
 * the CPU powered up here but not executing any code in order to simulate
 * an "off" state.
 */
static void rpi_power_switch_power_off(void)
{
	int ret;

	pr_info("Waiting for the switch to be flipped back...\n");
	if (mode == MODE_SWITCH)
		gpio_pol = !gpio_pol;
	ret = request_irq(gpio_to_irq(gpio_pin), reboot_isr,
			  gpio_pol?IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING,
			  "Reboot ISR", NULL);

	/*
	 * If it's taken us so long to reboot that the switch was flipped,
	 * immediately reboot.
	 */
	if (gpio_pol == gpio_get_value(gpio_pin))
		reboot_isr(0, NULL);

	do_breathing_forever(led_pin);
}

static irqreturn_t power_isr(int irqno, void *param)
{
	schedule_delayed_work(&initiate_shutdown_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

/* Sysfs entry */

static ssize_t do_shutdown_show(struct device *d,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	ret = sprintf(buf, "Write into this file to initiate a shutdown\n");
	return ret;
}

static ssize_t do_shutdown_store(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (mode == MODE_SWITCH)
		gpio_pol = !gpio_pol;
	schedule_delayed_work(&initiate_shutdown_work, msecs_to_jiffies(10));
	return count;
}
static DEVICE_ATTR(do_shutdown, 0660, do_shutdown_show, do_shutdown_store);

static struct attribute *rpi_power_switch_sysfs_entries[] = {
	&dev_attr_do_shutdown.attr,
	NULL,
};

static struct attribute_group rpi_power_switch_attribute_group = {
	.name = NULL,
	.attrs = rpi_power_switch_sysfs_entries,
};

static struct class power_switch_class = {
	.name =	POWER_SWITCH_CLASS_NAME,
	.owner =	THIS_MODULE,
};

/* Main module entry point */

static int rpi_power_switch_probe(struct platform_device *pdev)
{
	int ret = 0;
	bool force = false;
	u32 property_gpios[3];

	/* If a pm_power_off function has already been added, leave it alone */
	force = of_property_read_bool(pdev->dev.of_node, "force");
	if (!force && (pm_power_off != NULL)) {
		dev_err(&pdev->dev,
			"%s: pm_power_off function already registered",
			__func__);
		return -EBUSY;
	}

	old_pm_power_off = pm_power_off;
	pm_power_off = rpi_power_switch_power_off;

	pr_info("Adafruit Industries' power switch driver v%s\n",
		RPI_POWER_SWITCH_VERSION);

	INIT_DELAYED_WORK(&initiate_shutdown_work, initiate_shutdown);

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	/* Register our own class for the power switch */
	ret = class_register(&power_switch_class);
	if (ret < 0) {
		pr_err("%s: Unable to register class\n",
			power_switch_class.name);
		goto registration_failure;
	}

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	/* Create devices for each PWM present */
	switch_dev = device_create(&power_switch_class, &platform_bus,
				MKDEV(0, 0), NULL, "pswitch%u", 0);
	if (IS_ERR(switch_dev)) {
		pr_err("%s: device_create failed\n", power_switch_class.name);
		ret = PTR_ERR(switch_dev);
		goto device_create_failure;
	}

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	ret = sysfs_create_group(&switch_dev->kobj,
				 &rpi_power_switch_attribute_group);
	if (ret < 0) {
		pr_err("%s: create_group failed\n", power_switch_class.name);
		goto sysfs_create_group_failure;
	}

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	if (of_property_read_u32(pdev->dev.of_node, "button_mode", &mode))
		pr_devel("FAIL : of_property_read_u32 for button_mode\n");
	else
		pr_devel("%s : button_mode = %d\n", __FUNCTION__, mode);

	if (of_property_read_u32_array(pdev->dev.of_node,
				"gpios", property_gpios, 3)) {
		pr_devel("FAIL : of_proerty_read_u32 for gpios\n");
	} else {
		gpio_pin = property_gpios[1];
		pr_devel("%s : gpio_pin = %d\n", __FUNCTION__, gpio_pin);
	}

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	/*
	 * Request to use the gpio_pin for avoiding over use from/to
	 *  another driver 
	 */
	ret = gpio_request(gpio_pin, "Power switch");
	if (ret) {
		pr_err("%s: Fail devm_request_gpio by errno %d\n",
			__FUNCTION__, ret);
		goto gpio_request_failure;
	}

	/*
	 * The targeted polarity should be the opposite of the current value.
	 * I.e. we want the pin to transition to this state in order to
	 * initiate a shutdown.
	 */
	gpio_pol = !gpio_get_value(gpio_pin);
	pr_devel("%s : gpio_pol=%d\n", __FUNCTION__, gpio_pol);

	/*
	 * Request an interrupt to fire when the pin transitions to our
	 * desired state.
	 */
	ret = request_irq(__gpio_to_irq(gpio_pin), power_isr,
			gpio_pol?IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING,
			"Power button", NULL);
	if (ret) {
		pr_err("Unable to request IRQ\n");
		goto gpio_request_failure;
	}

	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	return 0;

	/* Error handling */
gpio_request_failure:
	sysfs_remove_group(&switch_dev->kobj,&rpi_power_switch_attribute_group);
sysfs_create_group_failure:
	device_unregister(switch_dev);
device_create_failure:
	class_unregister(&power_switch_class);
registration_failure:
	pm_power_off = old_pm_power_off;

	return ret;
}

/* Main module exit point (called at unload) */

static int rpi_power_switch_remove(struct platform_device *pdev)
{
	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);
	sysfs_remove_group(&switch_dev->kobj,&rpi_power_switch_attribute_group);
	device_unregister(switch_dev);
	free_irq(__gpio_to_irq(gpio_pin), NULL);
	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);
	gpio_free(gpio_pin);
	pm_power_off = old_pm_power_off;
	class_unregister(&power_switch_class);
	pr_devel("DEBUG: Passed %s %d \n",__FUNCTION__,__LINE__);

	return 0;
}

static const struct of_device_id rpi_power_switch_match[] = {
	{ .compatible = "rpi-power-switch",},
	{},
};

MODULE_DEVICE_TABLE(of, rpi_power_switch_match);

static struct platform_driver rpi_power_switch_driver = {
	.probe = rpi_power_switch_probe,
	.remove = rpi_power_switch_remove,
	.driver = {
		.name = "rpi_power_switch",
		.of_match_table = rpi_power_switch_match,
	},
};

module_platform_driver(rpi_power_switch_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(
"Sean Cross <xobs@xoblo.gs> for Adafruit Industries <www.adafruit.com>");
MODULE_ALIAS("platform:bcm2708_power_switch");

module_param(gpio_pin, int, 0);
MODULE_PARM_DESC(gpio_pin,
"GPIO pin number of the BCM processor for shutdown switch (default 22)");

module_param(led_pin, int, 0);
MODULE_PARM_DESC(led_pin, "Pin for LED to pulse after shutdown (default 16)");

module_param(mode, int, 0);
MODULE_PARM_DESC(mode,
"Shutdown switch mode (0 for button, 1 for switch, default 0)");
