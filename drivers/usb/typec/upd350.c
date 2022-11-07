// SDPX-identifier: GPL-2.0
/*
 * Copyright (C) Higgsone Sp. z o.o.
 *
 * Authors: Bartosz Nowicki <bartosz.nowicki@higgsone.com>
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>

struct upd350 {
    struct device *dev;
    struct i2c_client *i2c_client;

    struct typec_port *port;
    struct typec_capability capability;
    struct typec_partner *partner;

    struct regulator *vdd_supply;

    enum typec_port_type port_type_power;
    enum typec_port_data port_type_data;
    enum typec_pwr_opmode pwr_opmode;
    bool source_enabled;
    bool sink_enabled;

    struct usb_role_switch *role_sw;
};

#define UPD350_GPIO_OUTPUT_ACTIVE   BIT(3)

struct upd350_reg_info {
    uint16_t address;
    u8 length;
    bool is_writeable;
};

static struct upd350_reg_info UPD350_USB_PID_REGISTER = {
    .address = 0x0004,
    .length = 2,
    .is_writeable = false,
};

static struct upd350_reg_info UPD350_CFG_PIO0_REGISTER = {
    .address = 0x0030,
    .length = 1,
    .is_writeable = true,
};

static struct upd350_reg_info UPD350_CFG_PIO1_REGISTER = {
    .address = 0x0031,
    .length = 1,
    .is_writeable = true,
};

static struct upd350 *chip;

static int set_sink(struct upd350 *chip, bool value);
static int set_source(struct upd350 *chip, bool value);

/*
 * Callbacks from typec module for changing power and data role
 */

static int upd350_power_role_set(struct typec_port *port, enum typec_role role) {
    struct upd350 *chip = typec_get_drvdata(port);
    if (role == TYPEC_SINK) {
        set_sink(chip, true);
    } else {
        set_source(chip, true);
    }
    return 0;
}

static const struct typec_operations upd350_typec_ops = {
    .pr_set = upd350_power_role_set,
};

/*
 * I2C helper functions
 */

static int i2c_read_register(struct upd350 *chip, struct upd350_reg_info *reg, u8 *read_buffer) {
    int ret;
    __be16 reg_address = cpu_to_be16(reg->address);

    ret = i2c_master_send(chip->i2c_client, (u8 *)&reg_address, 2);
    if (!ret)  {
        dev_err(chip->dev, "cannot send i2c message: %d\n", ret);
        return ret;
    }
    ret = i2c_master_recv(chip->i2c_client, read_buffer, reg->length);
    if (!ret) {
        dev_err(chip->dev, "cannot read from i2c device: %d\n", ret);
        return ret;
    }

    return ret;
}

static int i2c_write_register(struct upd350 *chip, struct upd350_reg_info *reg, const u8 *write_buffer) {
    int ret;
    __be16 reg_address = cpu_to_be16(reg->address);
    u8 write_buffer_[256];
    u8 write_buffer_length = 2;

    if (!reg->is_writeable) {
        return -EINVAL;
    }

    memcpy((void *)write_buffer_, &reg_address, sizeof(__be16));
    for (; write_buffer_length < (reg->length + 2); write_buffer_length++) {
        write_buffer_[write_buffer_length] = write_buffer[write_buffer_length - 2];
    }
    ret = i2c_master_send(chip->i2c_client, write_buffer_, write_buffer_length);
    if (!ret) {
        dev_err(chip->dev, "cannot write to i2c device: %d\n", ret);
    }

    return ret;
}

static int i2c_write_register_u8(struct upd350 *chip, struct upd350_reg_info *reg, u8 value) {
    return i2c_write_register(chip, reg, &value);
}

static int i2c_write_register_u16(struct upd350 *chip, struct upd350_reg_info *reg, u16 value) {
    return i2c_write_register(chip, reg, (const u8 *)&value);
}

static int i2c_write_register_u32(struct upd350 *chip, struct upd350_reg_info *reg, u32 value) {
    return i2c_write_register(chip, reg, (const u8 *)&value);
}

/*
 * GPIO handling helper functions
 */
int set_sink(struct upd350 *chip, bool value) {
    int ret;
    u8 reg_val = value ? 0x0F : 0x07;
    chip->sink_enabled = false;

    if (chip->source_enabled) {
        if (set_source(chip, false) < 0) {
            chip->sink_enabled = false;
            return -EIO;
        }
    }

    ret = i2c_write_register_u8(chip, &UPD350_CFG_PIO0_REGISTER, reg_val);
    if (ret >= 0) {
        dev_info(chip->dev, "set to sink mode: %d\n", value);
        chip->sink_enabled = value;
        return 0;
    } else {
        dev_err(chip->dev, "failed to set sink mode: %d\n", ret);
        return ret;
    }
}

int set_source(struct upd350 *chip, bool value) {
    int ret;
    int regulator_ret;
    u8 reg_val = value ? 0x0F : 0x07;
    chip->source_enabled = false;

    if (!chip->vdd_supply) {
        dev_err(chip->dev, "failed to set source mode, no vdd supply is set\n");
        return -ENODEV;
    }

    if (chip->sink_enabled) {
        if (set_sink(chip, false) < 0) {
            return -EIO;
        }
    }

    ret = i2c_write_register_u8(chip, &UPD350_CFG_PIO1_REGISTER, reg_val);

    if (ret) {
        if (value) {
            regulator_ret = regulator_enable(chip->vdd_supply);
            if (regulator_ret) {
                dev_err(chip->dev, "failed to enable vdd supply: %d\n", ret);
                return -EAGAIN;
            }
        } else {
            regulator_ret = regulator_disable(chip->vdd_supply);
            if (regulator_ret) {
                dev_err(chip->dev, "failed to disable vdd supply: %d\n", ret);
                return -EAGAIN;
            } 
        }
        chip->source_enabled = value;
    }

    dev_info(chip->dev, "set to source mode: %d\n", value);
    return ret;
}

/*
 * Interrupt handler from chip IRQ line
 */

static irqreturn_t upd350_irq_handler(int irq, void *data) {
    pr_info("UPD350 interrupt\n");
    return IRQ_HANDLED;
}

/*
 * Device ID, module entry (probe) and module remove
 */

static const struct i2c_device_id upd350_id[] = {
    { "upd350", 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, upd350_id);

static const struct of_device_id upd350_of_match[] = {
    { .compatible = "microchip,upd350" },
    {},
};
MODULE_DEVICE_TABLE(of, upd350_of_match);

static int upd350_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    const struct of_device_id *match;
    struct fwnode_handle *fwnode;
    const char *cap_str;
    int ret;
    u8 op_mode;

    chip = devm_kzalloc(&client->dev, sizeof(struct upd350), GFP_KERNEL);
    if (!chip) {
        return -ENOMEM;
    }

    i2c_set_clientdata(client, chip);
    match = i2c_of_match_device(upd350_of_match, client);

    chip->dev = &client->dev;
    chip->i2c_client = client;

    if (client->irq) {
        ret = devm_request_threaded_irq(chip->dev, client->irq, NULL, upd350_irq_handler, IRQF_ONESHOT, dev_name(chip->dev), chip);
        if (ret) {
            return -ENOENT;
        }
    }

    chip->vdd_supply = devm_regulator_get_optional(chip->dev, "vdd");
	if (IS_ERR(chip->vdd_supply)) {
		ret = PTR_ERR(chip->vdd_supply);
		if (ret != -ENODEV) {
			return ret;
        }
		chip->vdd_supply = NULL;
	}

    fwnode = device_get_named_child_node(chip->dev, "connector");
    if (!fwnode) {
        return -ENODEV;
    }

    fw_devlink_purge_absent_suppliers(fwnode);

    /*
     * Supported port power type is configured through device tree.
     */
    ret = fwnode_property_read_string(fwnode, "power-role", &cap_str);
    if (!ret) {
        ret = typec_find_port_power_role(cap_str);
        if (ret < 0) {
            return ret;
        }
        chip->port_type_power = ret;
    }
    chip->capability.type = chip->port_type_power;

    /*
     * Supported port data type is configured through device tree.
     */
    ret = fwnode_property_read_string(fwnode, "data-role", &cap_str);
    if (!ret) {
        ret = typec_find_port_data_role(cap_str);
        if (ret < 0) {
            return ret;
        }
        chip->port_type_data = ret;
    }
    chip->capability.data = chip->port_type_data;

    if (chip->port_type_power == TYPEC_PORT_SNK) {
        return 0;
    }

    if (chip->port_type_power == TYPEC_PORT_DRP) {
        ret = fwnode_property_read_string(fwnode, "try-power-role", &cap_str);
        if (!ret) {
            ret = typec_find_port_power_role(cap_str);
            if (ret >= 0) {
                chip->capability.prefer_role = ret;
            }
        } else {
            chip->capability.prefer_role = TYPEC_SINK;
        }
    }

    /*
     * Supported power operation mode is configured through device tree.
     */
    ret = fwnode_property_read_string(fwnode, "typec-power-opmode", &cap_str);
    if (!ret) {
        ret = typec_find_pwr_opmode(cap_str);
        if (ret < 0) {
            dev_err(chip->dev, "bad power operation mode: %d\n", ret);
            return -EINVAL;
        }
        chip->pwr_opmode = ret;
    }

    chip->capability.revision = USB_TYPEC_REV_1_2;
    chip->capability.accessory[0] = TYPEC_ACCESSORY_DEBUG;
    chip->capability.driver_data = chip;
    chip->capability.ops = &upd350_typec_ops;

    chip->port = typec_register_port(chip->dev, &chip->capability);
    if (IS_ERR(chip->port)) {
        ret = PTR_ERR(chip->port);
    }

    typec_set_pwr_opmode(chip->port, TYPEC_PWR_MODE_PD); // set as PD to allow changing roles between source and sink

    fwnode_handle_put(fwnode);

    i2c_read_register(chip, &UPD350_CFG_PIO0_REGISTER, &op_mode);
    chip->sink_enabled = FIELD_GET(UPD350_GPIO_OUTPUT_ACTIVE, op_mode);
    i2c_read_register(chip, &UPD350_CFG_PIO1_REGISTER, &op_mode);
    chip->source_enabled = FIELD_GET(UPD350_GPIO_OUTPUT_ACTIVE, op_mode);

    if (chip->capability.prefer_role == TYPEC_SINK) {
        set_sink(chip, true);
    } else {
        set_source(chip, true);
    }

    return 0;
}

static int upd350_remove(struct i2c_client *client) {
    struct upd350 *chip = i2c_get_clientdata(client);

	if (chip->partner) {
		typec_unregister_partner(chip->partner);
		chip->partner = NULL;
	}

	if (chip->source_enabled) {
		regulator_disable(chip->vdd_supply);
    }

	typec_unregister_port(chip->port);

	return 0;
}

static struct i2c_driver upd350_driver = {
    .driver = {
        .name = "upd350",
        .of_match_table = of_match_ptr(upd350_of_match),
    },
    .probe = upd350_probe,
    .remove = upd350_remove,
};
module_i2c_driver(upd350_driver);

MODULE_DESCRIPTION("Microchip UPD350 driver for detecting USB cable insertion");
MODULE_AUTHOR("Bartosz Nowicki <bartosz.nowicki@higgsone.com>");
MODULE_LICENSE("GPL v2");