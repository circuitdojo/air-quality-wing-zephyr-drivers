/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#define DT_DRV_COMPAT sensirion_sgp40cd

#include <math.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>

#include "sgp40.h"
#include <sensirion_common.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sgp40, CONFIG_SENSOR_LOG_LEVEL);

struct sgp40_data
{
    const struct device *i2c_dev;
    struct sensor_value voc;
    bool has_temperature;
    bool has_humidity;
    uint8_t temperature[3];
    uint8_t humidity[3];
    /* Enable pin */
    const struct device *gpio;
    const char *power_en_dev_name;
    gpio_pin_t power_en_pin;
    gpio_dt_flags_t power_en_flags;
};

static int sgp40_sample_fetch(const struct device *dev,
                              enum sensor_channel chan)
{

    int err = 0;
    struct sgp40_data *dat = (struct sgp40_data *)dev->data;

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_VOC:
    {

        /* Prepare command */
        uint8_t cmd[] = SGP40_MEAS_RAW_NO_HUM_OR_TEMP_CMD;
        uint8_t rx_buf[] = {0, 0, 0};

        /* If calibration bytes are good, use them */
        if (dat->has_humidity && dat->has_temperature)
        {
            memcpy(&cmd[2], dat->humidity, sizeof(dat->humidity));
            memcpy(&cmd[5], dat->temperature, sizeof(dat->temperature));
        }

        /* Multiple readings since the first doesn't work.. */
        for (int i = 0; i < 1; i++)
        {

            /* Get the VOC level */
            err = i2c_write(dat->i2c_dev, cmd, sizeof(cmd), DT_INST_REG_ADDR(0));
            if (err)
            {
                LOG_WRN("Unable to write VOC request. Err: %i", err);
                return err;
            }

            /* Wait for the data */
            k_sleep(K_MSEC(30));

            err = i2c_read(dat->i2c_dev, rx_buf, sizeof(rx_buf), DT_INST_REG_ADDR(0));
            if (err)
            {
                LOG_WRN("Unable to read VOC data. Err: %i", err);
                return err;
            }
        }

        /* Check CRC */
        uint8_t crc = sensirion_calc_crc(rx_buf);
        if (rx_buf[2] != crc)
        {
            LOG_WRN("CRC does not match! CRC: %x", crc);
            return -EPROTO;
        }

        /* Copy data over */
        /* Note need to swap the bytes since the endianess is different */
        dat->voc.val1 = (rx_buf[0] << 8) + (rx_buf[1]);
        dat->voc.val2 = 0;

        /* Power heater off */
        // uint8_t heater_off_cmd[] = SGP40_HEATER_OFF_CMD;
        // err = i2c_write(dat->i2c_dev, heater_off_cmd, sizeof(heater_off_cmd), DT_INST_REG_ADDR(0));
        // if (err)
        // {
        //     LOG_WRN("Unable to power off SGP40 heater. Err: %i", err);
        //     return err;
        // }
    }

    break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    return 0;
}

static int sgp40_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{

    struct sgp40_data *dat = (struct sgp40_data *)dev->data;

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_VOC:
        memcpy(val, &dat->voc, sizeof(struct sensor_value));
        break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    return 0;
}

static int sgp40_init(const struct device *dev)
{

    struct sgp40_data *data = dev->data;

    data->i2c_dev = device_get_binding(DT_INST_BUS_LABEL(0));

    if (data->i2c_dev == NULL)
    {
        LOG_ERR("Unable to get I2C Master.");
        return -EINVAL;
    }

    /* By default false */
    data->has_humidity = false;
    data->has_temperature = false;

/* Configure GPIO */
#if DT_INST_PROP_HAS_IDX(0, enable_gpios, 0)
    data->power_en_dev_name = DT_INST_GPIO_LABEL(0, enable_gpios);
    data->power_en_pin = DT_INST_GPIO_PIN(0, enable_gpios);
    data->power_en_flags = DT_INST_GPIO_FLAGS(0, enable_gpios);
    data->gpio = device_get_binding(data->power_en_dev_name);
    if (data->gpio == NULL)
        LOG_WRN("Power enable pin is not defined!");

    /* Default on */
    gpio_pin_configure(data->gpio, data->power_en_pin, GPIO_OUTPUT_HIGH);
    k_sleep(K_USEC(800));
#endif

    uint8_t soft_rst_cmd[] = SGP40_SOFT_RST_CMD;
    i2c_write(data->i2c_dev, soft_rst_cmd, sizeof(soft_rst_cmd), DT_INST_REG_ADDR(0));

    /* Start hot plate */
    uint8_t start_cmd[] = SGP40_MEAS_RAW_NO_HUM_OR_TEMP_CMD;
    i2c_write(data->i2c_dev, start_cmd, sizeof(start_cmd), DT_INST_REG_ADDR(0));

    return 0;
}

static int sgp40_attr_set(const struct device *dev,
                          enum sensor_channel chan,
                          enum sensor_attribute attr,
                          const struct sensor_value *val)
{

    enum sgp40_attribute sgp40_attr = (enum sgp40_attribute)attr;
    struct sgp40_data *data = dev->data;

    /* Check if we're setting the temp/humidity info */
    if (chan == SENSOR_CHAN_VOC && sgp40_attr == SGP40_ATTR_RAW_HUM_TEMP)
    {
        /* Invalid if 0 */
        if (val->val1 == 0 || val->val2 == 0)
            return -EINVAL;

        uint8_t *temp_bytes = (uint8_t *)&val->val1;
        uint8_t *hum_bytes = (uint8_t *)&val->val2;

        /* Copy the data over*/
        memcpy(data->temperature, temp_bytes, sizeof(data->temperature));
        memcpy(data->humidity, hum_bytes, sizeof(data->humidity));

        /* We can use these in temperature calculations */
        data->has_humidity = true;
        data->has_temperature = true;
    }

    return 0;
}

static const struct sensor_driver_api sgp40_api = {
    .attr_set = &sgp40_attr_set,
    .sample_fetch = &sgp40_sample_fetch,
    .channel_get = &sgp40_channel_get,
};

/* Main instantiation matcro */
#define SGP40_DEFINE(inst)                                       \
    static struct sgp40_data sgp40_data_##inst;                  \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          sgp40_init, NULL,                      \
                          &sgp40_data_##inst, NULL, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &sgp40_api);

/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(SGP40_DEFINE)
