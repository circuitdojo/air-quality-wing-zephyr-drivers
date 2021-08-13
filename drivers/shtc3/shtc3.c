/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#define DT_DRV_COMPAT sensirion_shtc3

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include "shtc3.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(shtc3, CONFIG_SENSOR_LOG_LEVEL);

struct shtc3_data
{
    const struct device *i2c_dev;
    struct sensor_value temperature;
    struct sensor_value humidity;
};

static int shtc3_sample_fetch(const struct device *dev,
                              enum sensor_channel chan)
{

    int err = 0;
    struct shtc3_data *dat = (struct shtc3_data *)dev->data;

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_AMBIENT_TEMP:
    {

        uint8_t cmd[] = SHTC3_TEMP_HOLD_CMD;
        uint8_t rx_buf[] = {0, 0, 0};

        /* Get the temperature */
        err = i2c_write_read(dat->i2c_dev, DT_INST_REG_ADDR(0),
                             cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
        if (err)
        {
            LOG_WRN("Unable to read temperature. Err: %i", err);
            return err;
        }

        /* Calculate the temperature */
        uint16_t temperature = (rx_buf[0] << 8) + rx_buf[1];
        double temperature_float = (temperature * 175 >> 16) - 45;

        /* Convert! */
        sensor_value_from_double(&dat->temperature, temperature_float);
    }

    break;
    case SENSOR_CHAN_HUMIDITY:
    {

        uint8_t cmd[] = SHTC3_HUMIDITY_HOLD_CMD;
        uint8_t rx_buf[] = {0, 0, 0};

        /* Get the temperature */
        err = i2c_write_read(dat->i2c_dev, DT_INST_REG_ADDR(0),
                             cmd, sizeof(cmd), rx_buf, sizeof(rx_buf));
        if (err)
        {
            LOG_WRN("Unable to read temperature. Err: %i", err);
            return err;
        }

        /* Calculate the humidity */
        uint16_t humidity = (rx_buf[0] << 8) + rx_buf[1];
        double humidity_float = humidity * 100 >> 16;

        /* Convert! */
        sensor_value_from_double(&dat->humidity, humidity_float);
    }

    break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    return 0;
}

static int shtc3_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{

    struct shtc3_data *dat = (struct shtc3_data *)dev->data;

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_AMBIENT_TEMP:
        memcpy(val, &dat->temperature, sizeof(struct sensor_value));
        break;
    case SENSOR_CHAN_HUMIDITY:
        memcpy(val, &dat->humidity, sizeof(struct sensor_value));
        break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    return 0;
}

static int shtc3_init(const struct device *dev)
{

    struct shtc3_data *data = dev->data;

    data->i2c_dev = device_get_binding(DT_INST_BUS_LABEL(0));

    if (data->i2c_dev == NULL)
    {
        LOG_ERR("Unable to get I2C Master.");
        return -EINVAL;
    }

    return 0;
}

static const struct sensor_driver_api shtc3_api = {
    .sample_fetch = &shtc3_sample_fetch,
    .channel_get = &shtc3_channel_get,
};

static struct shtc3_data shtc3_data;

DEVICE_DEFINE(shtc3, DT_INST_LABEL(0),
              shtc3_init, NULL,
              &shtc3_data, NULL, POST_KERNEL,
              CONFIG_SENSOR_INIT_PRIORITY, &shtc3_api);
