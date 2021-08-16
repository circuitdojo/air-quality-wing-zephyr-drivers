/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#define DT_DRV_COMPAT sensirion_shtc3

#include <math.h>
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
    uint8_t raw_temp[3];
    uint8_t raw_humidity[3];
    bool use_raw_temp;
    bool use_raw_humidity;
};

static int shtc3_sleep(const struct device *i2c_dev)
{

    int err = 0;

    /* Power up command */
    uint8_t sleep_cmd[] = SHTC3_SLEEP;
    err = i2c_write(i2c_dev, sleep_cmd, sizeof(sleep_cmd), DT_INST_REG_ADDR(0));
    if (err)
    {
        LOG_WRN("Unable to sleep SHTC3. Err: %i", err);
    }

    return err;
}

static int shtc3_wake(const struct device *i2c_dev)
{

    int err = 0;

    /* Power up command */
    uint8_t wake_cmd[] = SHTC3_WAKE;
    err = i2c_write(i2c_dev, wake_cmd, sizeof(wake_cmd), DT_INST_REG_ADDR(0));
    if (err)
    {
        LOG_WRN("Unable to wake SHTC3. Err: %i", err);
    }

    return err;
}

static int shtc3_sample_fetch_humidity(struct shtc3_data *dat)
{
    int err = 0;

    uint8_t cmd[] = SHTC3_HUMIDITY_HOLD_CMD;

    /* Get the temperature */
    err = i2c_write_read(dat->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), dat->raw_humidity, sizeof(dat->raw_humidity));
    if (err)
    {
        LOG_WRN("Unable to read humidity. Err: %i", err);
        return err;
    }

    /* Calculate the humidity */
    uint16_t humidity = (dat->raw_humidity[0] << 8) + dat->raw_humidity[1];
    double humidity_float = humidity * 100 / pow(2, 16);

    /* Convert! */
    sensor_value_from_double(&dat->humidity, humidity_float);

    return 0;
}

static int shtc3_sample_fetch_temp(struct shtc3_data *dat)
{

    int err = 0;

    uint8_t cmd[] = SHTC3_TEMP_HOLD_CMD;

    /* Get the temperature */
    err = i2c_write_read(dat->i2c_dev, DT_INST_REG_ADDR(0),
                         cmd, sizeof(cmd), dat->raw_temp, sizeof(dat->raw_temp));
    if (err)
    {
        LOG_WRN("Unable to read temperature. Err: %i", err);
        return err;
    }

    /* Calculate the temperature */
    uint16_t temperature = (dat->raw_temp[0] << 8) + dat->raw_temp[1];
    double temperature_float = (temperature * 175) / pow(2, 16) - 45;

    /* Convert! */
    sensor_value_from_double(&dat->temperature, temperature_float);

    return 0;
}

static int shtc3_sample_fetch(const struct device *dev,
                              enum sensor_channel chan)
{

    struct shtc3_data *dat = (struct shtc3_data *)dev->data;

    /* Power up */
    shtc3_wake(dat->i2c_dev);
    k_sleep(K_MSEC(1));

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_ALL:
        shtc3_sample_fetch_temp(dat);
        shtc3_sample_fetch_humidity(dat);
        break;
    case SENSOR_CHAN_AMBIENT_TEMP:
        shtc3_sample_fetch_temp(dat);
        break;
    case SENSOR_CHAN_HUMIDITY:
        shtc3_sample_fetch_humidity(dat);
        break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    /* Power down */
    shtc3_sleep(dat->i2c_dev);

    return 0;
}

static int shtc3_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{

    struct shtc3_data *dat = (struct shtc3_data *)dev->data;

    /* Clear value */
    memset(val, 0, sizeof(*val));

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_AMBIENT_TEMP:

        if (dat->use_raw_temp)
        {
            memcpy(val, dat->raw_temp, sizeof(dat->raw_temp));
        }
        else
        {
            memcpy(val, &dat->temperature, sizeof(struct sensor_value));
        }

        break;
    case SENSOR_CHAN_HUMIDITY:

        if (dat->use_raw_humidity)
        {
            memcpy(val, dat->raw_humidity, sizeof(dat->raw_humidity));
        }
        else
        {
            memcpy(val, &dat->humidity, sizeof(struct sensor_value));
        }
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

    /* Ensure use of normal values by default */
    data->use_raw_humidity = false;
    data->use_raw_temp = false;

    return 0;
}

static int shtc3_attr_get(const struct device *dev,
                          enum sensor_channel chan,
                          enum sensor_attribute attr,
                          struct sensor_value *val)
{

    uint8_t *val_bytes = (uint8_t *)&val;
    struct shtc3_data *data = dev->data;

    switch (chan)
    {
    case SENSOR_CHAN_AMBIENT_TEMP:
        val_bytes[0] = data->use_raw_temp ? 1 : 0;
        break;
    case SENSOR_CHAN_HUMIDITY:
        val_bytes[0] = data->use_raw_humidity ? 1 : 0;
        break;
    default:
        LOG_WRN("Unknown channel %i", chan);
        break;
    }

    return 0;
}

static int shtc3_attr_set(const struct device *dev,
                          enum sensor_channel chan,
                          enum sensor_attribute attr,
                          const struct sensor_value *val)

{

    enum shtc3_attribute shtc3_attr = (enum shtc3_attribute)attr;
    struct shtc3_data *data = dev->data;

    switch (shtc3_attr)
    {

    case SHTC3_ATTR_USE_RAW:
    {

        switch (chan)
        {
        case SENSOR_CHAN_AMBIENT_TEMP:
            if (val->val1 > 0)
            {
                data->use_raw_temp = true;
            }
            else if (val->val1 == 0)
            {
                data->use_raw_temp = false;
            }
            break;
        case SENSOR_CHAN_HUMIDITY:
            if (val->val1 > 0)
            {
                data->use_raw_humidity = true;
            }
            else if (val->val1 == 0)
            {
                data->use_raw_humidity = false;
            }

            break;
        default:
            LOG_WRN("Unknown channel %i", chan);
            break;
        }
    }
    break;
    default:
        LOG_WRN("Unknown attr %i", shtc3_attr);
        break;
    }

    return 0;
}

static const struct sensor_driver_api shtc3_api = {
    .attr_set = &shtc3_attr_set,
    .attr_get = &shtc3_attr_get,
    .sample_fetch = &shtc3_sample_fetch,
    .channel_get = &shtc3_channel_get,
};

static struct shtc3_data shtc3_data;

DEVICE_DEFINE(shtc3, DT_INST_LABEL(0),
              shtc3_init, NULL,
              &shtc3_data, NULL, POST_KERNEL,
              CONFIG_SENSOR_INIT_PRIORITY, &shtc3_api);
