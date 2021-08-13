/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#include <zephyr.h>

#include <aqw.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(aqw, CONFIG_AQW_LOG_LEVEL);

static struct aqw_sensor **aqw_sensors;
static size_t aqw_sensor_count = 0;
static aqw_sensor_data_ready_t cb;
static struct k_work_delayable aqw_sensor_work;

static void aqw_sensor_work_fn(struct k_work *work)
{

    int err = 0;
    struct aqw_sensor_data data[aqw_sensor_count];

    /* Check each sensor */
    for (int i = 0; i < aqw_sensor_count; i++)
    {

        /* Set type to INVALID until proven otherwise */
        data[i].type = AQW_INVALID_SENSOR;

        /* If not null fetch */
        if (aqw_sensors[i]->dev == NULL)
            continue;

        LOG_DBG("Getting from %s chan %i", aqw_sensors[i]->dev_name, aqw_sensors[i]->chan);

        err = sensor_sample_fetch_chan(aqw_sensors[i]->dev, aqw_sensors[i]->chan);
        if (err)
        {
            LOG_ERR("Unable to fetch from %s on %i chan.", aqw_sensors[i]->dev_name, aqw_sensors[i]->chan);
            continue;
        }

        /* Then copy data over */
        err = sensor_channel_get(aqw_sensors[i]->dev, aqw_sensors[i]->chan, &data[i].val);
        if (err)
        {
            LOG_ERR("Unable to get from %s on %i chan.", aqw_sensors[i]->dev_name, aqw_sensors[i]->chan);
            continue;
        }

        LOG_DBG("Data from %s: %i.%i", aqw_sensors[i]->dev_name, data[i].val.val1, data[i].val.val2);

        /* Assign type */
        data[i].type = aqw_sensors[i]->type;

        /* Assign timestamp */
        data[i].ts = k_uptime_ticks();
    }

    /* call callback if not null */
    if (cb != NULL)
    {
        cb(data, aqw_sensor_count);
    }
}

int aqw_init(struct aqw_sensor **_sensors, size_t _sensor_count, aqw_sensor_data_ready_t _cb)
{

    /* Make sure not NULL */
    if (_sensors == NULL)
    {
        LOG_ERR("Sensors invalid");
        return -EINVAL;
    }

    /* Make sure more than 0 */
    if (_sensor_count == 0)
    {
        LOG_ERR("Sensor count invalid");
        return -EINVAL;
    }

    /* Callback is necessary */
    if (_cb == NULL)
        return -EINVAL;

    /* Copy pointer to array of pointers */
    aqw_sensors = _sensors;

    /* Set the count */
    aqw_sensor_count = _sensor_count;

    /* Set callback */
    cb = _cb;

    /* Get the device for each sensor */
    for (int i = 0; i < aqw_sensor_count; i++)
    {

        /* Then continue on our merry way */
        aqw_sensors[i]->dev = device_get_binding(aqw_sensors[i]->dev_name);

        /* Make sure the device is legit */
        if (aqw_sensors[i]->dev == NULL)
        {
            LOG_WRN("Unable to find %s.", aqw_sensors[i]->dev_name);
        }
        else
        {
            LOG_DBG("Found %s.", aqw_sensors[i]->dev_name);
        }
    }

    /* Initialize work */
    k_work_init_delayable(&aqw_sensor_work, aqw_sensor_work_fn);

    return 0;
}

int aqw_sensor_start_fetch(void)
{

    k_work_reschedule(&aqw_sensor_work, K_NO_WAIT);

    return 0;
}

char *aqw_sensor_type_to_string(enum aqw_sensor_type type)
{
    switch (type)
    {
    case AQW_TEMPERATURE_SENSOR:
        return "Temperature";
    case AQW_HUMIDITY_SENSOR:
        return "Humidity";
    case AQW_PM25_SENSOR:
        return "PM2.5";
    default:
        return "Unknown";
    }
}