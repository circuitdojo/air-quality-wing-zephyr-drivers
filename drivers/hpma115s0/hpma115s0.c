/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#define DT_DRV_COMPAT honeywell_hpma115s0

#include <math.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <sys/ring_buffer.h>

#include "hpma115s0.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(hpma115s0, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_SOC_FAMILY_STM32
#define RX_TIMEOUT SYS_FOREVER_US
#else
#define RX_TIMEOUT 10
#endif

#define FRAME_SIZE 32

struct hpma115s0_data
{
    /* Comms */
    const struct device *uart_dev;
    struct sensor_value pm25;
    bool ready;
    uint8_t rx_buf[FRAME_SIZE * 2];

    /* Enable pin */
    const struct device *gpio;
    const char *power_en_dev_name;
    gpio_pin_t power_en_pin;
    gpio_dt_flags_t power_en_flags;
};

static int hpma115s0_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan)
{

    int err = 0;
    struct hpma115s0_data *data = (struct hpma115s0_data *)dev->data;

    /* Fetch from the device */
    switch (chan)
    {
    case SENSOR_CHAN_PM_2_5:

        /* Power enable */
        if (data->gpio != NULL)
            gpio_pin_set(data->gpio, data->power_en_pin, 1);

        k_sleep(K_MSEC(3980));

        /* Reset memory before recieving */
        memset(data->rx_buf, 0, sizeof(data->rx_buf));

        /* Disable first */
        uart_rx_disable(data->uart_dev);

        /* Start UART rx and read bytes */
        err = uart_rx_enable(data->uart_dev, data->rx_buf, FRAME_SIZE + 1, RX_TIMEOUT);
        if (err)
        {

            /* Power off */
            if (data->gpio != NULL)
                gpio_pin_set(data->gpio, data->power_en_pin, 0);

            LOG_ERR("Unable to recieve bytes! Err %i", err);
            return err;
        }

        break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    return 0;
}

static int hpma115s0_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{

    struct hpma115s0_data *data = (struct hpma115s0_data *)dev->data;

    /* Return an error if not ready yet */
    if (!data->ready)
    {
        return -EBUSY;
    }

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_PM_2_5:
        memcpy(val, &data->pm25, sizeof(struct sensor_value));
        break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    return 0;
}

static const struct sensor_driver_api hpma115s0_api = {
    .sample_fetch = &hpma115s0_sample_fetch,
    .channel_get = &hpma115s0_channel_get,
};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

    struct hpma115s0_data *data = (struct hpma115s0_data *)user_data;

    LOG_DBG("evt->type %d", evt->type);
    switch (evt->type)
    {
    case UART_TX_DONE:
        LOG_DBG("Tx sent %d bytes", evt->data.tx.len);
        break;

    case UART_TX_ABORTED:
        LOG_ERR("Tx aborted");
        break;

    case UART_RX_RDY:
        LOG_DBG("Received data %d bytes", evt->data.rx.len);

        for (int i = 0; i < evt->data.rx.len; i++)
        {
            if (evt->data.rx.buf[i] == 0x42 &&
                evt->data.rx.buf[i + 1] == 0x4d &&
                evt->data.rx.len - i >= FRAME_SIZE)
            {

                LOG_HEXDUMP_DBG(evt->data.rx.buf, evt->data.rx.len, "bytes: ");
                data->pm25.val1 = (evt->data.rx.buf[i + 6] << 8) + evt->data.rx.buf[i + 7];
                data->ready = true;

                break;
            }
        }

        /* Turn it off once we get data */
        if (data->gpio != NULL)
            gpio_pin_set(data->gpio, data->power_en_pin, 0);

        break;
    case UART_RX_BUF_REQUEST:
    case UART_RX_BUF_RELEASED:
    case UART_RX_DISABLED:
    case UART_RX_STOPPED:
        break;
    }
}

static int hpma115s0_init(const struct device *dev)
{
    int err = 0;
    struct hpma115s0_data *data = dev->data;

    data->uart_dev = device_get_binding(DT_INST_BUS_LABEL(0));

    if (data->uart_dev == NULL)
    {
        LOG_ERR("Unable to get UART device.");
        return -EINVAL;
    }

    /* Set the callback */
    err = uart_callback_set(data->uart_dev, uart_cb, data);
    __ASSERT(err == 0, "Failed to set callback");

    data->ready = false;

/* Configure GPIO */
#if DT_INST_PROP_HAS_IDX(0, enable_gpios, 0)
    data->power_en_dev_name = DT_INST_GPIO_LABEL(0, enable_gpios);
    data->power_en_pin = DT_INST_GPIO_PIN(0, enable_gpios);
    data->power_en_flags = DT_INST_GPIO_FLAGS(0, enable_gpios);
    data->gpio = device_get_binding(data->power_en_dev_name);
    if (data->gpio == NULL)
        LOG_WRN("Power enable pin is not defined!");

    /* Default off */
    gpio_pin_configure(data->gpio, data->power_en_pin, GPIO_OUTPUT_INACTIVE);
#endif

    return 0;
}

static struct hpma115s0_data hpma115s0_data;

DEVICE_DEFINE(hpma115s0, DT_INST_LABEL(0),
              hpma115s0_init, NULL,
              &hpma115s0_data, NULL, POST_KERNEL,
              CONFIG_SENSOR_INIT_PRIORITY, &hpma115s0_api);
