/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#define DT_DRV_COMPAT honeywell_hpma115s0

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

#include "hpma115s0.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hpma115s0, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_SOC_FAMILY_STM32
#define RX_TIMEOUT SYS_FOREVER_US
#else
#define RX_TIMEOUT 10000
#endif

#define FRAME_SIZE 32

struct hpma115s0_config
{
    const struct device *bus;
    const struct gpio_dt_spec power_en_pin;
};

struct hpma115s0_data
{
    struct sensor_value pm25;
    bool ready;
    uint8_t rx_buf[FRAME_SIZE * 2];
};

static int hpma115s0_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan)
{

    int err = 0;
    struct hpma115s0_data *data = dev->data;
    const struct hpma115s0_config *config = dev->config;

    /* Fetch from the device */
    switch (chan)
    {
    case SENSOR_CHAN_PM_2_5:

        /* Power enable */
        if (config->power_en_pin.port != NULL)
            gpio_pin_set_dt(&config->power_en_pin, 1);

        k_sleep(K_MSEC(3980));

        /* Reset memory before recieving */
        memset(data->rx_buf, 0, sizeof(data->rx_buf));

        /* Disable first */
        uart_rx_disable(config->bus);

        /* Start UART rx and read bytes */
        err = uart_rx_enable(config->bus, data->rx_buf, FRAME_SIZE + 1, RX_TIMEOUT);
        if (err)
        {

            /* Power off */
            if (config->power_en_pin.port != NULL)
                gpio_pin_set_dt(&config->power_en_pin, 0);

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

    struct hpma115s0_data *data = dev->data;
    const struct hpma115s0_config *config = dev->config;

    /* Return an error if not ready yet */
    if (!data->ready)
    {
        return -EBUSY;
    }

    /* Get the data depending on which channel */
    switch (chan)
    {
    case SENSOR_CHAN_PM_2_5:
        /* Copy it over */
        *val = data->pm25;
        data->ready = false;
        break;
    default:
        LOG_WRN("Invalid sensor_channel %i", chan);
        return -EINVAL;
    }

    /* Turn it off once we get data */
    if (config->power_en_pin.port != NULL)
        gpio_pin_set_dt(&config->power_en_pin, 0);

    return 0;
}

static const struct sensor_driver_api hpma115s0_api = {
    .sample_fetch = &hpma115s0_sample_fetch,
    .channel_get = &hpma115s0_channel_get,
};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

    struct hpma115s0_data *data = user_data;

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

        break;
    case UART_RX_STOPPED:
        break;
    case UART_RX_BUF_REQUEST:
    case UART_RX_BUF_RELEASED:
    case UART_RX_DISABLED:
        break;
    }
}

static int hpma115s0_init(const struct device *dev)
{
    int err = 0;
    struct hpma115s0_data *data = dev->data;
    const struct hpma115s0_config *config = dev->config;

    if (!device_is_ready(config->bus))
    {
        LOG_ERR("Unable to get UART device.");
        return -EIO;
    }

    /* Set the callback */
    err = uart_callback_set(config->bus, uart_cb, data);
    __ASSERT(err == 0, "Failed to set callback");

    data->ready = false;

    /* Configure GPIO */
    if (config->power_en_pin.port != NULL)
    {
        if (!device_is_ready(config->power_en_pin.port))
        {
            LOG_ERR("Unable to configure power enable pin.");
            return -EIO;
        }

        /* Default off */
        gpio_pin_configure_dt(&config->power_en_pin, GPIO_OUTPUT_INACTIVE);
    }

    return 0;
}

/* Main instantiation macro */
#define HPMA115S0_DEFINE(inst)                                                           \
    static struct hpma115s0_data hpma115s0_data_##inst;                                  \
    static const struct hpma115s0_config hpma115s0_config_##inst = {                     \
        .bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                                         \
        .power_en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, enable_gpios, {0}),               \
    };                                                                                   \
    DEVICE_DT_INST_DEFINE(inst,                                                          \
                          hpma115s0_init, NULL,                                          \
                          &hpma115s0_data_##inst, &hpma115s0_config_##inst, POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, &hpma115s0_api);

/* Create the struct device for every status "okay"*/
DT_INST_FOREACH_STATUS_OKAY(HPMA115S0_DEFINE)