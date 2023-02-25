/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#ifndef _SHTC3_H
#define _SHTC3_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#define SHTC3_TEMP_HOLD_CMD \
    {                       \
        0x7C, 0xA2          \
    }
#define SHTC3_HUMIDITY_HOLD_CMD \
    {                           \
        0x5C, 0x24              \
    }

#define SHTC3_SLEEP \
    {               \
        0xB0, 0x98  \
    }

#define SHTC3_WAKE \
    {              \
        0x35, 0x17 \
    }

/* Additional custom attributes */
enum shtc3_attribute
{
    SHTC3_ATTR_USE_RAW = SENSOR_ATTR_PRIV_START,
};

#endif /*_SHTC3_H*/