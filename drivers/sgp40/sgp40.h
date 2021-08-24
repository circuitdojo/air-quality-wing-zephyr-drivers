/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#ifndef _SGP40_H
#define _SGP40_H

#define SGP40_MEAS_RAW_NO_HUM_OR_TEMP_CMD               \
    {                                                   \
        0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93, \
    }
#define SGP40_HEATER_OFF_CMD \
    {                        \
        0x36, 0x15           \
    }
#define SGP40_SOFT_RST_CMD \
    {                      \
        0x00, 0x06         \
    }

/* Additional custom attributes */
enum sgp40_attribute
{
    SGP40_ATTR_RAW_HUM_TEMP = SENSOR_ATTR_PRIV_START,
};

#endif /*_SGP40_H*/