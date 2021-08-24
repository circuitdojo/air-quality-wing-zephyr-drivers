/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#ifndef _AQW_H
#define _AQW_H

#include <zephyr/types.h>
#include <drivers/sensor.h>

/**
 * @brief Defines all the sensors possible
 * 
 */
enum aqw_sensor_type
{
    AQW_INVALID_SENSOR = 0,
    AQW_TEMPERATURE_SENSOR,
    AQW_HUMIDITY_SENSOR,
    AQW_PM25_SENSOR,
    AQW_VOC_SENSOR,
    AQW_SENSOR_TYPE_LAST
};

/**
 * @brief Sensor data that is shared with other contexts
 * 
 */
struct aqw_sensor_data
{
    enum aqw_sensor_type type;
    struct sensor_value val;
    uint64_t ts;
};

/**
 * @brief Sensor organization so it can be placed in an array
 * 
 */
struct aqw_sensor
{
    enum aqw_sensor_type type;
    enum sensor_channel chan;
    uint8_t *dev_name;
    const struct device *dev;
};

/**
 * @brief Callback to main context when data is ready
 * 
 */
typedef void (*aqw_sensor_data_ready_t)(struct aqw_sensor_data *data, size_t len);

/**
 * @brief Air Quality Wing init function
 * 
 * @param _sensors structure of all sensors defined. (pointer to an array of pointers)
 * @param _sensor_count number of sensors to init
 * @param _cb optional callback function for sending data
 * @return int 0 on success
 */
int aqw_init(struct aqw_sensor **_sensors, size_t _sensor_count, aqw_sensor_data_ready_t _cb);

/**
 * @brief Air Quality Wing fetch function for starting async data measurement
 * 
 * @return int 0 on success
 */
int aqw_sensor_start_fetch(void);

/**
 * @brief Returns pointer to static name string depending on sensor type 
 * 
 * @param type aqw_sensor_type enum
 * @return char* returned string pointer
 */
char *aqw_sensor_type_to_string(enum aqw_sensor_type type);

/**
 * @brief Returns pointer to static unit string depending on sensor type 
 * 
 * @param type aqw_sensor_type enum
 * @return char* returned string pointer
 */
char *aqw_sensor_unit_to_string(enum aqw_sensor_type type);

#endif /* _AQW_H */