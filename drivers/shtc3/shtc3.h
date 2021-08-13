/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#ifndef _SHTC3_H
#define _SHTC3_H

#define SHTC3_TEMP_HOLD_CMD \
    {                       \
        0x7C, 0xA2          \
    }
#define SHTC3_HUMIDITY_HOLD_CMD \
    {                           \
        0x5C, 0x24              \
    }

#endif /*_SHTC3_H*/