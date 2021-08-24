/**
 * @author Jared Wolff (hello@jaredwolff.com)
 * @copyright Copyright Circuit Dojo LLC 2021
 */

#ifndef _HPMA115S0_H
#define _HPMA115S0_H

#define HPMA115S0_ENABLE_AUTO_SEND_CMD \
    {                                  \
        0x68, 0x01, 0x40, 0x57         \
    }

#define HPMA115S0_DISABLE_AUTO_SEND_CMD \
    {                                   \
        0x68, 0x01, 0x20, 0x77          \
    }

#define HPMA115S0_ENABLE_MEASUREMENT_CMD \
    {                                    \
        0x68, 0x01, 0x01, 0x96           \
    }

#define HPMA115S0_DISABLE_MEASUREMENT_CMD \
    {                                     \
        0x68, 0x01, 0x02, 0x95            \
    }

#define HPMA115S0_READ_CMD     \
    {                          \
        0x68, 0x01, 0x04, 0x93 \
    }

#endif /*_HPMA115S0_H*/