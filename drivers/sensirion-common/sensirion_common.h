#ifndef _SENSIRION_COMMON_H
#define _SENSIRION_COMMON_H

static uint8_t sensirion_calc_crc(uint8_t *data)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31u;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

#endif