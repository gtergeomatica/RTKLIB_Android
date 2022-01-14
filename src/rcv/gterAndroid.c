#include "../rtklib.h"

#define AYSYNC1    65        /* yet message sync code 1 */
#define AYSYNC2    89        /* yet message sync code 2 */

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }

static double asc2dbl(int digits, uint8_t* input)
{
    double retVal = 0.0;
    for (int i = 0; i < digits; i++)
    {
        double tmp = (input[i] - 48);
        for (int j = 1; j < digits - i; j++)
            tmp *= 10.0;

        retVal += tmp;
    }
    return retVal;
}

static long asc2long(int digits, uint8_t* input)
{
    long retVal = 0;
    for (int i = 0; i < digits; i++)
    {
        long tmp = (input[i] - 48);
        for (int j = 1; j < digits - i; j++)
            tmp *= 10;

        retVal += tmp;
    }
    return retVal;
}

static double R8u(uint8_t* p)
{
    double retVal = 0;
    for (int i = 0; i < 8; i++)
        retVal += (p[i] * pow(16, i * 2));

    return retVal;
}

static double R8s(uint8_t* p)
{
    double fact = 1.0;
    double delta = 0.0;
    if ((p[7] & 128) == 128)
    {
        for (int i = 0; i < 8; i++)
            p[i] = 255 - p[i];

        fact = -1.0;
        delta = 1.0;
    }

    double retVal = 0;
    for (int i = 0; i < 8; i++)
        retVal += (p[i] * pow(16, i * 2));

    return (retVal + delta) * fact;
}

static int sync_gterAndroid(uint8_t* buff, uint8_t data)
{
    // code here to sync stream with start of next message
    //buff[0] = buff[1]; buff[1] = data;
    //return buff[0] == AYSYNC1 && buff[1] == AYSYNC2;
}

static int decode_gterAndroid(raw_t* raw)
{
    // code here to decode android message
}

extern int input_gterAndroid(raw_t* raw, uint8_t data)
{
    trace(5, "input_gterAndroid: data=%02x\n", data);

    /* synchronize frame */
    if (raw->nbyte == 0) {
        if (!sync_gterAndroid(raw->buff, data)) return 0;
        raw->nbyte = 2;
        return 0;
    }
    raw->buff[raw->nbyte++] = data;

    if (raw->nbyte == 4) {
        if ((raw->len = U2(&raw->buff[2])) > MAXRAWLEN) { // warning: modify this
            trace(2, "ay length error: len=%d\n", raw->len);
            raw->nbyte = 0;
            return -1;
        }
    }
    if (raw->nbyte < 4 || raw->nbyte < raw->len) return 0;
    raw->nbyte = 0;

    /* decode ublox raw message */
    return decode_gterAndroid(raw);
}