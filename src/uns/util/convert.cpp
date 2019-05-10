#include <uns/util/convert.hpp>
#include <uns/util/numeric.hpp>
#include <uns/util/arrays.hpp>

#include <cstring>
#include <cstdlib>

using namespace uns;

void uns::toBytes(int16_t value, uint8_t bytes[], BitOrder order) {
    if (order == BitOrder::ENDIAN_LITTLE) {
        bytes[0] = static_cast<uint8_t>(value);
        bytes[1] = static_cast<uint8_t>(value >> 8);
    } else {    // ENDIAN_BIG
        bytes[1] = static_cast<uint8_t>(value);
        bytes[0] = static_cast<uint8_t>(value >> 8);
    }
}

void uns::toBytes(int32_t value, uint8_t bytes[], BitOrder order) {
    if (order == BitOrder::ENDIAN_LITTLE) {
        bytes[0] = static_cast<uint8_t>(value);
        bytes[1] = static_cast<uint8_t>(value >> 8);
        bytes[2] = static_cast<uint8_t>(value >> 16);
        bytes[3] = static_cast<uint8_t>(value >> 24);
    } else {    // ENDIAN_BIG
        bytes[3] = static_cast<uint8_t>(value);
        bytes[2] = static_cast<uint8_t>(value >> 8);
        bytes[1] = static_cast<uint8_t>(value >> 16);
        bytes[0] = static_cast<uint8_t>(value >> 24);
    }
}

void uns::toBytes(float32_t value, uint8_t bytes[], BitOrder order) {
    int32_t intVal;
    memcpy(&intVal, &value, 4);
    toBytes(intVal, bytes, order);
}

int16_t uns::toInt16(const uint8_t bytes[], BitOrder order) {
    int16_t result;
    if (order == BitOrder::ENDIAN_LITTLE) {
        result = static_cast<int32_t>(bytes[0])
            | (static_cast<int32_t>(bytes[1]) << 8);
    } else {    // ENDIAN_BIG
        result = static_cast<int32_t>(bytes[1])
            | (static_cast<int32_t>(bytes[0]) << 8);
    }
    return result;
}

int32_t uns::toInt32(const uint8_t bytes[], BitOrder order) {
    int32_t result;
    if (order == BitOrder::ENDIAN_LITTLE) {
        result = static_cast<int32_t>(bytes[0])
            | (static_cast<int32_t>(bytes[1]) << 8)
            | (static_cast<int32_t>(bytes[2]) << 16)
            | (static_cast<int32_t>(bytes[3]) << 24);
    } else {    // ENDIAN_BIG
        result = static_cast<int32_t>(bytes[3])
            | (static_cast<int32_t>(bytes[2]) << 8)
            | (static_cast<int32_t>(bytes[1]) << 16)
            | (static_cast<int32_t>(bytes[0]) << 24);
    }
    return result;
}

float32_t uns::toFloat32(const uint8_t bytes[], BitOrder order) {
    int32_t intVal = toInt32(bytes, order);
    float32_t resultFloatVal;
    memcpy(&resultFloatVal, &intVal, 4);
    return resultFloatVal;
}

uint32_t uns::atoi(const char * const s, int32_t *pResult, uint32_t len) {
    uint32_t idx = 0;

    if (!len) {
        len = strlen(s);
    }

    if (len > 0) {
        bool neg = s[0] == '-';
        if (neg) {
            idx = 1;
        }
        *pResult = 0;
        for (; idx < len; ++idx) {
            char c = s[idx];
            if (c < '0' || c > '9') {
                break;
            }
            *pResult *= 10;
            *pResult += static_cast<int32_t>(c - '0');
        }

        if (neg) {
            *pResult *= -1;
        }
    }

    return idx;
}

uint32_t uns::atof(const char * const s, float32_t *pResult, uint32_t len) {
    int32_t dec, frac;

    if (!len) {
        len = strlen(s);
    }

    uint32_t idx = 0;
    bool neg = s[0] == '-';
    if (neg) {
        idx = 1;
    }

    idx += uns::atoi(&s[idx], &dec, len);

    if (++idx < len) {  // idx is incremented because of the dot character before the fraction
        len -= idx;     // calculates residual length

        uint32_t fracCount = uns::atoi(&s[idx], &frac, len);
        if (fracCount > 0) {
            idx += fracCount;
            *pResult = dec + frac / uns::powerOf(10.0f, fracCount);
        } else {
            idx = 0;    // if no fraction has been parsed, string is invalid
        }
    } else {
        len = 0;    // invalid floating point string
    }

    if (neg) {
        *pResult *= -1;
    }

    return idx;
}

uint32_t uns::itoa(int32_t n, char *const s, uint32_t numSize, uint32_t padding) {
    bool sign;

    if ((sign = n < 0))
        n = -n;

    uint32_t idx = 0;
    do {
        s[idx++] = '0' + (n % 10);
        if (padding) {
            --padding;
        }
    } while ((n /= 10) > 0 && idx < numSize) ;

    while(padding--) {
        s[idx++] = '0';
    }

    if (idx < numSize) {
        if (sign) {
            s[idx++] = '-';
        }
    } else if (sign || n != 0) {
        idx = 0;    // buffer full
    }

    s[idx] = '\0';

    uns::reverse(s, idx);
    return idx;
}

uint32_t uns::ftoa(float32_t n, char * const s, uint32_t decSize, uint32_t fracSize) {
    static const uint32_t padding = 4;

    uint32_t idx = 0;
    uint32_t decLen, fracLen;
    uint32_t sign;       // offset for sign

    if ((sign = (n < 0.0f) ? 1 : 0)) {
        n = -n;
        s[idx++] = '-';
    }

    int32_t dec = static_cast<int32_t>(n);
    int32_t frac = static_cast<int32_t>((n - static_cast<float32_t>(dec)) * uns::powerOf(10, padding));
    if ((decLen = uns::itoa(dec, &s[idx], decSize)) > 0) {
        idx += decLen;
        s[idx++] = '.';
        if ((fracLen = uns::itoa(frac, &s[idx], fracSize, padding)) > 0) {
            idx += fracLen;
        } else {
            idx = 0;
        }
    } else {
        idx = 0;
    }

    return idx;
}
