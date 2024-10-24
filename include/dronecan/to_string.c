#include "to_string.h"

void uintToStringInBuffer(uint8_t value, char *buffer, size_t buffer_size) {
    int i = 0;
    if (value == 0) {
        if (buffer_size > 1) {
            buffer[i++] = '0';
            buffer[i] = '\0';
        }
    } else {
        while (value > 0 && i < buffer_size - 1) {
            buffer[i++] = (value % 10) + '0';
            value /= 10;
        }
        buffer[i] = '\0';
        for (int j = 0; j < i / 2; ++j) {
            char temp = buffer[j];
            buffer[j] = buffer[i - j - 1];
            buffer[i - j - 1] = temp;
        }
    }
}

void uint16ToStringInBuffer(uint16_t value, char *buffer, size_t buffer_size) {
    int i = 0;
    if (buffer_size == 0) {
        return;
    }

    if (value == 0) {
        if (buffer_size > 1) {
            buffer[i++] = '0';
            buffer[i] = '\0';
        }
    } else {
        while (value > 0 && i < buffer_size - 1) {
            buffer[i++] = (value % 10) + '0';
            value /= 10;
        }
        buffer[i] = '\0';

        for (int j = 0; j < i / 2; ++j) {
            char temp = buffer[j];
            buffer[j] = buffer[i - j - 1];
            buffer[i - j - 1] = temp;
        }
    }
}

void floatToStringInBuffer(char *buffer, size_t buffer_size, float value, int decimal_places) {
    if (decimal_places < 0) {
        decimal_places = 0;
    } else if (decimal_places > 11) {
        decimal_places = 11;
    }

    if (buffer_size < (14 + decimal_places)) {
        buffer[0] = '\0';
        return;
    }

    int is_negative = 0;
    if (value < 0) {
        is_negative = 1;
        value = -value;
    }

    int int_part = (int)value;
    float fractional_part = value - int_part;
    long long frac_part = (long long)(fractional_part * pow(10, decimal_places) + 0.5);

    int offset = 0;

    if (is_negative) {
        buffer[offset++] = '-';
    }

    if (int_part == 0) {
        buffer[offset++] = '0';
    } else {
        int num_digits = 0;
        int temp = int_part;

        while (temp > 0) {
            temp /= 10;
            num_digits++;
        }

        offset += num_digits;
        buffer[offset] = '\0';

        while (int_part > 0) {
            buffer[--offset] = (int_part % 10) + '0';
            int_part /= 10;
        }
        offset += num_digits;
    }

    if (decimal_places > 0) {
        buffer[offset++] = '.';

        for (int i = decimal_places - 1; i >= 0; i--) {
            buffer[offset + i] = (frac_part % 10) + '0';
            frac_part /= 10;
        }
        offset += decimal_places;
    }
    buffer[offset] = '\0';
}
