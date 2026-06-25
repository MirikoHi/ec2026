#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define BSP_LOG_MESSAGE_CAPACITY 192
#define BSP_LOG_FLOAT_PRECISION_DEFAULT 3
#define BSP_LOG_FLOAT_PRECISION_MAX 6

static void BSPLogAppendChar(char *buffer, size_t capacity, size_t *length, char ch)
{
    if ((*length + 1u) >= capacity) {
        return;
    }
    buffer[*length] = ch;
    (*length)++;
    buffer[*length] = '\0';
}

static void BSPLogAppendString(char *buffer, size_t capacity, size_t *length, const char *text)
{
    if (text == NULL) {
        text = "(NULL)";
    }

    while (*text != '\0') {
        if ((*length + 1u) >= capacity) {
            return;
        }
        buffer[*length] = *text;
        (*length)++;
        text++;
    }
    buffer[*length] = '\0';
}

static void BSPLogAppendUnsigned(char *buffer, size_t capacity, size_t *length, unsigned long long value, unsigned int base, int uppercase)
{
    char digits[32];
    const char *table;
    size_t count = 0u;

    table = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    if (value == 0u) {
        BSPLogAppendChar(buffer, capacity, length, '0');
        return;
    }

    while ((value != 0u) && (count < sizeof(digits))) {
        digits[count++] = table[value % base];
        value /= base;
    }

    while (count > 0u) {
        BSPLogAppendChar(buffer, capacity, length, digits[--count]);
    }
}

static void BSPLogAppendSigned(char *buffer, size_t capacity, size_t *length, long long value)
{
    unsigned long long magnitude;

    if (value < 0) {
        BSPLogAppendChar(buffer, capacity, length, '-');
        magnitude = (unsigned long long)(-(value + 1)) + 1u;
    } else {
        magnitude = (unsigned long long)value;
    }

    BSPLogAppendUnsigned(buffer, capacity, length, magnitude, 10u, 0);
}

static void BSPLogAppendFloat(char *buffer, size_t capacity, size_t *length, double value, unsigned int precision)
{
    unsigned long long integer_part;
    double fractional;
    unsigned long long scale = 1u;
    unsigned long long fractional_part;
    unsigned int i;

    if (precision > BSP_LOG_FLOAT_PRECISION_MAX) {
        precision = BSP_LOG_FLOAT_PRECISION_MAX;
    }

    if (value < 0.0) {
        BSPLogAppendChar(buffer, capacity, length, '-');
        value = -value;
    }

    integer_part = (unsigned long long)value;
    fractional = value - (double)integer_part;

    for (i = 0u; i < precision; ++i) {
        scale *= 10u;
    }

    fractional_part = (unsigned long long)(fractional * (double)scale + 0.5);
    if (fractional_part >= scale) {
        integer_part += 1u;
        fractional_part -= scale;
    }

    BSPLogAppendUnsigned(buffer, capacity, length, integer_part, 10u, 0);

    if (precision == 0u) {
        return;
    }

    BSPLogAppendChar(buffer, capacity, length, '.');

    for (i = precision; i > 0u; --i) {
        unsigned long long divisor = 1u;
        unsigned int j;

        for (j = 1u; j < i; ++j) {
            divisor *= 10u;
        }
        BSPLogAppendChar(buffer, capacity, length, (char)('0' + (fractional_part / divisor) % 10u));
    }
}

static const char *BSPLogSkipFlagsAndWidth(const char *fmt, unsigned int *precision)
{
    while ((*fmt == '-') || (*fmt == '+') || (*fmt == ' ') || (*fmt == '#') || (*fmt == '0')) {
        ++fmt;
    }

    while ((*fmt >= '0') && (*fmt <= '9')) {
        ++fmt;
    }

    *precision = BSP_LOG_FLOAT_PRECISION_DEFAULT;
    if (*fmt == '.') {
        unsigned int parsed = 0u;

        ++fmt;
        parsed = 0u;
        while ((*fmt >= '0') && (*fmt <= '9')) {
            parsed = parsed * 10u + (unsigned int)(*fmt - '0');
            ++fmt;
        }
        *precision = parsed;
    }

    while ((*fmt == 'l') || (*fmt == 'h') || (*fmt == 'z') || (*fmt == 't') || (*fmt == 'j') || (*fmt == 'L')) {
        ++fmt;
    }

    return fmt;
}

static int BSPLogFormat(char *buffer, size_t capacity, const char *fmt, va_list args)
{
    size_t length = 0u;
    const char *cursor = fmt;

    if (capacity == 0u) {
        return 0;
    }

    buffer[0] = '\0';

    while (*cursor != '\0') {
        unsigned int precision;

        if (*cursor != '%') {
            BSPLogAppendChar(buffer, capacity, &length, *cursor++);
            continue;
        }

        ++cursor;
        if (*cursor == '%') {
            BSPLogAppendChar(buffer, capacity, &length, *cursor++);
            continue;
        }

        cursor = BSPLogSkipFlagsAndWidth(cursor, &precision);

        switch (*cursor) {
        case 'd':
        case 'i':
            BSPLogAppendSigned(buffer, capacity, &length, (long long)va_arg(args, int));
            break;
        case 'u':
            BSPLogAppendUnsigned(buffer, capacity, &length, (unsigned long long)va_arg(args, unsigned int), 10u, 0);
            break;
        case 'x':
            BSPLogAppendUnsigned(buffer, capacity, &length, (unsigned long long)va_arg(args, unsigned int), 16u, 0);
            break;
        case 'X':
            BSPLogAppendUnsigned(buffer, capacity, &length, (unsigned long long)va_arg(args, unsigned int), 16u, 1);
            break;
        case 'c':
            BSPLogAppendChar(buffer, capacity, &length, (char)va_arg(args, int));
            break;
        case 's':
            BSPLogAppendString(buffer, capacity, &length, va_arg(args, const char *));
            break;
        case 'p':
            BSPLogAppendString(buffer, capacity, &length, "0x");
            BSPLogAppendUnsigned(buffer, capacity, &length, (uintptr_t)va_arg(args, void *), 16u, 0);
            break;
        case 'f':
        case 'F':
            BSPLogAppendFloat(buffer, capacity, &length, va_arg(args, double), precision);
            break;
        default:
            BSPLogAppendChar(buffer, capacity, &length, '%');
            if (*cursor != '\0') {
                BSPLogAppendChar(buffer, capacity, &length, *cursor);
            }
            break;
        }

        if (*cursor != '\0') {
            ++cursor;
        }
    }

    return (int)length;
}

void BSPLogInit()
{
    if (_SEGGER_RTT.acID[0] != 'S') {
        SEGGER_RTT_Init();
    }
}

int BSPLogVPrintf(const char *type, const char *color, const char *fmt, va_list args)
{
    char message[BSP_LOG_MESSAGE_CAPACITY];
    int count;
    va_list args_copy;

    va_copy(args_copy, args);
    count = BSPLogFormat(message, sizeof(message), fmt, args_copy);
    va_end(args_copy);

    SEGGER_RTT_WriteString(BUFFER_INDEX, "  ");
    SEGGER_RTT_WriteString(BUFFER_INDEX, color);
    SEGGER_RTT_WriteString(BUFFER_INDEX, type);
    SEGGER_RTT_WriteString(BUFFER_INDEX, message);
    SEGGER_RTT_WriteString(BUFFER_INDEX, "\r\n");
    SEGGER_RTT_WriteString(BUFFER_INDEX, RTT_CTRL_RESET);

    return count;
}

int BSPLogPrintf(const char *type, const char *color, const char *fmt, ...)
{
    va_list args;
    int count;

    va_start(args, fmt);
    count = BSPLogVPrintf(type, color, fmt, args);
    va_end(args);

    return count;
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    char message[BSP_LOG_MESSAGE_CAPACITY];
    int count;

    va_start(args, fmt);
    count = BSPLogFormat(message, sizeof(message), fmt, args);
    va_end(args);

    SEGGER_RTT_WriteString(BUFFER_INDEX, message);
    return count;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}
