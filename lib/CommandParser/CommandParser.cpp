// MOTOR-ESP32S3/lib/CommandParser/CommandParser.cpp
#include "CommandParser.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char* skipSpaces(const char* s) {
    while (isspace((unsigned char)*s)) s++;
    return s;
}

// Reads one finite number and moves s past it.
static bool readFloat(const char*& s, float& value) {
    char* end;
    value = strtof(s, &end);
    if (end == s || !isfinite(value)) {
        return false;
    }
    s = end;
    return true;
}

Command parseCommand(const char* line) {
    Command cmd;
    const char* s = skipSpaces(line);
    const char letter = *s;
    if (letter == '\0') {
        return cmd;
    }
    s++;

    float a, b;
    CommandType type = CommandType::Invalid;
    switch (letter) {
        case 'm':
            if (!readFloat(s, a) || !readFloat(s, b)) return cmd;
            type = CommandType::Velocity;
            break;
        case 'f':
            if (!readFloat(s, a)) return cmd;
            if (!readFloat(s, b)) b = a;
            type = CommandType::ForcePwm;
            break;
        case 'd':
            a = b = 0;
            type = CommandType::Diagnostics;
            break;
        case 'g': {
            a = b = 0;
            s = skipSpaces(s);
            if (*s == '\0') {
                type = CommandType::GetTuning;
                break;
            }
            size_t len = 0;
            while (s[len] != '\0' && !isspace((unsigned char)s[len])) len++;
            if (len >= sizeof(cmd.name)) return cmd;
            memcpy(cmd.name, s, len);
            cmd.name[len] = '\0';
            s += len;
            if (!readFloat(s, a)) return cmd;
            type = CommandType::SetTuning;
            break;
        }
        default:
            return cmd;
    }

    // Trailing text means a damaged or merged line, like "m 0.1 0.1m 0.2 0.2".
    if (*skipSpaces(s) != '\0') {
        return cmd;
    }
    cmd.type = type;
    cmd.a = a;
    cmd.b = b;
    return cmd;
}
