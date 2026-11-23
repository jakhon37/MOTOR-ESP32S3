// MOTOR-ESP32S3/lib/CommandParser/CommandParser.h
// One line of the serial protocol. No Arduino dependency, so it can be tested on a PC.
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

enum class CommandType {
    Invalid,      // malformed: ignored, and does not count as a command for the timeout
    Velocity,     // m <left> <right>     wheel speeds in m/s, in a and b
    ForcePwm,     // f <pwm> | f <l> <r>  open-loop PWM test, in a and b
    Diagnostics,  // d                    print state, counts and encoder pin activity
    GetTuning,    // g                    print the tuning values
    SetTuning,    // g <name> <value>     change one until the next reboot, in name and a
};

struct Command {
    CommandType type = CommandType::Invalid;
    float a = 0;
    float b = 0;
    char name[12] = "";
};

Command parseCommand(const char* line);

#endif
