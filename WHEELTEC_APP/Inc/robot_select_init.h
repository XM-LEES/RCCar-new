#ifndef __ROBOT_SELECT_INIT_H
#define __ROBOT_SELECT_INIT_H

#include <stdint.h>

#define PI 3.14159265358979f

typedef enum
{
    SingleAxis_5inch = 0,
    DoubleAxis_5inch,
    SingleAxis_8inch,
    DoubleAxis_8inch
} RobotWheelType_t;

typedef struct
{
    uint8_t CarType;
    uint8_t driveCounts;
    uint8_t wheeltype;
    float WheelSpacing;
    float AxleSpacing;
    float WheelPerimeter;
} RobotParmentType_t;

typedef enum
{
    S300,
    S200,
    S200_OUTDOOR,
    S300Mini,
    S100,
    S260,
    SX04,
    Number_of_CAR
} CAR_MODE;

extern RobotParmentType_t RobotHardWareParam;

typedef struct
{
    float Vol;
    uint8_t SecurityLevel;
    uint8_t softwareEnflag;
    uint8_t DebugLevel;
    uint8_t LowPowerFlag;
} RobotControlParmentType_t;

void Robot_Select(void);
extern RobotControlParmentType_t RobotControlParam;

#endif
