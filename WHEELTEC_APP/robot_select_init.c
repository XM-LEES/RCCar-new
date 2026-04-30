#include "robot_select_init.h"

RobotParmentType_t RobotHardWareParam = {0};

RobotControlParmentType_t RobotControlParam = {
    .Vol = 0.0f,
    .SecurityLevel = 0,
    .softwareEnflag = 0,
    .DebugLevel = 0,
    .LowPowerFlag = 0,
};

static void Robot_Init(float wheelspacing, float axlespacing, float tyre_diameter, uint8_t wheel_type)
{
    RobotHardWareParam.WheelSpacing = wheelspacing;
    RobotHardWareParam.AxleSpacing = axlespacing;
    RobotHardWareParam.WheelPerimeter = tyre_diameter * PI;
    RobotHardWareParam.wheeltype = wheel_type;
}

void Robot_Select(void)
{
    /* Ackermann-only platform.
     * Geometry comes from EXTRINSICS.md:
     * track width = 0.48 m
     * wheelbase   = 0.54 m
     * wheel radius = 0.11 m
     */
    RobotHardWareParam.CarType = SX04;
    RobotHardWareParam.driveCounts = 0U;
    Robot_Init(0.48f, 0.54f, 0.22f, SingleAxis_8inch);
}
