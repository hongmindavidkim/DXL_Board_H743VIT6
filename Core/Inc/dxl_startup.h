#include <XM430_bus.h>
#include "dxl_transforms.h"

void Dynamixel_Shutdown_Routine();
void Dynamixel_Startup_Routine (bool torque_enable);

#define LEFT 1
#define RIGHT 2

#define ARM LEFT // Choose Left or Right

#if (ARM == LEFT)
#define HOME_JOINT_POS_1 0.4f
#define HOME_JOINT_POS_2 -0.4f
#define END_JOINT_POS_1 1.57f
#define END_JOINT_POS_2 -1.57f
#endif

#if (ARM == RIGHT)
#define HOME_JOINT_POS_1 -0.4f
#define HOME_JOINT_POS_2 0.4f
#define END_JOINT_POS_1 -1.57f
#define END_JOINT_POS_2 1.57f
#endif
