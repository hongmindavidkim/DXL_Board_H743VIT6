#include <dxl_startup.h>
#include "main.h"

extern XM430_bus dxl_bus_1;
extern uint8_t dxl_ID1[];
extern uint8_t idLength1;
extern uint8_t DXL_MODE;
float home_joint_pos[7] = {0.0f, 0.4f, -0.4f, 0.0f, 0.0f, 0.0f, 0.0f}; //0.4 -0.4 for left // -0.4 0.4 for right
float end_joint_pos[7] = {0.0f, 1.57f, -1.57f, 0.0f, 0.0f, 0.0f, 0.0f};//1.57, -1.57 for left // -1.57, 1.57 for right

void Dynamixel_Shutdown_Routine(){
	// disable all of the motors
//	for (int i=0; i<idLength1; i++) {
//		dxl_bus_1.SetTorqueEn(dxl_ID1[i],0x00);
//		HAL_Delay(10);
//	}
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.SetVelocityProfile(dxl_ID1[i], 40); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
		dxl_bus_1.SetAccelerationProfile(dxl_ID1[i], 15); // 80(17166) rev/min^2
		HAL_Delay(10);
	}
	int32_t end_motor_pos[7];
	JointPos2MotorPos(end_joint_pos, end_motor_pos);
	dxl_bus_1.SetMultGoalPositions(dxl_ID1, idLength1, (uint32_t*)end_motor_pos);
	HAL_Delay(3000);

}

void Dynamixel_Startup_Routine (bool torque_disable){
	// Enable dynamixels and set control mode...individual version
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.TurnOnLED(dxl_ID1[i], 0x00); // turn off LED
		dxl_bus_1.SetTorqueEn(dxl_ID1[i],0x00);
		dxl_bus_1.SetRetDelTime(dxl_ID1[i],0x02); // 4us delay time
		if (i==6){
		dxl_bus_1.SetControlMode(dxl_ID1[i],0x05);
		}else{dxl_bus_1.SetControlMode(dxl_ID1[i], DXL_MODE);}
		// set up indirect addresses for faster writing
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 168,  84); // KP
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 170,  85);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 172,  80); // KD
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 174,  81);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 176, 116); // goal position
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 178, 117);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 180, 118);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 182, 119);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 184, 104); // goal velocity
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 186, 105);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 188, 106);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 190, 107);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 192,  94); // feedforward current
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 194,  95);
		// re-enable motor
		HAL_Delay(100);
		dxl_bus_1.TurnOnLED(dxl_ID1[i], 0x01);
		dxl_bus_1.SetTorqueEn(dxl_ID1[i],0x01); // to be able to move
		HAL_Delay(100);
	}

	// set smooth DXL profile
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.SetVelocityProfile(dxl_ID1[i], 40); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
		dxl_bus_1.SetAccelerationProfile(dxl_ID1[i], 15); // 80(17166) rev/min^2
		HAL_Delay(100);
	}
//	dxl_bus_1.SetGoalCurrent(dxl_ID1[6], 800);
//	dxl_bus_1.SetPosPGain(dxl_ID1[6], 800);
//	dxl_bus_1.SetPosDGain(dxl_ID1[6], 4000);
	// controlled setup to send fingers to zero joint angles
	int32_t home_motor_pos[7];
	JointPos2MotorPos(home_joint_pos, home_motor_pos);
//	int32_t pos1[3];
//	int32_t pos2[3];
//	for (int i=0; i<3; i++) {
//		pos1[i] = home_motor_pos[i];
//		pos2[i] = home_motor_pos[i+4];
//	}
//	int32_t pos3[2];
//	pos3[0] = home_motor_pos[3];
//	pos3[1] = home_motor_pos[7];
	if (!torque_disable){
		dxl_bus_1.SetMultGoalPositions(dxl_ID1, idLength1, (uint32_t*)home_motor_pos);
		HAL_Delay(3000);
	}

	// re-set to fast DXL profile, if not in current control mode set current limit
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.SetVelocityProfile(dxl_ID1[i], 0);
		dxl_bus_1.SetAccelerationProfile(dxl_ID1[i], 0);
//		dxl_bus_1.SetPosPGain(dxl_ID1[i], 0);
//		dxl_bus_1.SetPosDGain(dxl_ID1[i], 0);
		if(DXL_MODE!=0x00){ dxl_bus_1.SetGoalCurrent(dxl_ID1[i], 1193); }
		dxl_bus_1.SetGoalCurrent(dxl_ID1[6], 250);
		HAL_Delay(100);
	}

	// motors are enabled by default on startup, but if torque_disable flag is set they can be disabled after the setup
	if (torque_disable){
		Dynamixel_Shutdown_Routine();
	}

	printf("Start Up Routine Finished!!\r\n");

}
