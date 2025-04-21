#include "dxl_transforms.h"
#include "math.h"
#include <cstdint>

//Driver Pulley
float rm = 14.38f;   // Motor driver pulley diameter for MCP, PIP, DIP
float rmcr = 15.98f; // Motor driver pulley diameter for MCR

//MCR
float r1 = 15.98f;   // MCR output
float r12 = 16.38f;  // MCP routing
float r13 = 11.48f;  // PIP routing
float r14 = 8.08f;   // DIP routing

//MCP
float r2 = 14.38f;   // MCP output
float r23 = 9.98f;   // PIP routing
float r24 = 5.58f;   // DIP routing

//PIP
float r3 = 14.38f;   // PIP output
float r34 = 9.98f;   // DIP routing

//DIP
float r4 = 14.38f;   // DIP output

// from joint space to actuator space... phidot = Jact*thetadot
float JactL[4][4] = { {(14.38f/14.38f), 0.0f, 0.0f, -(16.38f/15.98f)},
                     {(9.98f/14.38f), (14.38f/14.38f), 0.0f, (11.48f/15.98f)},
                     {(5.58f/14.38f), (9.98f/14.38f), (14.38f/14.38f), -(8.08f/15.98f)},
                     {0.0f, 0.0f, 0.0f, (15.98f/15.98f)} };
float JactR[4][4] = { {(14.38f/14.38f), 0.0f, 0.0f, -(16.38f/15.98f)},
                     {(9.98f/14.38f), (14.38f/14.38f), 0.0f, (11.48f/15.98f)},
                     {(5.58f/14.38f), (9.98f/14.38f), (14.38f/14.38f), -(8.08f/15.98f)},
                     {0.0f, 0.0f, 0.0f, (15.98f/15.98f)} };

// from actuator space to joint space... thetadot = Jjoint*phidot
float JjointL[4][4] = {{1.0, 0.0f, 0.0f, 1.025031289f},
                     {-0.694019471f, 1.0f, 0.0f, -1.429789671f},
                     {0.0936240838f, -0.6940194714f, 1.0f, 1.1001818539f},
                     {0.0f, 0.0f, 0.0f, 1.0f}};
float JjointR[4][4] = {{1.0, 0.0f, 0.0f, 1.025031289f},
                     {-0.694019471f, 1.0f, 0.0f, -1.429789671f},
                     {0.0936240838f, -0.6940194714f, 1.0f, 1.1001818539f},
                     {0.0f, 0.0f, 0.0f, 1.0f}};

// transform positions from joint-space to motor-space
void JointPos2MotorPos(float* joint_pos_in, int32_t* motor_pos_out){
    for(int i=0;i<7;i++){
    	motor_pos_out[i] = (int32_t)round(rad2pulse(joint_pos_in[i]));
    }
}

// transform positions from motor-space to joint-space
void MotorPos2JointPos(int32_t* motor_pos_in, float* joint_pos_out){
    for(int i=0; i<7; i++){
    	joint_pos_out[i] = pulse2rad(motor_pos_in[i]);
    }

}

// transform velocities from joint-space to motor-space
void JointVel2MotorVel(float* joint_vel_in, int32_t* motor_vel_out){
    // TODO: implement this if necessary
    for (int i=0; i<8; i++){
        motor_vel_out[i] = (int32_t)joint_vel_in[i];
    }
}

// transform velocities from motor-space to joint-space
void MotorVel2JointVel(int32_t* motor_vel_in, float* joint_vel_out){
    for(int k=0; k<7; k++){
    	joint_vel_out[k] = rpm2rads(motor_vel_in[k]);
    }
}

// transform torques from joint-space to motor-space
void JointTau2MotorTau(float* joint_tau_in, float* motor_tau_out){
    // joint_tau_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
    // motor_tau_out: same as joint_tau_in

    // motor_tau = Jjoint^T * joint_tau
    for(int i=0; i<4; i++){
        // Left finger
        motor_tau_out[i] = JjointL[0][i]*joint_tau_in[0] + JjointL[1][i]*joint_tau_in[1] 
                            + JjointL[2][i]*joint_tau_in[2] + JjointL[3][i]*joint_tau_in[3];
        // Right finger
        motor_tau_out[i+4] = JjointR[0][i]*joint_tau_in[4] + JjointR[1][i]*joint_tau_in[5] 
                            + JjointR[2][i]*joint_tau_in[6] + JjointR[3][i]*joint_tau_in[7];
    }
}

//// transform torques from motor-space to joint-space
//void MotorTau2JointTau(float* motor_tau_in, float* joint_tau_out){
//    // motor_tau_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
//    // joint_tau_out: same as motor_tau_in
//
//    // joint_tau = Jact^T * motor_tau
//    for(int i=0; i<4; i++){
//        // Left finger
//		joint_tau_out[i] = JactL[0][i]*motor_tau_in[0] + JactL[1][i]*motor_tau_in[1]
//                            + JactL[2][i]*motor_tau_in[2] + JactL[3][i]*motor_tau_in[3];
//        // Right finger
//		joint_tau_out[i] = JactR[0][i]*motor_tau_in[4] + JactR[1][i]*motor_tau_in[5]
//                            + JactR[2][i]*motor_tau_in[6] + JactR[3][i]*motor_tau_in[7];
//	}
//}

void MotorTau2JointTau(float* motor_tau_in, float* joint_tau_out){
    // motor_tau_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
    // joint_tau_out: same as motor_tau_in

    // joint_tau = Jact^T * motor_tau
    for(int i=0; i<6; i++){
        // Left finger
		joint_tau_out[i] = 0;
	}
    	joint_tau_out[6] = motor_tau_in[6];
}

