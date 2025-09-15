/*
 * kinematics_solver.c
 *
 *  Created on: Sep 9, 2025
 *      Author: khoi
 */

#include "motor/kinematics_solver.h"

#define PI 3.14159265358979323846f

extern motor_t *motors[]; //io gain access to motor variables in interrupt service routine

//global variables
// Current joint angles (deg) after homing
float q1_curr_g = Q1_CURR;
float q2_curr_g = Q2_CURR;
float q3_curr_g = Q3_CURR;
float q4_curr_g = Q4_CURR;


float wrap180(float a) {
    while (a >  180.0) a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}


uint8_t map_into_range(float *a, float amin, float amax) {
    for (int k = -2; k <= 2; ++k) {
        float cand = *a + 360.0 * k;
        if (cand >= amin - 1e-9 && cand <= amax + 1e-9) {
            *a = cand;   // update angle to the feasible equivalent
            return 1;    // success
        }
    }
    return 0; // no equivalent angle fits into the allowed range
}


int ik_2r_deg(float xtcp, float ytcp, float l1, float l2, motor_angles out[2]) {
    out[0] = (motor_angles){0,0,0};
    out[1] = (motor_angles){0,0,0};

    float r2 = xtcp*xtcp + ytcp*ytcp;
    float r  = sqrt(r2);

    if (r > l1 + l2 + 1e-9 || r < fabs(l1 - l2) - 1e-9) return 0; //The arm reaches its maximum length when stretched or
    																			//if the TCP is too close to the base origin, this will not work either.

    float c = (r2 - l1*l1 - l2*l2) / (2.0*l1*l2); 	// c = cos(q2)
    if (c >  1.0) c = 1.0;	// -1 < cosine < 1, but since c ist float, there might be Rounding errors -> c may be slightly outside threshold
    if (c < -1.0) c = -1.0;

    // q2 candidates (rad)
    float q2a_r =  acos(c);
    float q2b_r = -q2a_r;

    // q1 for each (rad)
    float q1a_r = atan2(ytcp, xtcp) - atan2(l2*sin(q2a_r), l1 + l2*cos(q2a_r));
    float q1b_r = atan2(ytcp, xtcp) - atan2(l2*sin(q2b_r), l1 + l2*cos(q2b_r));

    float s = 180.0/PI; //Conversion from radians to degrees.
    float q1a = wrap180(q1a_r * s), q2a = wrap180(q2a_r * s);
    float q1b = wrap180(q1b_r * s), q2b = wrap180(q2b_r * s);

    // Special case: "singular duplicate"
    // In general, a 2R arm has two IK solutions (elbow-up and elbow-down).
    // But in singular configurations these two solutions collapse into one:
    //   - Arm fully stretched  -> q2 = 0°
    //   - Arm fully folded     -> q2 = 180° or -180°
    // In those cases, (q1a,q2a) and (q1b,q2b) become numerically identical.
    // To avoid returning the same solution twice, we detect this by checking
    // if both angle pairs differ less than a tiny epsilon (1e-7).
    // If yes, we keep only one solution in out[0] and return 1.
    if (fabs(q1a - q1b) < 1e-7 && fabs(q2a - q2b) < 1e-7) {
        out[0] = (motor_angles){ q1a, q2a, 1 };
        return 1;
    }
    out[0] = (motor_angles){ q1a, q2a, 1 }; // elbow-down usually
    out[1] = (motor_angles){ q1b, q2b, 1 }; // elbow-up usually
    return 2;
}


motion_plan_1R motion_plan_in_xy(float xt, float yt,
                              float q_curr_deg,
                              float q_min, float q_max)
{
    motion_plan_1R mp = {0.0, 0};

    // 1) rotation axis not defined if x=0,y=0 -> no valid direction
    if (fabs(xt) < 1e-12 && fabs(yt) < 1e-12) {
        return mp; // invalid
    }

    // 2) Desired absolute joint angle from world X-axis toward (xt,yt)
    float q_des_rad = atan2(yt, xt);         // atan2 return value in range [-pi, pi] (all four quadrants considered)
    float q_des_deg = q_des_rad * 180.0 / PI;
    q_des_deg = wrap180(q_des_deg);           // (-180,180]

    // 3) Try to place desired angle inside mechanical limits via ±360° shifts
    if (!map_into_range(&q_des_deg, q_min, q_max)) {
        return mp; // cannot realize inside limits
    }

    // 4) Compute relative motion from current pose
    mp.dq_deg = q_des_deg - q_curr_deg;       // positive -> left, negative -> right (as per your moveDegree convention)
    mp.valid  = 1;
    return mp;
}


motion_plan_3R motion_plan_in_xz(float xtcp, float ztcp, float end_effector_angle,
                          float l1, float l2, float l3,
                          float q1_curr, float q2_curr, float q3_curr,
                          float q1_min, float q1_max,
                          float q2_min, float q2_max,
                          float q3_min, float q3_max)
{
    motion_plan_3R mp = {0,0,0,0};    // default: invalid result
    motor_angles sols[2];

    float x_joint2 = xtcp - l3 * cos(end_effector_angle);
    float y_joint2 = ztcp - l3 * sin(end_effector_angle);

    int n = ik_2r_deg(x_joint2, y_joint2, l1, l2, sols);
    if (n == 0) return mp;     // No IK solution at all (target unreachable)

    float best_cost = FLT_MAX; // Max float value = "infinite" cost initially
    float best_q1 = 0, best_q2 = 0, best_q3 = 0;

    // Iterate over all candidate IK solutions
    for (int i = 0; i < n; ++i) {
        if (!sols[i].valid) continue; // skip invalid

        float q1 = sols[i].q1_deg;
        float q2 = sols[i].q2_deg;

        // Try to shift each joint angle by ±360° into the allowed band [q_min, q_max].
        // Example: IK gives -190°, but joint allows [-170, 170] -> shift +360° -> +170° is valid.
        if (!map_into_range(&q1, q1_min, q1_max)) continue;
        if (!map_into_range(&q2, q2_min, q2_max)) continue;

        // q3 = thetaE_deg - (q1 + q2); // everything in degrees!
        // end_effector_angle is in RAD (for cos/sin), so first convert to degrees:
        float thetaE_deg = end_effector_angle * 180.0f / PI;
        float q3 = wrap180(thetaE_deg - (q1 + q2));

        // Check the joint against its mechanical limits (±360° mapping)
        if (!map_into_range(&q3, q3_min, q3_max)) continue;

        // Compute travel distance from current pose:
        //   dq1 = how much joint 1 must move,
        //   dq2 = how much joint 2 must move.
        // Cost = total travel = |dq1| + |dq2|.
        // Example:
        //   Current: (-30,-20), Candidate A: (-40,-25) -> dq=(-10,-5), cost=15
        //   Candidate B: (-150,-10) -> dq=(-120,+10), cost=130
        //   -> Candidate A is chosen (smaller cost).
        float dq1 = q1 - q1_curr;
        float dq2 = q2 - q2_curr;
        float dq3 = q3 - q3_curr;
        float cost = fabsf(dq1) + fabsf(dq2) + fabsf(dq3);

        // Keep the solution with the smallest cost so far
        if (cost < best_cost) {
            best_cost = cost;
            best_q1 = q1;
            best_q2 = q2;
            best_q3 = q3;   
        }
    }

    // If no solution inside the mechanical limits was found, abort.
    // Example: target is geometrically reachable but requires q1>0°,
    // while limit is [q1_min=-170°, q1_max=0°]. Then best_cost never updates.
    if (best_cost == FLT_MAX) return mp;

    // Compute final delta commands relative to current pose.
    // These can be sent directly to your moveDegree() function.
    mp.dq1_deg = best_q1 - q1_curr;
    mp.dq2_deg = best_q2 - q2_curr;
    mp.dq3_deg = best_q3 - q3_curr;
    mp.valid = 1;
    return mp;
}


motion_plan_4R motion_plan_in_xyz(float xtcp, float ytcp, float ztcp, float end_effector_angle,
                          float l1, float l2, float l3, float l4,
                          float q1_curr, float q2_curr, float q3_curr, float q4_curr,
                          float q1_min, float q1_max,
                          float q2_min, float q2_max,
                          float q3_min, float q3_max,
                          float q4_min, float q4_max)
{
    motion_plan_4R mp;

    mp.dq1_deg = motion_plan_in_xy(xtcp, ytcp, q1_curr, q1_min, q1_max);
    
    ztcp = ztcp - l1;   // Since coordinate system from Motor 2 is l1 [mm] higher than world frame, z_new = ztcp - l1
    end_effector_angle = end_effector_angle * PI /180.0f;
    float q1_deg = (q1_curr + mp.dq1_deg.dq_deg) * PI / 180.0f; //target q1_degree
    if (cos(q1_deg) != 0)
        xtcp = xtcp / cos(q1_deg);   //x_new after Motor 1 has rotated
    else 
        xtcp = 0;

    mp.dq2_dq3_dq4_deg = motion_plan_in_xz(xtcp,ztcp,end_effector_angle,l2,l3,l4,q2_curr,q3_curr,q4_curr,q2_min,q2_max,q3_min,q3_max,q4_min,q4_max);

    return mp;
}

/*INPUTS: Target TCP position and orientation(mm)
 *OUTPUT: 0 if target point is not reachable (No inverse kinematics solution)
 *OUTPUT: 1 if motion possible
 */
uint8_t move_to_Coordinate(float xtcp, float ytcp, float ztcp, float end_effector_angle)
{
    // Link lengths (mm)
    float l1 = LENGTH_SEGMENT_1;
    float l2 = LENGTH_SEGMENT_2;
    float l3 = LENGTH_SEGMENT_3;
    float l4 = LENGTH_SEGMENT_4;

    // Mechanical limits/stops (deg): q_min <= q <= q_max
    float q1_min = Q1_MIN, q1_max = Q1_MAX;
    float q2_min = Q2_MIN, q2_max = Q2_MAX;
    float q3_min = Q3_MIN, q3_max = Q3_MAX;
    float q4_min = Q4_MIN, q4_max = Q4_MAX;

    motion_plan_4R mp = motion_plan_in_xyz(xtcp,ytcp,ztcp,end_effector_angle,
                                        l1,l2,l3,l4,
                                        q1_curr_g,q2_curr_g,q3_curr_g,q4_curr_g,
                                        q1_min,q1_max,q2_min,q2_max,q3_min,q3_max,q4_min,q4_max);

    if(mp.dq1_deg.valid && mp.dq2_dq3_dq4_deg.valid)
    {
    		writeDisplay("Motion possible!\n");
    		HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_SET);

        moveDegrees(mp.dq2_dq3_dq4_deg.dq3_deg, motors[3]); 	//move motor 4 first in order not to collide with object, which will be grasped.
        HAL_Delay(1000);																		// Just a temporary solution. No need to delay if path planning is implemented.

        moveDegrees(mp.dq1_deg.dq_deg, motors[0]);
        moveDegrees(mp.dq2_dq3_dq4_deg.dq1_deg, motors[1]);
        moveDegrees(mp.dq2_dq3_dq4_deg.dq2_deg, motors[2]);

        // StallGuard
      	while(motors[0] -> active_movement_flag ||
							motors[1] -> active_movement_flag ||
							motors[2] -> active_movement_flag ||
							motors[3] -> active_movement_flag )
      	{
      		checkDriverStatus(motors[0]);
      		checkDriverStatus(motors[1]);
      		checkDriverStatus(motors[2]);
      		checkDriverStatus(motors[3]);
      	}

        // Update current joint angles
        q1_curr_g = q1_curr_g + mp.dq1_deg.dq_deg;
        q2_curr_g = q2_curr_g + mp.dq2_dq3_dq4_deg.dq1_deg;
        q3_curr_g = q3_curr_g + mp.dq2_dq3_dq4_deg.dq2_deg;
        q4_curr_g = q4_curr_g + mp.dq2_dq3_dq4_deg.dq3_deg;

    		HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_RESET);
        return 1;
    }
    else{
  		writeDisplay("Target unreachable! Choose new one"); //Inverse kinematics failed
  		HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
  		return 0;
    }
}
