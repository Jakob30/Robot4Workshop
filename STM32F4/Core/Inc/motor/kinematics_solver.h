/*
 * kinematics_solver.h
 *
 *  Created on: Sep 9, 2025
 *      Author: khoi
 */

#ifndef INC_MOTOR_KINEMATICS_SOLVER_H_
#define INC_MOTOR_KINEMATICS_SOLVER_H_

#include <stdint.h>
#include <math.h>
#include <float.h>

#include "motor/motor_init.h"

//Define Roboter's physical dimensions
#define LENGTH_SEGMENT_1 210.0f
#define LENGTH_SEGMENT_2 160.0f
#define LENGTH_SEGMENT_3 160.0f
#define LENGTH_SEGMENT_4 90.0f //length from joint (motor 4) to the tip of the threaded guide spindle (inside the gripper).

/* =============================================================================
   Coordinate Systems & Conventions (for readability and maintenance)
   ============================================================================

   WORLD FRAME (right-handed):
     - X_world : forward
     - Y_world : left
     - Z_world : up

   DECOMPOSITION (3D -> two planar subproblems):
     - XY-plane motion  : handled by Motor 1 (base yaw).
     - XZ-plane motion  : handled by Motors 2–3–4 as a planar 3R (3 Revolute Joints) arm.

   GENERAL ANGLE CONVENTIONS
     - All angles are in degrees.
     - Positive rotation = counterclockwise (CCW) in the respective plane,
       i.e., from local +x toward local +y (right-hand rule).
     - moveDegree(Joint, +Δ) : rotate CCW / to the "left".
       moveDegree(Joint, −Δ) : rotate CW  / to the "right".

   ---------------------------------------------------------------------------
   1) XY-PLANE  (Motor 1 only)
      - Motor-1 local frame = WORLD XY frame:
          x_1 ≡ X_world  (forward)
          y_1 ≡ Y_world  (left)
      - Homing: drives to the LEFT mechanical stop.
      - Example numbers (measured):
          • Left stop (Homing position)  : q1 = −173.57°
          • Right stop (if any)          : (set your measured value)

   ---------------------------------------------------------------------------
   2) XZ-PLANE  (Motors 2–3–4 act as a planar 3R)
      - We use the standard 3R planar IK conventions from:
        https://expred.co/kinematics-of-planar-robots-with-revolute-joints/ and
        https://youtu.be/NjAAKruKiQM?si=46BqkbxPjFhr0zLm
        In that derivation the first joint of the 2D chain uses:
          x → to the right, y → up  (in that plane).

      • Motor 2 (shoulder in XZ-plane)
          - Local frame:
              x_2  ≡  +X_right_in_that_plane  ≡  +Y_world
              y_2  ≡  +Y_up_in_that_plane     ≡  +Z_world
            (i.e., the XZ-plane’s +x is aligned with WORLD Y, and +y with WORLD Z)
          - Limits (example):
              upper stop  : +173.57°
              lower stop  : −173.57°
          - Homing position: q2 = +173.57°

      • Motor 3 (elbow in XZ-plane)
          - Link-frames rule for planar 3R:
              x-axis is always aligned with the current rigid link (segment),
              y-axis is rotated by +90° from the link’s +x (within the plane).
            E.g., in our Robot, Motor 3 control segment 3 ⇒ x-axis Motor 3 is aligned with segment 2 (both lie on a straight line)
          - At Homing: Motor 3 forms a straight line with Motor 2 (links colinear).
              ⇒ x_3  ≡  Z_world (up)
              ⇒ y_3  ≡ −Y_world (opposite WORLD Y)
          - Limits (example):
              upper stop  :   0.00°   (straight w.r.t. Motor 2)
              lower stop  : −138.77°
          - Homing position: q3 = 0.00°

      • Motor 4 (wrist / end-effector in XZ-plane)
          - After Homing the 3rd link points UP (along Z_world).
              ⇒ x_4  ≡  Z_world (up)
              ⇒ y_4  ≡ −Y_world (opposite WORLD Y)
          - Limits (example):
              upper stop  : +90.00°
              lower stop  : −90.00°
          - Homing position: q4 = +90.00°

   ---------------------------------------------------------------------------
   SUMMARY TABLE (angles in degrees)

     Joint   Plane   Local x-axis                 Local y-axis                 Homing      Limits
     -----   -----   ---------------------------  ---------------------------  ----------  -------------------------
     M1      XY      X_world                      Y_world                      −173.57°    [-173.57°, +173.57°]
     M2      XZ      +Y_world                     +Z_world                     +173.57°    [-26.92°, 90.00°]
     M3      XZ      Z_world (colinear w/ M2)     −Y_world                     0.00°       [−138.77°, 0.00°]
     M4      XZ      Z_world (tool direction)     −Y_world                     +90.00°     [−90.00°, +90.00°]

   ============================================================================ */

/*Current joint angles (deg) after homing*/
#define Q1_CURR 173.57f
#define Q2_CURR 90.0f
#define Q3_CURR 0.0f
#define Q4_CURR 90.0f

/*Mechanical limits/stops (deg): q_min <= q <= q_max*/
#define Q1_MIN -173.57f
#define Q1_MAX 173.57f
#define Q2_MIN -26.92f
#define Q2_MAX 90.0f
#define Q3_MIN -138.77f
#define Q3_MAX 0.0f
#define Q4_MIN -90.0f
#define Q4_MAX 90.0f

typedef struct {
    float q1_deg;
    float q2_deg;
    uint8_t valid; // 1 valid, 0 invalid
} motor_angles;

typedef struct {
    float dq_deg;   // relative rotation degree (delta) for command: moveDegrees(float degrees, motor_t* motor)
    uint8_t valid;   // 1 = feasible, 0 = infeasible
} motion_plan_1R;   //1 Revolute (joint)

/*
Select the feasible IK (inverse kinematic) solution within per-joint limits and compute deltas from current pose.
Limits (Stops) are absolute mechanical limits in degrees, e.g. q_max = 0 (left stop), q_min -180 (right stop).
*/
typedef struct {
    float dq1_deg; // delta for moveDegree(dq1_deg, &motor1)
    float dq2_deg; // delta for moveDegree(dq2_deg, &motor2)
    float dq3_deg;  // delta for moveDegree(dq3_deg, &motor3)
    uint8_t valid;  // 1 if a feasible solution exists
} motion_plan_3R; //3 Revolute (joints)

typedef struct {
    motion_plan_1R dq1_deg; // delta for moveDegree(Motor1, dq1_deg)
    motion_plan_3R dq2_dq3_dq4_deg;
} motion_plan_4R;

/*
 * Function Declaration
 */

// -----------------------------------------------------------------------------
// float wrap180(float a)
// Purpose: Why do we normalize angles to the range (-180°, 180°] ?
//
// Trigonometric functions (atan2, acos, etc.) can produce mathematically
// correct angles, but they are not unique. For example, a rotation of +370°,
// -350° and +10° all describe the exact same direction.
//
// Without normalization, you might see "weird" numbers like +370° or -190°
// in your inverse kinematics results. If you then pass those directly to the
// motor control, the robot could try to make a full turn instead of a short move.
//
// Normalizing ensures that *every orientation has exactly one unique
// representation in a compact interval. The choice (-180°, 180°] is common
// because it is symmetric around 0 and makes it easy to compute the "shortest
// path" rotation between two angles.
//
// Examples:
//   Input angle =  370°  -> normalized =  +10°
//   Input angle =  -190° -> normalized =  +170°
//   Input angle =  -540° -> normalized =  -180°
//
// With this normalization, your controller always works with small,
// predictable numbers and the motion planner can choose the shortest rotation.
//
// If your mechanics or motors prefer another convention (e.g. [0,360)),
// you can simply adapt this function accordingly.
// -----------------------------------------------------------------------------
float wrap180(float a);


// -----------------------------------------------------------------------------
// map_into_range()
// Purpose:
//   Angles are periodic: -190°, +170°, +530° all describe the same orientation.
//   However, your robot joints are physically limited to a certain window
//   [amin, amax] (e.g., [-170°, 0°]).
//
//   This function tries to "shift" the mathematical IK solution by adding or
//   subtracting whole turns (±360°) until the equivalent angle falls inside the
//   allowed window.
//
// How it works:
//   - Start with the candidate angle *a (from IK).
//   - Test shifted versions: a-720, a-360, a, a+360, a+720.
//   - If any shifted version falls inside [amin, amax], accept it and update *a.
//   - If no equivalent version is valid, return 0 (invalid).
//
// Why ±2 turns?
//   In robotics, the allowed range is always <360° (a joint cannot spin forever).
//   Therefore testing a small number of shifts is enough to cover all cases.
//
// Example 1:
//   Joint limit: [0°, 180°]
//   IK solution: -190°
//   Candidates: -190, +170, +530 ...
//   +170° is inside [0,180] -> valid, so *a becomes 170°.
//
// Example 2:
//   Joint limit: [-170°, 0°]
//   IK solution: -190°
//   Candidates: -190, +170, +530 ...
//   None of them fall inside [-170,0] -> return 0 (solution discarded).
//
// Return:
//   1 if a valid mapping was found (and *a updated),
//   0 if no equivalent angle lies inside [amin, amax].
// -----------------------------------------------------------------------------
uint8_t map_into_range(float *a, float amin, float amax);


// 2R (revolute joints) IK (inverse kinematic) → up to 2 solutions (in degrees, wrapped to (-180,180])
int ik_2r_deg(float xtcp, float ytcp, float l1, float l2, motor_angles out[2]);


// -----------------------------------------------------------------------------
// motion_plan_in_xy()
// Computes a feasible motion plan to reach a target point (xt, yt)
// for 1 Motor, respecting current joint pose and mechanical limits.
//
// INPUTS
//   xt, yt       : x target, y target
//   q_curr_deg   : current joint angle [deg] (since homing)
//   q_min, q_max : allowed range for joint [deg]
// OUTPUT
//   motion_plan_1R mp : contains delta angles to send with moveDegree(),
//                  and a valid flag (1 = feasible, 0 = infeasible)
motion_plan_1R motion_plan_in_xy(float xt, float yt,
                              float q_curr_deg,
                              float q_min, float q_max);


// -----------------------------------------------------------------------------
// motion_plan_in_xz()
// Computes a feasible motion plan to reach a target TCP (xtcp, ztcp)
// for a R planar robot arm, respecting current joint pose and mechanical limits.
//
// INPUTS
//   xtcp, ztcp   : target TCP position
//   l1, l2, l3       : link lengths
//   q1_curr      : current joint 1 angle [deg] (since homing)
//   q2_curr      : current joint 2 angle [deg] (since homing)
//   q3_curr      : current joint 3 angle [deg] (since homing)
//   q1_min, q1_max : allowed range for joint 1 [deg]
//   q2_min, q2_max : allowed range for joint 2 [deg]
//   q3_min, q3_max : allowed range for joint 3 [deg]
//
// OUTPUT
//   motion_plan_3R mp : contains delta angles (dq1, dq2, dq3) to send with moveDegree(),
//                  and a valid flag (1 = feasible, 0 = infeasible)
//
// LOGIC
//   1. Compute all IK solutions (up to 2).
//   2. Discard solutions outside mechanical limits.
//   3. Among feasible ones, choose the one with the *shortest travel*
//      from the current pose (minimum cost).
//   4. Return deltas (dq = target - current) so they can be executed
//      as relative movements.
// -----------------------------------------------------------------------------
motion_plan_3R motion_plan_in_xz(float xtcp, float ztcp, float end_effector_angle,
                          float l1, float l2, float l3,
                          float q1_curr, float q2_curr, float q3_curr,
                          float q1_min, float q1_max,
                          float q2_min, float q2_max,
                          float q3_min, float q3_max);


// 3D motion by calling motion_plan_in_xy() and motion_plan_in_xz()
motion_plan_4R motion_plan_in_xyz(float xtcp, float ytcp, float ztcp, float end_effector_angle,
                          float l1, float l2, float l3, float l4,
                          float q1_curr, float q2_curr, float q3_curr, float q4_curr,
                          float q1_min, float q1_max,
                          float q2_min, float q2_max,
                          float q3_min, float q3_max,
                          float q4_min, float q4_max);


/*INPUTS: Target TCP position and orientation(mm)
 *OUTPUT: 0 if target point is not reachable (No inverse kinematics solution)
 *OUTPUT: 1 if motion possible
 */
uint8_t move_to_Coordinate(float xtcp, float ytcp, float ztcp, float end_effector_angle);


#endif /* INC_MOTOR_KINEMATICS_SOLVER_H_ */


// -----------------------------------------------------------------------------
// Difference between wrap180() and map_into_range():
//
// wrap180():
//   - Normalizes any angle into the canonical range (-180°, 180°].
//   - Purpose: make sure we don’t carry around values like +370° or -350°.
//   - Example:
//       Input: -190°  -> wrap180 -> +170°
//       Input: +370°  -> wrap180 -> +10°
//   - Result is always a clean, unique representation, independent of joint limits.
//
// map_into_range():
//   - Checks if an angle (plus any ±360° equivalent) can be placed
//     inside the allowed mechanical limits [amin, amax].
//   - If possible, shifts it by ±360° and accepts it, otherwise rejects.
//   - Example 1: Limits [0°,180°], IK gives -190°.
//       Candidates: -190, +170, +530 ...
//       +170° fits in [0,180] -> valid solution.
//   - Example 2: Limits [-170°,0°], IK gives -190°.
//       Candidates: -190, +170, +530 ...
//       None fits in [-170,0] -> solution invalid.
//   - Result depends on the actual physical joint constraints.
//
// In practice:
//   - First call wrap180() after converting radians->degrees (for consistency).
//   - Later call map_into_range() to decide if the solution is usable
//     within the robot’s mechanical range.
// -----------------------------------------------------------------------------
