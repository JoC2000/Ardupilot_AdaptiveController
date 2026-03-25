#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Adaptive.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_Adaptive::var_info[] = {
    // @Param: AH_MIN_R
    // @DisplayName: AH_MIN_R
    // @Description: Min inertial estimation for roll angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MIN_R", 1, AC_CustomControl_Adaptive, ah_min_r, 0.01F),

    // @Param: AH_MIN_P
    // @DisplayName: AH_MIN_P
    // @Description: Min inertial estimation for pitch angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MIN_P", 2, AC_CustomControl_Adaptive, ah_min_p, 0.01F),

    // @Param: AH_MIN_Y
    // @DisplayName: AH_MIN_Y
    // @Description: Min inertial estimation for yaw angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MIN_Y", 3, AC_CustomControl_Adaptive, ah_min_y, 0.01F),

    // @Param: AH_MAX_R
    // @DisplayName: AH_MAX_R
    // @Description: Max inertial estimation for roll angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MAX_R", 4, AC_CustomControl_Adaptive, ah_max_r, 0.1F),

    // @Param: AH_MAX_P
    // @DisplayName: AH_MAX_P
    // @Description: Max inertial estimation for pitch angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MAX_P", 5, AC_CustomControl_Adaptive, ah_max_p, 0.1F),

    // @Param: AH_MAX_Y
    // @DisplayName: AH_MAX_Y
    // @Description: Max inertial estimation for yaw angle, must be positive
    // @User: Advanced
    AP_GROUPINFO("AH_MAX_Y", 6, AC_CustomControl_Adaptive, ah_max_y, 0.1F),

    // @Param: L_SLIDING_R
    // @DisplayName: L_SLIDING_R
    // @Description: Lambda gain for x rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("L_SLIDING_R", 7, AC_CustomControl_Adaptive, lambda_sr, 0.87F),

    // @Param: L_SLIDING_P
    // @DisplayName: L_SLIDING_P
    // @Description: Lambda gain for y rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("L_SLIDING_P", 8, AC_CustomControl_Adaptive, lambda_sp, 0.73F),

    // @Param: L_SLIDING_Y
    // @DisplayName: L_SLIDING_Y
    // @Description: Lambda gain for z rate sliding surface
    // @User: Advanced
    AP_GROUPINFO("L_SLIDING_Y", 9, AC_CustomControl_Adaptive, lambda_sy, 0.45F),

    // @Param: K_ROLL
    // @DisplayName: K_ROLL
    // @Description: K gain for Roll controller
    // @User: Advanced
    AP_GROUPINFO("K_ROLL", 10, AC_CustomControl_Adaptive, k1, 0.045F),

    // @Param: K_PITCH
    // @DisplayName: K_PITCH
    // @Description: K gain for Pitch controller
    // @User: Advanced
    AP_GROUPINFO("K_PITCH", 11, AC_CustomControl_Adaptive, k2, 0.045F),

    // @Param: K_YAW
    // @DisplayName: K_YAW
    // @Description: K gain for Yaw controller
    // @User: Advanced
    AP_GROUPINFO("K_YAW", 12, AC_CustomControl_Adaptive, k3, 0.023F),

    // @Param: P_ROLL
    // @DisplayName: P_ROLL
    // @Description: P roll adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_ROLL", 13, AC_CustomControl_Adaptive, P_11, 0.08F),

    // @Param: P_PITCH
    // @DisplayName: P_PITCH
    // @Description: P pitch adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_PITCH", 14, AC_CustomControl_Adaptive, P_22, 0.08F),

    // @Param: P_YAW
    // @DisplayName: P_YAW
    // @Description: P yaw adaptive gain
    // @User: Advanced
    AP_GROUPINFO("P_YAW", 15, AC_CustomControl_Adaptive, P_33, 0.05F),

    // @Param: GUESS_R
    // @DisplayName: GUESS_R
    // @Description: Initial guess for roll intertia
    // @User: Advanced
    AP_GROUPINFO("GUESS_R", 16, AC_CustomControl_Adaptive, ah_guess_r, 0.03F),

    // @Param: GUESS_P
    // @DisplayName: GUESS_P
    // @Description: Initial guess for pitch intertia
    // @User: Advanced
    AP_GROUPINFO("GUESS_P", 17, AC_CustomControl_Adaptive, ah_guess_p, 0.02F),

    // @Param: GUESS_Y
    // @DisplayName: GUESS_Y
    // @Description: Initial guess for yaw intertia
    // @User: Advanced
    AP_GROUPINFO("GUESS_Y", 18, AC_CustomControl_Adaptive, ah_guess_y, 0.01F),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_Adaptive::AC_CustomControl_Adaptive(
    AC_CustomControl &frontend,
    AP_AHRS_View*& ahrs,
    AC_AttitudeControl*& att_control,
    AP_MotorsMulticopter*& motors,
    float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);

    Vector3f guesses{ah_guess_r.get(), ah_guess_p.get(), ah_guess_y.get()};
    adaptive_controller.initialize(guesses);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_Adaptive::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // We are still at the ground. Reset custom controller to avoid
        // build up, ex: integrator
        reset();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // we are off the ground
        break;
    }

    // run custom controller after here
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);

    attitude_target = _att_control->get_attitude_target_quat();


    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error,
            _thrust_angle, _thrust_error_angle);

    Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    Vector3f target_rate;
    target_rate[0] = ang_vel_body_feedforward[0];
    target_rate[1] = ang_vel_body_feedforward[1];
    target_rate[2] = ang_vel_body_feedforward[2];

    Vector3f ah_min{ah_min_r.get(), ah_min_p.get(), ah_min_y.get()};
    Vector3f ah_max{ah_max_r.get(), ah_max_p.get(), ah_max_y.get()};
    Vector3f lambdas_sliding{lambda_sr.get(), lambda_sp.get(), lambda_sy.get()};
    Vector3f kd_gains{k1.get(), k2.get(), k3.get()};
    Vector3f p_gains{P_11.get(), P_22.get(), P_33.get()};
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;

    adaptive_controller.step(target_rate, gyro_latest, motor_out, attitude_error, _dt, ah_min, ah_max, lambdas_sliding, kd_gains, p_gains);

    // return what arducopter main controller outputted
    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_Adaptive::reset(void)
{
}

#endif  // AP_CUSTOMCONTROL_Adaptive_ENABLED
