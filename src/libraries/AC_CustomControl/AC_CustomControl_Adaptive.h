#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Backend.h"
#include <AC_CustomControl_Adaptive/Custom_Att_Controller.h>

class AC_CustomControl_Adaptive : public AC_CustomControl_Backend
{
public:
    AC_CustomControl_Adaptive(
        AC_CustomControl &frontend,
        AP_AHRS_View*& ahrs,
        AC_AttitudeControl*& att_control,
        AP_MotorsMulticopter*& motors,
        float dt);

    Vector3f update(void) override;
    void reset(void) override;

    Custom_Att_Controller adaptive_controller;
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    float _dt;
    AP_Float lambda_pm, lambda_qm, lambda_rm;
    AP_Float lambda_sr, lambda_sp, lambda_sy;
    AP_Float k1, k2, k3;
    AP_Float P_11, P_22, P_33;
    AP_Float ah_guess_r, ah_guess_p, ah_guess_y;
    AP_Float ah_min_r, ah_min_p, ah_min_y;
    AP_Float ah_max_r, ah_max_p, ah_max_y;
};

#endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED
