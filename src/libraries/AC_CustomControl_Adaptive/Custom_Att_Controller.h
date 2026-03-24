#pragma once
#include <cmath>
#include <AP_Logger/AP_Logger.h>
#include <Filter/LowPassFilter.h>
// Class declaration for model Custom_Att_Controller
class Custom_Att_Controller final
{
    // public data and function members
public:
    // Copy Constructor
    Custom_Att_Controller(Custom_Att_Controller const&) = delete;

    // Assignment Operator
    Custom_Att_Controller& operator= (Custom_Att_Controller const&) & = delete;

    // Move Constructor
    Custom_Att_Controller(Custom_Att_Controller &&) = delete;

    // Move Assignment Operator
    Custom_Att_Controller& operator= (Custom_Att_Controller &&) = delete;

    // Controller initialization
    void initialize(Vector3f guesses);

    // Controller step function
    void step(Vector3f w_d, Vector3f w, Vector3f &U, Vector3f att_error, float dt, Vector3f ah_min,
              Vector3f ah_max, Vector3f lambdas_sliding, Vector3f kd_gains, Vector3f p_gains);

    void Log_CC0(Vector3f U, Vector3f controller, Vector3f adaptation, Vector3f att_error) const;

    void Log_CC1(Vector3f wr, Vector3f d_wr, Vector3f wm, Vector3f dwm) const;

    void Log_CC2(Vector3f w, Vector3f wd, Vector3f s) const;

    void Log_CC3(Vector3f ah, Vector3f dah, Vector3f ys) const;

    float param_projection(float a_hat, float da_hat, float ahat_min, float ahat_max);

    // Constructor
    Custom_Att_Controller();

    // Destructor
    ~Custom_Att_Controller();

    // private data and function members
private:
    Vector3f ys, a_hat, da_hat;
    Vector3f w_r, dw_r;
    Vector3f s;
    Vector3f controller, adaptation;
    Vector3f wd_prev;
    LowPassFilterVector3f target_accel;
    Matrix3f Y;
};