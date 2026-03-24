#include "Custom_Att_Controller.h"

void Custom_Att_Controller::Log_CC0(Vector3f U, Vector3f control, Vector3f adapt, Vector3f error) const
{
    struct log_CC0 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC0_MSG),
time_us     : AP_HAL::micros64(),
u_roll      : U.x,
u_pitch     : U.y,
u_yaw       : U.z,
control_r   : control.x,
control_p   : control.y,
control_y   : control.z,
adapt_r     : adapt.x,
adapt_p     : adapt.y,
adapt_y     : adapt.z,
err1        : degrees(error.x),
err2        : degrees(error.y),
err3        : degrees(error.z)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC1(Vector3f wr, Vector3f dwr, Vector3f w_m, Vector3f d_wm) const
{
    struct log_CC1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC1_MSG),
time_us     : AP_HAL::micros64(),
wr1         : degrees(wr.x),
wr2         : degrees(wr.y),
wr3         : degrees(wr.z),
dwr1        : degrees(dwr.x),
dwr2        : degrees(dwr.y),
dwr3        : degrees(dwr.z),
wm1         : degrees(w_m.x),
wm2         : degrees(w_m.y),
wm3         : degrees(w_m.z),
dwm1        : degrees(d_wm.x),
dwm2        : degrees(d_wm.y),
dwm3        : degrees(d_wm.z)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC2(Vector3f w_, Vector3f wd, Vector3f s_) const
{
    struct log_CC2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC2_MSG),
time_us     : AP_HAL::micros64(),
w1          : degrees(w_.x),
w2          : degrees(w_.y),
w3          : degrees(w_.z),
wd1         : degrees(wd.x),
wd2         : degrees(wd.y),
wd3         : degrees(wd.z),
s_roll      : degrees(s_.x),
s_pitch     : degrees(s_.y),
s_yaw       : degrees(s_.z)
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void Custom_Att_Controller::Log_CC3(Vector3f ah, Vector3f dah, Vector3f ys_) const
{
    struct log_CC3 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CC3_MSG),
time_us     : AP_HAL::micros64(),
ah1         : ah.x,
ah2         : ah.y,
ah3         : ah.z,
dah1        : dah.x,
dah2        : dah.y,
dah3        : dah.z,
ys1         : ys_.x,
ys2         : ys_.y,
ys3         : ys_.z
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

float Custom_Att_Controller::param_projection(float ahat, float dahat, float ahat_min, float ahat_max)
{
    if (ahat >= ahat_max && dahat > 0.0F) {
        return 0.0F;
    } else if (ahat <= ahat_min && dahat < 0.0F) {
        return 0.0F;
    } else {
        return dahat;
    }
}

void Custom_Att_Controller::step(
    Vector3f w_d, Vector3f w, Vector3f &U, Vector3f att_error, float dt, Vector3f ah_min,
    Vector3f ah_max, Vector3f lambdas_sliding, Vector3f kd_gains, Vector3f p_gains)
{
    Y.zero();

    // Create acceleration raw ref based on filtered speed reference
    Vector3f raw_dw_d;

    if (dt > 0.0F) {
        raw_dw_d = (w_d - wd_prev) / dt;
    } else {
        raw_dw_d.zero();
    }

    // Update previous speed reference
    wd_prev = w_d;

    // Generate filtered acceleration reference
    Vector3f dw_d = target_accel.apply(raw_dw_d, dt);

    // Virtual controller reference, includes velocity and attitude targets
    // xr = x_d + (lambda * error);
    Vector3f ls_att = att_error;
    ls_att *= lambdas_sliding;
    w_r = w_d + ls_att;

    // Desired - Actual to match ArduPilot's logic
    s = w_r - w;

    // Derivate of the virtual controller reference
    // dxr = dx_d + (lambda * d_error);
    Vector3f ls_diff = w_d - w;
    ls_diff *= lambdas_sliding;
    dw_r = dw_d + ls_diff;

    // Populate Y matrix
    Y.a.x = dw_r.x;
    Y.a.y = -(w.y * w.z);
    Y.a.z = w.y * w.z;

    Y.b.x = w.x * w.z;
    Y.b.y = dw_r.y;
    Y.b.z = -(w.x * w.z);

    Y.c.x = -(w.x * w.y);
    Y.c.y = w.x * w.y;
    Y.c.z = dw_r.z;

    // Adaptation contribution
    adaptation = Y * a_hat;

    // Controller contribution Kd * s
    controller = s;
    controller *= kd_gains;

    // Control output
    U = adaptation + controller;

    // Adaptation law
    // da_hat = P*Y(t)*s
    ys = Y.transposed() * s;
    da_hat = ys;
    da_hat *= p_gains;

    // Apply param projection to stop adaptation if not necessary
    da_hat.x = param_projection(a_hat.x, da_hat.x, ah_min.x, ah_max.x);
    da_hat.y = param_projection(a_hat.y, da_hat.y, ah_min.y, ah_max.y);
    da_hat.z = param_projection(a_hat.z, da_hat.z, ah_min.z, ah_max.z);

    // Update adaptation
    a_hat += da_hat * dt;
    a_hat.x = constrain_float(a_hat.x, ah_min.x, ah_max.x);
    a_hat.y = constrain_float(a_hat.y, ah_min.y, ah_max.y);
    a_hat.z = constrain_float(a_hat.z, ah_min.z, ah_max.z);

    // Log debug variables
    Log_CC0(U, controller, adaptation, att_error);
    Log_CC1(w_r, dw_r, w_r, dw_r);
    Log_CC2(w, w_d, s);
    Log_CC3(a_hat, da_hat, ys);
}

void Custom_Att_Controller::initialize(Vector3f guesses)
{
    w_r.zero();
    dw_r.zero();
    s.zero();
    Y.zero();
    wd_prev.zero();
    target_accel.set_cutoff_frequency(10.0F);
    a_hat.x = guesses.x;
    a_hat.y = guesses.y;
    a_hat.z = guesses.z;
    da_hat.zero();
    controller.zero();
    adaptation.zero();
}

// Constructor
Custom_Att_Controller::Custom_Att_Controller() {}

Custom_Att_Controller::~Custom_Att_Controller() = default;