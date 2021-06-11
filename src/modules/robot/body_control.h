#ifndef __BODY_CONTROL__
#define __BODY_CONTROL__

#include <string>

namespace bz_robot
{
class BodyControl
{
public:
    static std::string server_url;
public:
    static bool set_control_mode(int mode);
    static int get_control_mode();
    static bool e_stop(int mode = 1);
    static bool stop(int mode = 1);
    static bool begin(int mode = 1);
    static bool is_ok();
    static bool set_velocity_and_angle(float linear_velocity, float angle, int mode = 1);
    static bool set_linear_velocity(float linear_velocity, int mode = 1);
    static bool set_angle(float angle, int mode = 1);
    static bool reset_error_state(int mode = 1);
    static bool reset_warning_state(int mode = 1);
    static bool set_safe_mode_on(int mode = 1);
    static bool set_safe_mode_off(int mode = 1);
    static void exit(int mode = 1);
    static int feedback_velocity();
    static int feedback_angle();
    static int left_wheel_encoder();
    static int right_wheel_encoder();

};
}

#endif
