#include "Motion_control.h"

AS5600_soft_IIC_many MC_AS5600;
uint32_t AS5600_SCL[] = {PB15, PB14, PB13, PB12};
uint32_t AS5600_SDA[] = {PD0, PC15, PC14, PC13};
#define AS5600_PI 3.1415926535897932384626433832795
#define speed_filter_k 100
float speed_as5600[4] = {0, 0, 0, 0};

void MC_PULL_ONLINE_init()
{
    ADC_DMA_init();
}
float MC_PULL_stu_raw[4] = {0, 0, 0, 0};
int MC_PULL_stu[4] = {0, 0, 0, 0};
float MC_ONLINE_key_stu_raw[4] = {0, 0, 0, 0};
// 0-Offline 1-Online dual microswitch trigger 2-Outer trigger 3-Inner trigger
int MC_ONLINE_key_stu[4] = {0, 0, 0, 0};

// Voltage control related constants
float PULL_voltage_up = 1.85f;   // Status pressure high red light
float PULL_voltage_down = 1.45f; // Status pressure low blue light
#define PULL_VOLTAGE_SEND_MAX 1.7f
// Microswitch trigger control related constants
bool Assist_send_filament[4] = {false, false, false, false};
bool pull_state_old = false; // Last trigger state - True: not triggered, False: feeding completed
bool is_backing_out = false;
uint64_t Assist_filament_time[4] = {0, 0, 0, 0};
uint64_t Assist_send_time = 1200; // Only trigger outer after, feeding duration
// Retraction distance unit MM
float_t P1X_OUT_filament_meters = 200.0f; // Built-in 200mm external 700mm
float_t last_total_distance[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // Initialize retraction start distance
// bool filament_channel_inserted[4]={false,false,false,false};//Channel inserted
// Use dual microswitch
#define is_two false

void MC_PULL_ONLINE_read()
{
    float *data = ADC_DMA_get_value();
    MC_PULL_stu_raw[3] = data[0];
    MC_ONLINE_key_stu_raw[3] = data[1];
    MC_PULL_stu_raw[2] = data[2];
    MC_ONLINE_key_stu_raw[2] = data[3];
    MC_PULL_stu_raw[1] = data[4];
    MC_ONLINE_key_stu_raw[1] = data[5];
    MC_PULL_stu_raw[0] = data[6];
    MC_ONLINE_key_stu_raw[0] = data[7];

    for (int i = 0; i < 4; i++)
    {
        /*
        if (i == 0){
            DEBUG_MY("MC_PULL_stu_raw = ");
            DEBUG_float(MC_PULL_stu_raw[i],3);
            DEBUG_MY("  MC_ONLINE_key_stu_raw = ");
            DEBUG_float(MC_ONLINE_key_stu_raw[i],3);
            DEBUG_MY("  Channel:");
            DEBUG_float(i,1);
            DEBUG_MY("   \n");
        }
        */
        if (MC_PULL_stu_raw[i] > PULL_voltage_up) // Greater than 1.85V, pressure too high
        {
            MC_PULL_stu[i] = 1;
        }
        else if (MC_PULL_stu_raw[i] < PULL_voltage_down) // Less than 1.45V, pressure too low
        {
            MC_PULL_stu[i] = -1;
        }
        else // 1.4~1.7 between normal error range, no action needed
        {
            MC_PULL_stu[i] = 0;
        }
        /*Online status*/

        // Filament online judgment
        if (is_two == false)
        {
            // Greater than 1.65V, filament online, high level.
            if (MC_ONLINE_key_stu_raw[i] > 1.65)
            {
                MC_ONLINE_key_stu[i] = 1;
            }
            else
            {
                MC_ONLINE_key_stu[i] = 0;
            }
        }
        else
        {
            // DEBUG_MY(MC_ONLINE_key_stu_raw);
            // Dual microswitch
            if (MC_ONLINE_key_stu_raw[i] < 0.6f)
            { // Less than offline.
                MC_ONLINE_key_stu[i] = 0;
            }
            else if ((MC_ONLINE_key_stu_raw[i] < 1.7f) & (MC_ONLINE_key_stu_raw[i] > 1.4f))
            { // Only trigger outer microswitch, need auxiliary feeding
                MC_ONLINE_key_stu[i] = 2;
            }
            else if (MC_ONLINE_key_stu_raw[i] > 1.7f)
            { // Dual microswitch simultaneously triggered, online status
                MC_ONLINE_key_stu[i] = 1;
            }
            else if (MC_ONLINE_key_stu_raw[i] < 1.4f)
            { // Only trigger inner microswitch, confirm if out of filament or vibration.
                MC_ONLINE_key_stu[i] = 3;
            }
        }
    }
}

#define PWM_lim 1000

struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    int check = 0x40614061;
} Motion_control_data_save;

#define Motion_control_save_flash_addr ((uint32_t)0x0800E000)
bool Motion_control_read()
{
    Motion_control_save_struct *ptr = (Motion_control_save_struct *)(Motion_control_save_flash_addr);
    if (ptr->check == 0x40614061)
    {
        memcpy(&Motion_control_data_save, ptr, sizeof(Motion_control_save_struct));
        return true;
    }
    return false;
}
void Motion_control_save()
{
    Flash_saves(&Motion_control_data_save, sizeof(Motion_control_save_struct), Motion_control_save_flash_addr);
}

class MOTOR_PID
{

    float P = 0;
    float I = 0;
    float D = 0;
    float I_save = 0;
    float E_last = 0;
    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) / 2;

public:
    MOTOR_PID()
    {
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    MOTOR_PID(float P_set, float I_set, float D_set)
    {
        init_PID(P_set, I_set, D_set);
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    void init_PID(float P_set, float I_set, float D_set) // Note, adopted PID independent calculation method, I and D default multiplied by P
    {
        P = P_set;
        I = I_set;
        D = D_set;
        I_save = 0;
        E_last = 0;
    }
    float caculate(float E, float time_E)
    {
        I_save += I * E * time_E;
        if (I_save > pid_range) // Limit I
            I_save = pid_range;
        if (I_save < -pid_range)
            I_save = -pid_range;

        float ouput_buf;
        if (time_E != 0) // Prevent rapid calls
            ouput_buf = P * E + I_save + D * (E - E_last) / time_E;
        else
            ouput_buf = P * E + I_save;

        if (ouput_buf > pid_MAX)
            ouput_buf = pid_MAX;
        if (ouput_buf < pid_MIN)
            ouput_buf = pid_MIN;

        E_last = E;
        return ouput_buf;
    }
    void clear()
    {
        I_save = 0;
        E_last = 0;
    }
};

enum class filament_motion_enum
{
    filament_motion_send,
    filament_motion_redetect,
    filament_motion_slow_send,
    filament_motion_pull,
    filament_motion_stop,
    filament_motion_pressure_ctrl_on_use,
    filament_motion_pressure_ctrl_idle,
};
enum class pressure_control_enum
{
    less_pressure,
    all,
    over_pressure
};

class _MOTOR_CONTROL
{
public:
    filament_motion_enum motion = filament_motion_enum::filament_motion_stop;
    int CHx = 0;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID_speed = MOTOR_PID(2, 20, 0);
    MOTOR_PID PID_pressure = MOTOR_PID(1500, 0, 0);
    float pwm_zero = 500;
    float dir = 0;
    int x1 = 0;
    _MOTOR_CONTROL(int _CHx)
    {
        CHx = _CHx;
        motor_stop_time = 0;
        motion = filament_motion_enum::filament_motion_stop;
    }

    void set_pwm_zero(float _pwm_zero)
    {
        pwm_zero = _pwm_zero;
    }
    void set_motion(filament_motion_enum _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        motor_stop_time = time_now + over_time;
        if (motion != _motion)
        {
            motion = _motion;
            PID_speed.clear();
        }
    }
    filament_motion_enum get_motion()
    {
        return motion;
    }
    float _get_x_by_pressure(float pressure_voltage, float control_voltage, float time_E, pressure_control_enum control_type)
    {
        float x=0;
        switch (control_type)
        {
        case pressure_control_enum::all: // Full range control
        {
            x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            break;
        }
        case pressure_control_enum::less_pressure: // Low pressure control only
        {
            if (pressure_voltage < control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        case pressure_control_enum::over_pressure: // High pressure control only
        {
            if (pressure_voltage > control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        }
        if (x > 0) // Convert control force to square enhancement, square eliminates positive and negative, need to judge
            x = x * x / 250;
        else
            x = -x * x / 250;
        return x;
    }
    void run(float time_E)
    {
        // When in retraction state and need to retract, start recording mileage.
        if (is_backing_out){
            last_total_distance[CHx] += fabs(speed_as5600[CHx] * time_E);
        }
        float speed_set = 0;
        float now_speed = speed_as5600[CHx];
        float x=0;

        uint16_t device_type = get_now_BambuBus_device_type();
        static uint64_t countdownStart[4] = {0};          // Auxiliary feeding countdown
        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) // In idle state
        {
            // When both microswitches are released
            if (MC_ONLINE_key_stu[CHx] == 0)
            {
                Assist_send_filament[CHx] = true; // After a channel goes offline, auxiliary feeding can be triggered once
                countdownStart[CHx] = 0;          // Clear countdown
            }

            if (Assist_send_filament[CHx] && is_two)
            { // Allowed state, try auxiliary feeding
                if (MC_ONLINE_key_stu[CHx] == 2)
                {                   // Trigger outer microswitch
                    x = -dir * 666; // Drive feeding
                }
                if (MC_ONLINE_key_stu[CHx] == 1)
                { // Simultaneously trigger dual microswitches, prepare to stop
                    if (countdownStart[CHx] == 0)
                    { // Start countdown
                        countdownStart[CHx] = get_time64();
                    }
                    uint64_t now = get_time64();
                    if (now - countdownStart[CHx] >= Assist_send_time) // Countdown
                    {
                        x = 0;                             // Stop motor
                        Assist_send_filament[CHx] = false; // Achieve condition, complete one round of auxiliary feeding.
                    }
                    else
                    {
                        // Drive feeding
                        x = -dir * 666;
                    }
                }
            }
            else
            {
                // Already triggered, or microswitch triggered in other states
                if (MC_ONLINE_key_stu[CHx] != 0 && MC_PULL_stu[CHx] != 0)
                { // If the slider is manually pulled, respond accordingly
                    x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - 1.65, time_E);
                }
                else
                { // Otherwise, keep stopped
                    x = 0;
                    PID_pressure.clear();
                }
            }
        }
        else if (MC_ONLINE_key_stu[CHx] != 0) // Channel is in running state and has filament
        {
            if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) // In use state
            {
                if (pull_state_old) { // First time entering use, do not trigger retraction, flushing will reset buffer.
                    if (MC_PULL_stu_raw[CHx] < 1.55){
                        pull_state_old = false; // Detected filament is at low pressure.
                    }
                } else {
                    if (MC_PULL_stu_raw[CHx] < 1.65)
                    {
                        x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.65, time_E, pressure_control_enum::less_pressure);
                    }
                    else if (MC_PULL_stu_raw[CHx] > 1.7)
                    {
                        x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.7, time_E, pressure_control_enum::over_pressure);
                    }
                }
            }
            else
            {
                if (motion == filament_motion_enum::filament_motion_stop) // Request to stop
                {
                    PID_speed.clear();
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
                if (motion == filament_motion_enum::filament_motion_send) // Feeding
                {
                    if (device_type == BambuBus_AMS_lite)
                    {
                        if (MC_PULL_stu_raw[CHx] < PULL_VOLTAGE_SEND_MAX) // Pressure actively to this position
                            speed_set = 30;
                        else
                            speed_set = 0; // Original here is 10
                    }
                    else
                    {
                        speed_set = 50; // P series go all out
                    }
                }
                if (motion == filament_motion_enum::filament_motion_slow_send) // Request slow feeding
                {
                    speed_set = 3;
                }
                if (motion == filament_motion_enum::filament_motion_pull) // Retract
                {
                    speed_set = -50;
                }
                x = dir * PID_speed.caculate(now_speed - speed_set, time_E);
            }
        }
        else // During operation, filament runs out, need to stop motor control
        {
            x = 0;
        }

        if (x > 10)
            x += pwm_zero;
        else if (x < -10)
            x -= pwm_zero;
        else
            x = 0;

        if (x > PWM_lim)
        {
            x = PWM_lim;
        }
        if (x < -PWM_lim)
        {
            x = -PWM_lim;
        }

        Motion_control_set_PWM(CHx, x);
    }
};
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};

void Motion_control_set_PWM(uint8_t CHx, int PWM)//Pass to hardware layer to control motor PWM
{
    uint16_t set1 = 0, set2 = 0;
    if (PWM > 0)
    {
        set1 = PWM;
    }
    else if (PWM < 0)
    {
        set2 = -PWM;
    }
    else // PWM==0
    {
        set1 = 1000;
        set2 = 1000;
    }
    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    }
}

int32_t as5600_distance_save[4] = {0, 0, 0, 0};
void AS5600_distance_updata()//Read as5600, update related data
{
    static uint64_t time_last = 0;
    uint64_t time_now;
    float T;
    do
    {
        time_now = get_time64();
    } while (time_now <= time_last); // T!=0
    T = (float)(time_now - time_last);
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false))
        {
            as5600_distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        int32_t cir_E = 0;
        int32_t last_distance = as5600_distance_save[i];
        int32_t now_distance = MC_AS5600.raw_angle[i];
        float distance_E;
        if ((now_distance > 3072) && (last_distance <= 1024))
        {
            cir_E = -4096;
        }
        else if ((now_distance <= 1024) && (last_distance > 3072))
        {
            cir_E = 4096;
        }

        distance_E = -(float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096; // D=7.5mm, add negative sign because AS5600 is facing the magnet
        as5600_distance_save[i] = now_distance;

        float speedx = distance_E / T * 1000;
        // T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx; // * (1 - T) + speed_as5600[i] * T; // mm/s
        add_filament_meters(i, distance_E / 1000);
    }
    time_last = time_now;
}

enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_pulling_back,
    filament_redetect,
};
int filament_now_position[4];
bool wait = false;

bool Prepare_For_filament_Pull_Back(float_t OUT_filament_meters)
{
    bool wait = false;
    for (int i = 0; i < 4; i++)
    {
        if (filament_now_position[i] == filament_pulling_back)
        {
            // DEBUG_MY("last_total_distance: "); // Output debug information
            // Debug_log_write_float(last_total_distance[i], 5);
            if (last_total_distance[i] < OUT_filament_meters)
            {
                // When not reached, perform retraction
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pull, 100); // Drive motor retraction
                // Gradient light effect
                float npercent = (last_total_distance[i] / OUT_filament_meters) * 100.0f;
                MC_STU_RGB_set(i, 255 - ((255 / 100) * npercent), 125 - ((125 / 100) * npercent), (255 / 100) * npercent);
                // Retraction not completed needs priority processing
            }
            else
            {
                // Reach stop distance
                is_backing_out = false; // No need to continue recording distance
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100); // Stop motor
                filament_now_position[i] = filament_idle;               // Set current position to idle
                set_filament_motion(i, AMS_filament_motion::idle);      // Force enter idle
                last_total_distance[i] = 0;                             // Reset retraction distance
                // Retraction completed
            }
            // As long as in retraction state must wait, until not in retraction, next cycle will not wait.
            wait = true;
        }
    }
    return wait;
}
void motor_motion_switch() // Channel state switching function, only controls the current channel in use, others set to stop
{
    int num = get_now_filament_num();                      // Current channel number
    uint16_t device_type = get_now_BambuBus_device_type(); // Device type
    for (int i = 0; i < 4; i++)
    {
        if (i != num)
        {
            filament_now_position[i] = filament_idle;
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 1000);
        }
        else if (MC_ONLINE_key_stu[num] == 1 || MC_ONLINE_key_stu[num] == 3) // Channel has filament
        {
            switch (get_filament_motion(num)) // Judge simulator state
            {
            case AMS_filament_motion::need_send_out: // Need feeding
                MC_STU_RGB_set(num, 00, 255, 00);
                filament_now_position[num] = filament_sending_out;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_send, 100);
                break;
            case AMS_filament_motion::need_pull_back:
                pull_state_old = false; // Reset mark
                is_backing_out = true; // Mark as retracting
                filament_now_position[num] = filament_pulling_back;
                if (device_type == BambuBus_AMS_lite)
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pull, 100);
                }
                // Prepare_For_filament_Pull_Back(OUT_filament_meters); // Control retraction completion by distance
                break;
            case AMS_filament_motion::before_pull_back:
            case AMS_filament_motion::on_use:
            {
                static uint64_t time_end = 0;
                uint64_t time_now = get_time64();
                if (filament_now_position[num] == filament_sending_out) // If channel just started feeding
                {
                    is_backing_out = false; // Set no need to record distance
                    pull_state_old = true; // First time won't pull back, will wait for low voltage trigger to avoid filament being pulled out just entered.
                    filament_now_position[num] = filament_using; // Mark as in use
                    time_end = time_now + 1500;                  // Prevent not being engaged, continue feeding for 1.5 seconds
                }
                else if (filament_now_position[num] == filament_using) // Already triggered and in use
                {
                    last_total_distance[i] = 0; // Reset retraction distance
                    if (time_now > time_end)
                    {                                          // Over 1.5 seconds, enter channel use for refilling
                        MC_STU_RGB_set(num, 255, 255, 255); // White
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_on_use, 20);
                    }
                    else
                    {                                                                  // Whether just detected filament
                        MC_STU_RGB_set(num, 128, 192, 128);                         // Light green
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_slow_send, 100); // First 1.5 seconds slow feeding, auxiliary tool head engagement.
                    }
                }
                break;
            }
            case AMS_filament_motion::idle:
                filament_now_position[num] = filament_idle;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
                for (int i = 0; i < 4; i++)
                {
                    // Hardware normal
                    if (MC_ONLINE_key_stu[i] == 1 || MC_ONLINE_key_stu[i] == 0)
                    {   // Simultaneous trigger or no filament
                        MC_STU_RGB_set(i, 0, 0, 255); // Blue
                    }
                    else if (MC_ONLINE_key_stu[i] == 2)
                    {   // Outer trigger only
                        MC_STU_RGB_set(i, 255, 144, 0); // Orange / like gold
                    }
                    else if (MC_ONLINE_key_stu[i] == 3)
                    {   // Inner trigger only
                        MC_STU_RGB_set(i, 0, 255, 255); // Cyan
                    }
                }
                break;
            }
        }
        else if (MC_ONLINE_key_stu[num] == 0) // 0: definitely no filament, 1: simultaneous trigger definitely has filament 2: outer trigger only 3: inner trigger only, here has anti-drop function
        {
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
            // MC_STU_RGB_set(num, 0, 0, 255);
        }
    }
}
// According to AMS simulator information, schedule motor
void motor_motion_run(int error)
{
    uint64_t time_now = get_time64();
    static uint64_t time_last = 0;
    float time_E = time_now - time_last; // Get time difference (ms)
    time_E = time_E / 1000;              // Switch to unit s
    uint16_t device_type = get_now_BambuBus_device_type();
    if (!error) // Normal mode
    {
        // Execute different motor control logic according to device type
        if (device_type == BambuBus_AMS_lite)
        {
            motor_motion_switch(); // Schedule motor
        }
        else if (device_type == BambuBus_AMS)
        {
            if (!Prepare_For_filament_Pull_Back(P1X_OUT_filament_meters)) // Negate (return true), means no need to prioritize retraction, and continue scheduling motor.
            {
                motor_motion_switch(); // Schedule motor
            }
        }
    }
    else // error mode
    {
        for (int i = 0; i < 4; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100); // Turn off motor
    }

    for (int i = 0; i < 4; i++)
    {
        /*if (!get_filament_online(i)) // Channel not online, motor not allowed to work
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);*/
        MOTOR_CONTROL[i].run(time_E); // Drive motor according to status information

        if (MC_PULL_stu[i] == 1)
        {
            MC_PULL_ONLINE_RGB_set(i, 255, 0, 0); // Pressure too high, red light
        }
        else if (MC_PULL_stu[i] == 0)
        { // Normal pressure
            if (MC_ONLINE_key_stu[i] == 1)
            { // Online and dual microswitch trigger
                int filament_colors_R = channel_colors[i][0];
                int filament_colors_G = channel_colors[i][1];
                int filament_colors_B = channel_colors[i][2];
                // According to stored filament color
                MC_PULL_ONLINE_RGB_set(i, filament_colors_R, filament_colors_G, filament_colors_B);
                // Light white
                // MC_STU_RGB_set(i, 255, 255, 255);
            }
            else
            {
                MC_PULL_ONLINE_RGB_set(i, 0, 0, 0); // No filament, no light
            }
        }
        else if (MC_PULL_stu[i] == -1)
        {
            MC_PULL_ONLINE_RGB_set(i, 0, 0, 255); // Pressure too low, blue light
        }
    }
    time_last = time_now;
}
// Motion control function
void Motion_control_run(int error)
{
    MC_PULL_ONLINE_read();

    AS5600_distance_updata();
    for (int i = 0; i < 4; i++)
    {
        if (MC_ONLINE_key_stu[i] == 0) {
            set_filament_online(i, false);
        } else if (MC_ONLINE_key_stu[i] == 1) {
            set_filament_online(i, true);
        } else if (MC_ONLINE_key_stu[i] == 3 && filament_now_position[i] == filament_using) {
            // If only inner trigger and in use, do not go offline first
            set_filament_online(i, true);
        } else if (filament_now_position[i] == filament_redetect || (filament_now_position[i] == filament_pulling_back)) {
            // If in redetect or retracting, do not go offline first
            set_filament_online(i, true);
        } else {
            set_filament_online(i, false);
        }
    }
    /*
        If outer microswitch triggered, orange/ like gold
        If only inner microswitch triggered, // Cyan
        If simultaneous trigger, idle = blue, simultaneously represents filament online, blue + white/channel saved color
    */

    if (error) // Error != 0
    {
        for (int i = 0; i < 4; i++)
        {
            set_filament_online(i, false);
            // filament_channel_inserted[i] = true; // For testing
            if (MC_ONLINE_key_stu[i] == 1)
            {                                        // Simultaneous trigger
                MC_STU_RGB_set(i, 0, 0, 255); // Blue
            }
            else if (MC_ONLINE_key_stu[i] == 2)
            {                                        // Outer trigger only
                MC_STU_RGB_set(i, 255, 144, 0); // Orange/ like gold
            }
            else if (MC_ONLINE_key_stu[i] == 3)
            {                                        // Inner trigger only
                MC_STU_RGB_set(i, 0, 255, 255); // Cyan
            } else if (MC_ONLINE_key_stu[i] == 0)
            {   // Not connected to printer and no filament
                MC_STU_RGB_set(i, 0, 0, 0); // Black
            }
        }
    } else { // Normally connected to printer
        // Setting color here will be repeated modification.
        for (int i = 0; i < 4; i++)
        {
            if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1)) // AS5600 error
            {
                set_filament_online(i, false);
                MC_STU_ERROR[i] = true;
            }
        }
    }
    motor_motion_run(error);
}
// Set PWM to drive motor
void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // Enable multiplexing clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // Enable TIM2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // Enable TIM3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // Enable TIM4 clock

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Timer basic configuration
    TIM_TimeBaseStructure.TIM_Period = 999;  // Period (x+1)
    TIM_TimeBaseStructure.TIM_Prescaler = 1; // Prescaler (x+1)
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM mode configuration
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // Duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA15
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PB3
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PB4
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PB5
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // PB6
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // PB7
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // PB8
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // PB9

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2 full remap -CH1-PA15/CH2-PB3
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3 partial remap -CH1-PB4/CH2-PB5
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4 no remap -CH1-PB6/CH2-PB7/CH3-PB8/CH4-PB9

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}
// Get PWM friction zero point (deprecated, assume 50% duty cycle)
void MOTOR_get_pwm_zero()
{
    float pwm_zero[4] = {0, 0, 0, 0};
    MC_AS5600.updata_angle();

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];
    }
    for (int pwm = 300; pwm < 1000; pwm += 10)
    {
        MC_AS5600.updata_angle();
        for (int index = 0; index < 4; index++)
        {

            if (pwm_zero[index] == 0)
            {
                if (abs(MC_AS5600.raw_angle[index] - last_angle[index]) > 50)
                {
                    pwm_zero[index] = pwm;
                    pwm_zero[index] *= 0.90;
                    Motion_control_set_PWM(index, 0);
                }
                else if ((MC_AS5600.online[index] == true))
                {
                    Motion_control_set_PWM(index, -pwm);
                }
            }
            else
            {
                Motion_control_set_PWM(index, 0);
            }
        }
        delay(100);
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(pwm_zero[index]);
    }
}
// Convert angle value to angle difference value
int M5600_angle_dis(int16_t angle1, int16_t angle2)
{

    int cir_E = angle1 - angle2;
    if ((angle1 > 3072) && (angle2 <= 1024))
    {
        cir_E = -4096;
    }
    else if ((angle1 <= 1024) && (angle2 > 3072))
    {
        cir_E = 4096;
    }
    return cir_E;
}

// Test motor movement direction
void MOTOR_get_dir()
{
    int dir[4] = {0, 0, 0, 0};
    bool done = false;
    bool have_data = Motion_control_read();
    if (!have_data)
    {
        for (int index = 0; index < 4; index++)
        {
            Motion_control_data_save.Motion_control_dir[index] = 0;
        }
    }
    MC_AS5600.updata_angle(); // Read 5600 initial angle value

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];                  // Record initial angle value
        dir[index] = Motion_control_data_save.Motion_control_dir[index]; // Record dir data in flash
    }
    //bool need_test = false; // Whether need to detect
    bool need_save = false; // Whether need to update status
    for (int index = 0; index < 4; index++)
    {
        if ((MC_AS5600.online[index] == true)) // Have 5600, channel online
        {
            if (Motion_control_data_save.Motion_control_dir[index] == 0) // Previous test result 0, need test
            {
                Motion_control_set_PWM(index, 1000); // Turn on motor
                //need_test = true;                    // Set need test
                need_save = true;                    // Have status update
            }
        }
        else
        {
            dir[index] = 0;   // Channel not online, clear its direction data
            need_save = true; // Have status update
        }
    }
    int i = 0;
    while (done == false)
    {
        done = true;

        delay(10);                // Check every 10ms interval
        MC_AS5600.updata_angle(); // Update angle data

        if (i++ > 200) // Over 2s no response
        {
            for (int index = 0; index < 4; index++)
            {
                Motion_control_set_PWM(index, 0);                       // Stop
                Motion_control_data_save.Motion_control_dir[index] = 0; // Direction set to 0
            }
            break; // Break loop
        }
        for (int index = 0; index < 4; index++) // Traverse
        {
            if ((MC_AS5600.online[index] == true) && (Motion_control_data_save.Motion_control_dir[index] == 0)) // For new channel
            {
                int angle_dis = M5600_angle_dis(MC_AS5600.raw_angle[index], last_angle[index]);
                if (abs(angle_dis) > 163) // Move over 1mm
                {
                    Motion_control_set_PWM(index, 0); // Stop
                    if (angle_dis > 0)                // Here AS600 facing magnet, opposite to back direction
                    {
                        dir[index] = 1;
                    }
                    else
                    {
                        dir[index] = -1;
                    }
                }
                else
                {
                    done = false; // No movement. Continue waiting
                }
            }
        }
    }
    for (int index = 0; index < 4; index++) // Traverse four motors
    {
        Motion_control_data_save.Motion_control_dir[index] = dir[index]; // Data copy
    }
    if (need_save) // If need save data
    {
        Motion_control_save(); // Data save
    }
}
// Initialize motor
// Specify direction
int first_boot = 1; // 1 means first boot, for executing only on boot.
void set_motor_directions(int dir0, int dir1, int dir2, int dir3)
{
    Motion_control_data_save.Motion_control_dir[0] = dir0;
    Motion_control_data_save.Motion_control_dir[1] = dir1;
    Motion_control_data_save.Motion_control_dir[2] = dir2;
    Motion_control_data_save.Motion_control_dir[3] = dir3;

    Motion_control_save(); // Save to flash
}
void MOTOR_init()
{

    MC_PWM_init();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    MC_AS5600.init(AS5600_SCL, AS5600_SDA, 4);
    // MOTOR_get_pwm_zero();
    // Auto direction
    MOTOR_get_dir();

    // Fixed motor direction use
    if (first_boot == 1)
    { // First boot
        // set_motor_directions(1 , 1 , 1 , 1 ); // 1 for forward -1 for reverse
        first_boot = 0;
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(500);
        MOTOR_CONTROL[index].dir = Motion_control_data_save.Motion_control_dir[index];
    }
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        as5600_distance_save[i] = MC_AS5600.raw_angle[i];
    }
}
extern void RGB_update();
void Motion_control_init() // Initialize all motion and sensors
{
    MC_PULL_ONLINE_init();
    MC_PULL_ONLINE_read();
    MOTOR_init();
    
    /*
    //This is a blocking DEBUG code
    while (1)
    {
        delay(10);
        MC_PULL_ONLINE_read();

        for (int i = 0; i < 4; i++)
        {
            MOTOR_CONTROL[i].set_motion(filament_motion_pressure_ctrl_on_use, 100);
            if (!get_filament_online(i)) // Channel not online, motor not allowed to work
                MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
            MOTOR_CONTROL[i].run(0); // Drive motor according to status information
        }
        char s[100];
        int n = sprintf(s, "%d\n", (int)(MC_PULL_stu_raw[3] * 1000));
        DEBUG_num(s, n);
    }*/

    for (int i = 0; i < 4; i++)
    {
        // if(MC_AS5600.online[i])//Use AS5600 signal to judge if channel is inserted
        // {
        //     filament_channel_inserted[i]=true;
        // }
        // else
        // {
        //     filament_channel_inserted[i]=false;
        // }
        filament_now_position[i] = filament_idle;//Set channel initial state to idle
    }
}
