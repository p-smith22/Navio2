#include <unistd.h>
#include <cstdio>
#include <memory>
#include <vector>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCInput_Navio2.h"
#include "Navio+/RCInput_Navio.h"
#include "Common/Util.h"

using namespace Navio;

constexpr int NUM_MOTORS  = 4;
constexpr int PWM_START   = 0;

constexpr int SERVO_MIN   = 1000;
constexpr int SERVO_MAX   = 1600;   
constexpr int SERVO_IDLE  = 1100;

constexpr int FEED_US     = 50000; 

#define READ_FAILED -1

// =====================
// RC CHANNEL MAPPING
// =====================
constexpr int CHANNEL_ROLL       = 0;
constexpr int CHANNEL_PITCH      = 1;
constexpr int CHANNEL_THROTTLE   = 2;
constexpr int CHANNEL_YAW        = 3;
constexpr int CHANNEL_SAFETY     = 4;
constexpr int CHANNEL_AUTO       = 5;

constexpr float ROLL_SCALE  = 0.10f;  
constexpr float PITCH_SCALE = 0.30f;
constexpr float YAW_SCALE   = 0.20f;


// =====================
// Clamp motor commands
// =====================
int clamp_pwm(int value) {
    if (value < SERVO_MIN) return SERVO_MIN;
    if (value > SERVO_MAX) return SERVO_MAX;
    return value;
}

std::unique_ptr<RCOutput> get_rcout() {
    if (get_navio_version() == NAVIO2)
        return std::unique_ptr<RCOutput_Navio2>(new RCOutput_Navio2());
    else
        return std::unique_ptr<RCOutput_Navio>(new RCOutput_Navio());
}
std::unique_ptr<RCInput> get_rcin() {
    if (get_navio_version() == NAVIO2)
        return std::unique_ptr<RCInput_Navio2>(new RCInput_Navio2());
    else
        return std::unique_ptr<RCInput_Navio>(new RCInput_Navio());
}


// =====================
// Main
// =====================
int main() {

    if (check_apm()) return 1;

    if (getuid()) {
        fprintf(stderr, "Not root. Use: sudo\n");
        return 1;
    }

    auto pwm  = get_rcout();
    auto rcin = get_rcin();

    // Initialize motors
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (!pwm->initialize(PWM_START + i)) return 1;
        pwm->set_frequency(PWM_START + i, 400);
        if (!pwm->enable(PWM_START + i)) return 1;
        pwm->set_duty_cycle(PWM_START + i, SERVO_MIN);
    }

    rcin->initialize();

    printf("Manual flight mixer started.\n");
    printf("Safety ON = motors cut.\n");


    int throttle, safety, autonomy;

    while (true) {

        throttle = rcin->read(CHANNEL_THROTTLE); 
        if ( safety > 1750 ) { 
            if (throttle < 1020) { 
                safety = rcin->read(CHANNEL_SAFETY); 
            } 
        } 
        else { 
            safety = rcin->read(CHANNEL_SAFETY); 
        } 
        autonomy = rcin->read(CHANNEL_AUTO);

        if (throttle == READ_FAILED || safety == READ_FAILED || autonomy == READ_FAILED) {
            fprintf(stderr, "RC read failed\n");
            return 1;
        }

        const char* mode = (autonomy > 1750) ? "AUTO" : "MANUAL";

        // =====================
        // SAFETY CUT
        // =====================
        if (safety > 1750) {
            for (int i = 0; i < NUM_MOTORS; ++i)
                pwm->set_duty_cycle(PWM_START + i, SERVO_MIN);

            printf("[SAFETY] Motors cut\n");
            usleep(FEED_US);
            continue;
        }

        // =====================
        // Read Controls
        // =====================
        int roll     = rcin->read(CHANNEL_ROLL);
        int pitch    = rcin->read(CHANNEL_PITCH);
        int yaw      = rcin->read(CHANNEL_YAW);

        // Center sticks
        float roll_offset  = roll  - 1500;
        float pitch_offset = pitch - 1500;
        float yaw_offset   = yaw   - 1500;

        roll_offset  *= ROLL_SCALE;
        pitch_offset *= -PITCH_SCALE;
        yaw_offset   *= YAW_SCALE;

        /*
            X Configuration

                 Front
           M2(CCW)    M4(CW)

           M1(CW)     M3(CCW)
                 Back

            Pin mapping:
            M1 = 0
            M2 = 1
            M3 = 2
            M4 = 3
        */

        int m1 = throttle - pitch_offset + roll_offset - yaw_offset; // Back Left (CW)
        int m2 = throttle + pitch_offset + roll_offset + yaw_offset; // Front Left (CCW)
        int m3 = throttle - pitch_offset - roll_offset + yaw_offset; // Back Right (CCW)
        int m4 = throttle + pitch_offset - roll_offset - yaw_offset; // Front Right (CW)

        m1 = clamp_pwm(m1);
        m2 = clamp_pwm(m2);
        m3 = clamp_pwm(m3);
        m4 = clamp_pwm(m4);

        pwm->set_duty_cycle(0, m1);
        pwm->set_duty_cycle(1, m2);
        pwm->set_duty_cycle(2, m3);
        pwm->set_duty_cycle(3, m4);

        printf("[%s] T:%d | Rg:%.2f Pg:%.2f Yg:%.2f\n",
               mode, throttle);

        usleep(FEED_US);
    }

    return 0;
}
