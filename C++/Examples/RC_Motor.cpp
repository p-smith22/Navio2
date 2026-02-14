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
constexpr int PWM_START   = 0;       // Motor PWM channels 0-3
constexpr int SERVO_MIN   = 1000;    // µs
constexpr int SERVO_MAX   = 2000;    // µs
constexpr int SERVO_IDLE  = 1100;    // µs
constexpr int FEED_US     = 50000;   // 50 ms

constexpr int CHANNEL_THROTTLE = 2;  // RC input channel for throttle
constexpr int CHANNEL_SAFETY   = 5;  // Safety switch channel
constexpr int CHANNEL_AUTO     = 6;  // Autonomy switch channel

#define READ_FAILED -1

// Create RCOutput depending on board
std::unique_ptr<RCOutput> get_rcout() {
    if (get_navio_version() == NAVIO2)
        return std::unique_ptr<RCOutput_Navio2>(new RCOutput_Navio2());
    else
        return std::unique_ptr<RCOutput_Navio>(new RCOutput_Navio());
}

// Create RCInput depending on board
std::unique_ptr<RCInput> get_rcin() {
    if (get_navio_version() == NAVIO2)
        return std::unique_ptr<RCInput_Navio2>(new RCInput_Navio2());
    else
        return std::unique_ptr<RCInput_Navio>(new RCInput_Navio());
}

int main() {
    if (check_apm()) return 1;

    if (getuid()) {
        fprintf(stderr, "Not root. Use: sudo\n");
        return 1;
    }

    auto pwm = get_rcout();
    auto rcin = get_rcin();

    // Initialize all 4 motor channels
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (!pwm->initialize(PWM_START + i)) return 1;
        pwm->set_frequency(PWM_START + i, 400);
        if (!pwm->enable(PWM_START + i)) return 1;
        pwm->set_duty_cycle(PWM_START + i, SERVO_MIN);
    }

    // Initialize RC input
    rcin->initialize();

    printf("Starting motors control loop. Safety up = stop motors.\n");

    while (true) {
        int throttle = rcin->read(CHANNEL_THROTTLE);
        int safety   = rcin->read(CHANNEL_SAFETY);
        int autonomy = rcin->read(CHANNEL_AUTO);

        if (throttle == READ_FAILED || safety == READ_FAILED || autonomy == READ_FAILED) {
            fprintf(stderr, "RC read failed\n");
            return 1;
        }

        // Determine mode
        const char* mode = (autonomy > 1750) ? "AUTO" : "MANUAL";

        // Safety switch: if up (value > 1500), cut all motors
        if (safety > 1750) {
            for (int i = 0; i < NUM_MOTORS; ++i)
                pwm->set_duty_cycle(PWM_START + i, SERVO_MIN);
            printf("[SAFETY] Motors cut\n");
        } else {
            // Otherwise, send throttle to all motors
            for (int i = 0; i < NUM_MOTORS; ++i)
                pwm->set_duty_cycle(PWM_START + i, throttle);

            printf("[%s] Throttle %d\n", mode, throttle);
        }

        usleep(FEED_US);
    }

    return 0;
}
