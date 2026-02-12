/*
Code modified based on example from Emlid Ltd (c) 2015.
*/

#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <unistd.h>
#include <memory>

constexpr int PWM_OUTPUT  = 0;
constexpr int SERVO_MIN   = 1000;   // µs
constexpr int SERVO_MAX   = 2000;   // µs
constexpr int SERVO_IDLE  = 1100;   // µs
constexpr int SERVO_TEST  = 1250;   // µs
constexpr int FEED_US     = 50000;  // 50 ms (<= 100 ms requirement)

#define CALIBRATION 0

using namespace Navio;

std::unique_ptr <RCOutput> get_rcout()
{
    if (get_navio_version() == NAVIO2)
    {
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
        return ptr;
    } else
    {
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio() };
        return ptr;
    }

}

int main(int argc, char *argv[])
{

    std::unique_ptr <RCOutput> pwm = get_rcout();

    if (check_apm()) {
        return 1;
    }

    if (getuid()) {
        fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    }


    if( !(pwm->initialize(PWM_OUTPUT)) ) {
        return 1;
    }

	pwm->set_frequency(PWM_OUTPUT, 400);

	if ( !(pwm->enable(PWM_OUTPUT)) ) {
	    return 1;
	}

    #if CALIBRATION == 1
    printf("Setting max thrust now - Plug Power Supply NOW.\n");
    const int loops = (10.0 <= 0.0) ? 0 : static_cast<int>(10.0 * 1000000 / FEED_US);
    for (int i = 0; i < loops; ++i) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MAX);
        usleep(FEED_US);
    }
    // Ensure we write at least once even if seconds < FEED_US
    if (loops == 0) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MAX);
        usleep(FEED_US);
    }

    printf("Setting min thrust now - holding for 5 seconds\n");
    for (int i = 0; i < loops; ++i) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        usleep(FEED_US);
    }
    // Ensure we write at least once even if seconds < FEED_US
    if (loops == 0) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        usleep(FEED_US);
    }

    printf("Holding idle - for 5 seconds\n");
    for (int i = 0; i < loops; ++i) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_IDLE);
        usleep(FEED_US);
    }
    // Ensure we write at least once even if seconds < FEED_US
    if (loops == 0) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_IDLE);
        usleep(FEED_US);
    }

    printf("Test Speed - for 5 seconds\n");
    for (int i = 0; i < loops; ++i) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        usleep(FEED_US);
    }
    // Ensure we write at least once even if seconds < FEED_US
    if (loops == 0) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        usleep(FEED_US);
    }
    #else

    printf("Starting zero throttle for 2 seconds.\n");
    const int loops = (2.0 <= 0.0) ? 0 : static_cast<int>(2.0 * 1000000 / FEED_US);
    for (int i = 0; i < loops; ++i) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        usleep(FEED_US);
    }
    // Ensure we write at least once even if seconds < FEED_US
    if (loops == 0) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        usleep(FEED_US);
    }

    printf("Sending command - for 2 seconds\n");
    for (int i = 0; i < loops; ++i) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_TEST);
        usleep(FEED_US);
    }
    // Ensure we write at least once even if seconds < FEED_US
    if (loops == 0) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_TEST);
        usleep(FEED_US);
    }
    #endif

    printf("Stop\n");
    pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
    usleep(FEED_US);

    return 0;
}