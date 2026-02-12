// Include header files:
#include <cstdio>
#include <Navio2/RCInput_Navio2.h>
#include <Navio+/RCInput_Navio.h>
#include <memory>
#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"

// Define constants:
#define READ_FAILED -1
#define THRESHOLD 1750
#define PWM_Output { 0, 1, 2, 3 }
#define FEED_US 50000

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

std::unique_ptr <RCInput> get_rcin()
{
    if (get_navio_version() == NAVIO2)
    {
        auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
        return ptr;
    } 
    
    else
    {
        auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio() };
        return ptr;
    }

}

int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }
    auto rcin = get_rcin();
    rcin->initialize();

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

    while (true)
    {
        int period = rcin->read(2);
        if (period == READ_FAILED)
            return EXIT_FAILURE;
        printf("%d\n", period);
        
        sleep(1);
    }



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

