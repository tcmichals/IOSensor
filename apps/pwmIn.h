
/*
 * PB3   CH0
 * PB4   CH1
 * PB12  CH2
 * PB13  CH3
 * PB14  CH4
 * PB15  CH5
 */

#define CHAN0_PN (0)
#define CHAN1_PN (1)
#define CHAN2_PN (2)
#define CHAN3_PN (3)
#define CHAN4_PN (4)
#define CHAN5_PN (5)

#define ALL_PWM_IN ( (1 << CHAN0_PN) | (1 << CHAN1_PN) | (1 << CHAN2_PN) | ( 1 << CHAN3_PN) | ( 1 << CHAN4_PN) | ( 1 << CHAN5_PN) )

void updatePWMChannelFromISR(int pin, uint8_t value, uint16_t timerVal);

bool createPWMInMsg(void);