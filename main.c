#include "msp430g2553.h"
#include <time.h>
#include <stdlib.h>
#include <math.h>

/******************************************************************************************************/
//Macro declarations and global variables
#define CSN     BIT5                        // Chip select pin
#define SCK     BIT4                        // SCLK
#define MOSI    BIT2                        // Master out slave in
#define MISO    BIT1                        // Master in slave out
#define NCS_H   P1OUT |= CSN
#define NCS_L   P1OUT &= ~CSN
#define ALL_MOTOR BIT2 + BIT3 + BIT5 + BIT6

#define SMPLRT_DIV      0x19
#define CONFIG          0x18
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D
#define PWR_MGMT_1      0x6B
#define WHO_AM_I        0x75
#define SlaveAddress    0x68                //MPU6050
#define DETECT_CTRL      0x6a
#define DMP_CFG_1        0x70
#define DMP_CFG_2        0x71

unsigned char dmpdatas[42];
float accelf[3];
int speed = 0;
int counter;
/******************************************************************************************************/
//Initializing the functions that control the robot’s movement
void turn_right(int);
void turn_left(int);
void move_forward(int);
void move_backward(int);
void stop(void);

//The variables we use to read data from the IMU
unsigned char mpu9250_Write_Reg(unsigned char reg, unsigned char value);
unsigned char mpu9250_Read_Reg(unsigned char reg);
unsigned char SPI_RW(unsigned char Byte);
/******************************************************************************************************/

void main(void)
{
    /****************************************************************************************************/
    //Initialization
    if (CALBC1_8MHZ == 0xFF || CALDCO_8MHZ == 0xff)
        while (1)
            ;                               // Erased calibration data? Trap!

    counter = 0;

    //Start WDT
    IE1 |= WDTIE;                           // Watchdog interrupt enable
    WDTCTL = WDT_MDLY_0_5;                  // Watchdog timer 0.5ms

    BCSCTL1 = CALBC1_8MHZ;                  // Set the DCO to 8 MHz
    DCOCTL = CALDCO_8MHZ;                   // And load calibration data
    BCSCTL3 |= LFXT1S_2;
    IFG1 &= ~OFIFG;

    srand(time(NULL));                      // start the random seeding

    /*
     * LEDs and Corresponding Sensor
     * Sensor 1 - P2.1 - LED1 - P2.5
     * Sensor 2 - P1.6 - LED2 - P2.4
     * Sensor 3 - P2.2 - LED3 - P2.3
     * Sensor 4 - P2.7 - LED4 - P3.4
     */

    /* Initialize the Sensors */
    P1IE |= BIT6;
    P1IFG &= ~BIT6;

    /* Initialize the LEDs */
    P2DIR |= BIT4 + BIT3 + BIT5;            // Set the Direction
    P3DIR |= BIT4;

    /*
     * Motors and Corresponding Pin
     * Motor A1 - P3.2
     * Motor A2 - P3.3
     * Motor B1 - P3.6
     * Motor B2 - P3.5
     *
     * Note: Motor A uses TA.1, Motor B uses TA.0
     */

    /* Enable Motor Control */
    P3DIR |= BIT2 + BIT3 + BIT5 + BIT6;
    P3SEL |= BIT2 + BIT3 + BIT5 + BIT6;
    P3SEL2 &= ~(BIT2 + BIT3 + BIT5 + BIT6);

    /* Standard PWM Setup */
    TA0CCTL1 |= OUTMOD_7;
    TA0CCTL2 |= OUTMOD_7;
    TA0CTL |= TASSEL_1 + MC_1;
    TA0CCR0 = 99;
    TA0CCR1 = 0;                            // Corresponds to pin 3.5
    TA0CCR2 = 0;                            // Corresponds to pin 3.6

    TA1CCTL1 |= OUTMOD_7;
    TA1CCTL2 |= OUTMOD_7;
    TA1CTL |= TASSEL_1 + MC_1;
    TA1CCR0 = 99;
    TA1CCR1 = 0;                            // Corresponds to pin 3.2
    TA1CCR2 = 0;                            // Corresponds to pin 3.3

    /* SPI set up */
    P1DIR |= CSN;
    P1SEL2 |= SCK + MISO + MOSI;
    P1SEL |= SCK + MISO + MOSI;
    UCA0CTL1 |= UCSWRST;
    UCA0CTL0 |= UCCKPH + UCMSB + UCMST + UCMODE_0 + UCSYNC;
    UCA0CTL1 = UCSSEL_2 + UCSWRST;
    UCA0BR0 = 32;
    UCA0CTL1 &= ~UCSWRST;

    /* Setting up the IMU registers */
    mpu9250_Write_Reg(PWR_MGMT_1, 0x00);
    mpu9250_Write_Reg(CONFIG, 0x06);        // configure digital low pass filter
    mpu9250_Write_Reg(GYRO_CONFIG, 0x18);
    configure scale
    to + -250
    degrees / sec
    mpu9250_Write_Reg(ACCEL_CONFIG, 0x00);  // configures scale to +-2g
    mpu9250_Write_Reg(ACCEL_CONFIG2, 0x0E);

    /* IMU Variables */
    unsigned char accelzh = 0;
    unsigned char accelzl = 0;
    float accelz;                           // acceleration in the z axis

    unsigned char gyroyh = 0;
    unsigned char gyroyl = 0;
    float gyroy;                            // detect orientation change of falling over a cliff
    ;

    P2OUT = 0;
    int direction, duration;
    direction = 0;                          // forward direction
    int speed = 24;                         // set speed of motor
    __enable_interrupt();
    long counter2 = 0;
    long debounce1 = 0;
    long debounce2 = 0;
    int first_turn = 1;
    int turn_dir = -1;
    /****************************************************************************************************/
    //Robot finite state machine
    while (1)
    {
        /* If direction is 0 then we want the robot to move forward until it detects an edge */
        if (direction == 0)
        {
            first_turn = 1;                 // When the robot starts off, we turn it once
            if (counter2 < 50)
            {
                move_forward(16);           // Start the robot slowly so it doesn’t jump
            }
            else if (counter2 < 130)
            {
                move_forward(30);           // Giving the robot a start the motor can get the wheels running
            }
            else
            {
                move_forward(15);           // Slowing down so the robot can react to cliffs
            }

            /* Reading the values from the IMU so the robot can react to its orientation */
            gyroyh = mpu9250_Read_Reg(69);
            gyroyl = mpu9250_Read_Reg(70);
            gyroy = (gyroyh << 8 | gyroyl) / 16.384;
            accelzh = mpu9250_Read_Reg(63);
            accelzl = mpu9250_Read_Reg(64);
            accelz = (accelzh << 8 | accelzl) / 32767.0;
            // Generate a random number to pick duration
            if (gyroy < -40.8 && accelz < .3 && counter2 > 400)
            {                               // it is falling
                if (debounce1 > 2)
                {
                    P2OUT |= BIT4 + BIT6 + BIT5 + BIT3;
                    direction = 1;          // Setting the robot to its reverse state
                    move_backward(65);      // Back up quickly to get away from the cliff
                    counter2 = 0;           // Initialize the time count
                    debounce1 = 0;          // Basic software debounce count
                }
                else
                {
                    debounce1 += 1;
                }
            }
            P2OUT &= ~(BIT4 + BIT6 + BIT5 + BIT3);
        }
        else
        {                                   // Move backwards

            accelzh = mpu9250_Read_Reg(63);
            accelzl = mpu9250_Read_Reg(64);
            accelz = (accelzh << 8 | accelzl) / 32767.0;

            if (counter2 > 9600)            // Hard limit on how long we let the robot back up
            {
                direction = 0;
                counter2 = 0;
            }
            else if ((counter2 > 7300) && (accelz > 0.45)) // Checking if the robot has stabilized
            {
                if (first_turn == 1)
                {                           // Randomly turning the robot
                    turn_dir = rand() % 2;
                    first_turn = 0;
                }
                if (turn_dir == 0)
                {
                    turn_right(60);
                }
                else
                {
                    turn_left(60);
                }
            }
            else if (counter2 > 2200)
            {
                move_backward(16);          // Slowing the robot back down
            }
        }
        counter2 += 1;                      // Incrementing the time count

    }
}

/*
 * To stop, we need to set all of the registers to zero.
 */
void stop(void)
{
    speed = 0;

    TA0CCR1 = 0;
    TA0CCR2 = 0;

    TA1CCR1 = 0;
    TA1CCR2 = 0;
}

/*
 * To move forwards, we want the left motor to run forwards,
 * and the right motor to run forwards. This means for the
 * left motor, B, we want B1 low and B2 high. As a corollary,
 * we want A1 low and A2 high.
 */
void move_forward(int speed)
{
    counter = 0;

    TA0CCR1 = speed;
    TA0CCR2 = 0;

    TA1CCR1 = 0;
    TA1CCR2 = speed;
}

/*
 * To move backwards, we want the left motor to run in reverse,
 * and the right motor to run in reverse. This means for the
 * left motor, B, we want B1 low and B2 high. As a corollary,
 * we want A1 low and A2 high.
 */
void move_backward(int speed)
{
    counter = 0;

    TA0CCR1 = 0;
    TA0CCR2 = speed;

    TA1CCR1 = speed;
    TA1CCR2 = 0;
}

/*
 * To turn right, we want the left motor to run forwards,
 * and the right motor to run in reverse. This means for the
 * left motor, B, we want B1 low and B2 high. As a corollary,
 * we want A1 high and A2 low.
 */
void turn_right(int speed)
{

    TA0CCR1 = speed;
    TA0CCR2 = 0;

    TA1CCR1 = speed;
    TA1CCR2 = 0;
}

/*
 * To turn left, we want the left motor to run in reverse,
 * and the right motor to run forwards. This means for the
 * left motor, B, we want B1 high and B2 low. As a corollary,
 * we want A1 low and A2 high..............................................
 */
void turn_left(int speed)
{

    TA0CCR1 = 0;
    TA0CCR2 = speed;

    TA1CCR1 = 0;
    TA1CCR2 = speed;
}

/*
 * write to a register on the IMU
 */
unsigned char mpu9250_Write_Reg(unsigned char reg, unsigned char value)
{
    unsigned char status;
    NCS_L;
    status = SPI_RW(reg);
    SPI_RW(value);
    NCS_H;                      // CSN=1;
    return status;
}

/*
 * read from a register on the imu
 */
unsigned char mpu9250_Read_Reg(unsigned char reg)
{
    unsigned char value;
    NCS_L;                      // CSN=0;
    SPI_RW(reg + 0x80);
    value = SPI_RW(0);
    NCS_H;                      // CSN=1;
    return value;
}

/*
 * Wait for one byte of data to be transmitted or received
 */
unsigned char SPI_RW(unsigned char Byte)
{
    while (!(IFG2 & UCA0TXIFG))
        ;                       // USCI_A0 TX buffer ready?
    UCA0TXBUF = Byte;           // Send value

    while (!(IFG2 & UCA0RXIFG))
        ;                       // Has USCI_A0 RX buffer received data?

    return (UCA0RXBUF);         // Received value

}

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
    counter += 1;
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_ISR(void)
{
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer1_ISR(void)
{
}
