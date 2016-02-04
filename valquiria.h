#ifndef VALQUIRIA_H
#define VALQUIRIA_H

#if defined (__SDCC)
#include <pic18fregs.h>
#include <delay.h>
#else
#include <xc.h>
#include <plib.h>
#include <delays.h>
#endif
#include <stdio.h>

#if defined (__SDCC)
#include <adc.h>
#include <usart.h>
#include <signal.h>
#endif

// CONFIG1L @ 0x300000
#pragma config CPUDIV = NOCLKDIV    // CPU System Clock Selection bits
#pragma config USBDIV = OFF        // USB Clock Selection bit
// CONFIG1H @ 0x300001
#pragma config IESO = OFF        // Internal/External Oscillator Switchover bit
#pragma config PLLEN = OFF        // 4 X PLL Enable bit
#pragma config FOSC = IRC        // Oscillator Selection bits
#pragma config FCMEN = OFF        // Fail-Safe Clock Monitor Enable
#pragma config PCLKEN = ON        // Primary Clock Enable bit
// CONFIG2L @ 0x300002
#pragma config BOREN = OFF        // Brown-out Reset Enable bits
#pragma config BORV = 19        // Brown-out Reset Voltage bits
#pragma config PWRTEN = OFF        // Power-up Timer Enable bit
// CONFIG2H @ 0x300003
#pragma config WDTPS = 32768        // Watchdog Timer Postscale Select bits
#pragma config WDTEN = OFF        // Watchdog Timer Enable bit
// CONFIG3H @ 0x300004
#pragma config MCLRE = ON        // MCLR Pin Enable bit
#pragma config HFOFST = OFF        // HFINTOSC Fast Start-up bit
// CONFIG4L @ 0x300006
#pragma config DEBUG = OFF        // Background Debugger Enable bit
#pragma config STVREN = ON        // Stack Full/Underflow Reset Enable bit
#pragma config XINST = OFF        // Extended Instruction Set Enable bit
#pragma config BBSIZ = OFF        // Boot Block Size Select bit
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit
// CONFIG5L @ 0x300008
#pragma config CP0 = OFF        // Code Protection bit
#pragma config CP1 = OFF        // Code Protection bit
// CONFIG5H @ 0x300009
#pragma config CPD = OFF        // Data EEPROM Code Protection bit
#pragma config CPB = OFF        // Boot Block Code Protection bit
// CONFIG6L @ 0x30000A
#pragma config WRT0 = OFF        // Table Write Protection bit
#pragma config WRT1 = OFF        // Table Write Protection bit
// CONFIG6H @ 0x30000B
#pragma config WRTB = OFF        // Boot Block Write Protection bit
#pragma config WRTC = OFF        // Configuration Register Write Protection bit
#pragma config WRTD = OFF        // Data EEPROM Write Protection bit
// CONFIG7L @ 0x30000C
#pragma config EBTR0 = OFF        // Table Read Protection bit
#pragma config EBTR1 = OFF        // Table Read Protection bit
// CONFIG7H @ 0x30000D
#pragma config EBTRB = OFF        // Boot Block Table Read Protection bit

#define _XTAL_FREQ    16000000UL

#define USE_DEBUG
#define string        const MEM_MODEL rom char*

#define FL_SERVO    LATAbits.LATA5                        // Front-Left Servo
#define FR_SERVO    LATAbits.LATA4                        // Front-Right Servo
#define BL_SERVO    LATCbits.LATC5                        // Back-Left Servo
#define BR_SERVO    LATCbits.LATC4                        // Back-Right Servo

#define FL_SERVO_TRIS    TRISAbits.TRISA5                    // Front-Left Servo
#define FR_SERVO_TRIS    TRISAbits.TRISA4                    // Front-Right Servo
#define BL_SERVO_TRIS    TRISCbits.TRISC5                    // Back-Left Servo
#define BR_SERVO_TRIS    TRISCbits.TRISC4                    // Back-Right Servo

#define Ticks4Window    5000UL * (_XTAL_FREQ/4000000UL) // PWM Window for servo = 5 ms x 4 = 20 ms
#define Ticks4CCW       700    * (_XTAL_FREQ/4000000UL) // PWM High for Counter-Clockwise Rotation = 0.7 ms
#define Ticks4Center    1500   * (_XTAL_FREQ/4000000UL) // PWM High for Center  Position = 1.5 ms
#define Ticks4CW        2300   * (_XTAL_FREQ/4000000UL) // PWM High for Clockwise Rotation = 2.3 ms
//#define Ticks4Window  20000                           // PWM Window for servo = 5 ms x 4 = 20 ms
//#define Ticks4CCW     2800                            // PWM High for Counter-Clockwise Rotation = 0.7 ms
//#define Ticks4Center  6000                            // PWM High for Center  Position = 1.5 ms
//#define Ticks4CW      9200                            // PWM High for Clockwise Rotation = 2.3 ms

#define FL_SENSOR   7
#define ML_SENSOR   8
#define BL_SENSOR   9
#define FR_SENSOR   6
#define MR_SENSOR   10
#define BR_SENSOR   11
#define OL_SENSOR   4
#define OR_SENSOR   5

#define TRUE        1
#define FALSE       0

#define FORWARD     'f'
#define BACKWARD    'b'
#define LEFT        'l'
#define RIGHT       'r'
#define STOP        's'

// Modes for the State Machine
#define SURVIVE     0
#define HUNT        1
#define TARGET      2
#define ATTACK      3

#if !defined(__SDCC)
#define delay10ktcy(x) Delay10KTCYx(x)

#define usart_putc(x) WriteUSART(x)
#define usart_getc() getcUSART()
#define usart_open(x, y) OpenUSART(x, y)
#define usart_busy() BusyUSART()

#define adc_busy() BusyADC()
#define adc_read() ReadADC()
#endif

// Serial variables
volatile char last_char_recv = ' ';
volatile char char_recv;
volatile char autonomous = FALSE;

volatile int Ticks4NextInterrupt = 0;
volatile int Servo_PWM[4] = {Ticks4CCW, Ticks4CCW, Ticks4CCW, Ticks4CCW};
volatile char Servo_Idx = 0;

volatile struct
{
    unsigned FL : 1;
    unsigned FR : 1;
    unsigned BL : 1;
    unsigned BR : 1;
    unsigned Kbhit : 1;
    unsigned Phase : 1;
    unsigned done : 1;
} Servo = {0, 0, 0, 0, 0, 0, 0};

volatile struct
{
    unsigned atLeftEdge : 1;
    unsigned atRightEdge : 1;
    unsigned state : 2;
    unsigned searchClockwise : 1;
} Sumo = {0, 0, 0, 1};

int mRightEdgeThreshold = 100; // Cambiar en base a la pista
int mLeftEdgeThreshold = 100; // Cambiar en base a la pista
int mTargetThreshold = 130;
int mAttackThreshold = 130;
int mLeftLine;
int mRightLine;
int mLeftRange;
int mCenterRange;
int mRightRange;
int rangeDifference;
int rangeAverage;

#if defined (__SDCC)
static void ISRRx ( void ) __interrupt 1;
#else
void interrupt ISRRx ( void );
#endif

void write_timer_0(unsigned int value);
void init_sumo ( void );
void sumo_move ( char direction );
int read_a2d ( char channel );
void get_sensors ( void );
void wait_5s ( void );
void putch ( char data );
void process_serial(void);
void do_autonomous(void);

#endif
