#include "valquiria.h"

#if defined (__SDCC)
int main ( void )
#else
void main ( void )
#endif
{
    init_sumo();
    sumo_move(STOP);
    //wait_5s();
    for(;;)
    {
        process_serial();
        do_autonomous();
    }
}

void init_sumo ( void )
{
    OSCCONbits.IRCF = 7;                // Set internal clock to 16 Mhz

    ANSELHbits.ANS11 = 0;               // Make RX digital

    // 115200,8,n,1: 138 - 64MHz
    // 19200,8,n,1: 832 - 64MHz; 207 - 16MHz
    // 9600,8,n,1: 129 - 20 MHz; 1249 - 48MHz; 1666 - 64MHz
    usart_open(USART_TX_INT_OFF &       // disable TX interrupt
               USART_RX_INT_ON &        // enable RX interrupt
               USART_BRGH_HIGH &        // Use High BRGH
               USART_CONT_RX &          // Receive continuous data
               USART_EIGHT_BIT &        // Use 8 bit
               USART_ASYNCH_MODE,       // Use Asynchronous mode
               207);                    // 19200 for 16 Mhz
    #if defined (__SDCC)
    stdout = STREAM_USART;              // Set stdout to serial stream
    #else
    baudUSART(BAUD_16_BIT_RATE &
              BAUD_IDLE_CLK_LOW &
              BAUD_WAKEUP_OFF &
              BAUD_AUTO_OFF & 0x48); // 16 bit BRG
    #endif

    #if defined (__SDCC)
    adc_open(ADC_CHN_0,                 // Set channel 0
             ADC_FOSC_16 | ADC_ACQT_8,  // Configure Acquisition frequency
             ADC_CFG_7A,                // Set the number of channels to use
             ADC_FRM_RJUST |            // Adjust result to the right
             ADC_INT_OFF |              // Disable interrupts
             ADC_VCFG_VDD_VSS |         // Configure reference voltages
             ADC_NVCFG_VSS |            // Negative reference to VSS
             ADC_PVCFG_VDD);            // Positive reference to VDD
    #else
    OpenADC(ADC_RIGHT_JUST & ADC_FOSC_16 & ADC_8_TAD,
            ADC_CH0 & ADC_INT_OFF,
            ADC_REF_VDD_VDD & ADC_REF_VDD_VSS,
            0b0000111111110000);
    #endif

    T0CONbits.T08BIT = 0;               // 16 bit mode
    T0CONbits.T0CS = 0;                 // Source is internal
    T0CONbits.PSA = 1;                  // Disable prescaler
    T0CONbits.T0PS = 7;                 // Prescaler to 1:256
    TMR0H = 0; TMR0L = 0;               // Reset Timer0 to 0x0000
    INTCONbits.TMR0IF = 0;              // Clear interrupt flag
    INTCONbits.TMR0IE = 1;              // Enable Interruption from timer0
    T0CONbits.TMR0ON = 1;               // Start timer0

    write_timer_0(Ticks4NextInterrupt);

    Servo.FL = 1;
    Servo.FR = 1;
    Servo.BL = 1;
    Servo.BR = 1;

    FL_SERVO_TRIS = 0;
    FR_SERVO_TRIS = 0;
    BL_SERVO_TRIS = 0;
    BR_SERVO_TRIS = 0;

    RCONbits.IPEN = 0;                  // Interruption Priority Disabled
    INTCONbits.PEIE = 1;                // Peripheral Interrupt Enabled
    INTCONbits.GIE = 1;                 // Global Interrupt Enable
}

void do_autonomous(void)
{
    if (!autonomous)
        return;

    get_sensors(); // Sensor sampling code
    if ( Sumo.atLeftEdge || Sumo.atRightEdge )
        Sumo.state = SURVIVE;

    switch ( Sumo.state )
    {
        case SURVIVE:
            if ( Sumo.atLeftEdge && Sumo.atRightEdge )
            {
                sumo_move(BACKWARD);
                delay10ktcy(250);
                delay10ktcy(250);
                //rotate(180);
                Sumo.searchClockwise = TRUE;
            }
            else if ( Sumo.atLeftEdge )
            {
                sumo_move(BACKWARD);
                delay10ktcy(250);
                delay10ktcy(250);
                //rotate(-135);
                //Add a Delay or create a function rotate(int degrees) instead
                Sumo.searchClockwise = TRUE;
            }
            else if ( Sumo.atRightEdge )
            {
                sumo_move(BACKWARD);
                delay10ktcy(250);
                delay10ktcy(250);
                //rotate(135);
                //Add a Delay or create a function rotate(int degrees) instead
                Sumo.searchClockwise = FALSE;
            }
            Sumo.state = HUNT;
            break;
        case HUNT:
            if ( rangeAverage > mTargetThreshold )
                Sumo.state = ATTACK;
            else if ( Sumo.searchClockwise )
                sumo_move(RIGHT);
                //arc(mHuntPower, mHuntFactor);
            else
                sumo_move(LEFT);
                //arc(mHuntPower, -mHuntFactor);
            break;
        case ATTACK:
            sumo_move(FORWARD);
            if ( rangeAverage < mTargetThreshold )
            {
                Sumo.state = HUNT;
            }
            break;
                default:
            Sumo.state = SURVIVE;
    }
}

void sumo_move ( char direction )
{
    switch ( direction )
    {
        case FORWARD:
            Servo_PWM[0] = Ticks4CCW;
            Servo_PWM[1] = Ticks4CW;
            Servo_PWM[2] = Ticks4CCW;
            Servo_PWM[3] = Ticks4CW;
            Servo.FL = Servo.FR = Servo.BL = Servo.BR = 1;
            break;
        case BACKWARD:
            Servo_PWM[0] = Ticks4CW;
            Servo_PWM[1] = Ticks4CCW;
            Servo_PWM[2] = Ticks4CW;
            Servo_PWM[3] = Ticks4CCW;
            Servo.FL = Servo.FR = Servo.BL = Servo.BR = 1;
            break;
        case LEFT:
            Servo_PWM[0] = Ticks4CW;
            Servo_PWM[1] = Ticks4CW;
            Servo_PWM[2] = Ticks4CW;
            Servo_PWM[3] = Ticks4CW;
            Servo.FL = Servo.FR = Servo.BL = Servo.BR = 1;
            break;
        case RIGHT:
            Servo_PWM[0] = Ticks4CCW;
            Servo_PWM[1] = Ticks4CCW;
            Servo_PWM[2] = Ticks4CCW;
            Servo_PWM[3] = Ticks4CCW;
            Servo.FL = Servo.FR = Servo.BL = Servo.BR = 1;
            break;
        case STOP:
            Servo.FL = Servo.FR = Servo.BL = Servo.BR = 0;
            break;
        default:
            Servo.FL = Servo.FR = Servo.BL = Servo.BR = 0;
            break;
    }
}

int read_a2d ( char channel ) // Read specified channel
{
    ADCON0 = (ADCON0 & 0b11000011) | ((channel << 2) & 0b00111100);
    ADCON0bits.GO = 1; // Start conversion
    while ( adc_busy() == 1 ); // While conversion is not finished
    return adc_read(); // Return 10 bit value
}

void get_sensors ( void )
{
    char i;
    mLeftLine = 0;
    mRightLine = 0;
    mCenterRange = 0;

    for ( i = 0; i < 8; i++ )
    {
        //mLeftLine += read_a2d(FL_SENSOR);
        //mRightLine += read_a2d(FR_SENSOR);
        mCenterRange += read_a2d(OL_SENSOR);
    }

    //mLeftLine >>= 3;
    //mRightLine >>= 3;
    mCenterRange >>= 3;

    //Sumo.atLeftEdge = (mLeftLine < mLeftEdgeThreshold);
    //Sumo.atRightEdge = (mRightLine < mRightEdgeThreshold);
    Sumo.atLeftEdge = 0;
    Sumo.atRightEdge = 0;

    rangeDifference = mCenterRange; // - mRightRange;
    rangeAverage = mCenterRange; // + mRightRange) >> 1;

    return;
}

#if defined (__SDCC)
static void ISRRx ( void ) __interrupt 1
#else
void interrupt ISRRx ( void )
#endif
{
    if ( INTCONbits.TMR0IF && INTCONbits.TMR0IE )
    {
        if ( !Servo.Phase )
        {
            if ( Servo_Idx == 0 && Servo.FL ) FL_SERVO = 1;
            if ( Servo_Idx == 1 && Servo.FR ) FR_SERVO = 1;
            if ( Servo_Idx == 2 && Servo.BL ) BL_SERVO = 1;
            if ( Servo_Idx == 3 && Servo.BR ) BR_SERVO = 1;
            Ticks4NextInterrupt = 65535 - Servo_PWM[Servo_Idx];
            write_timer_0(Ticks4NextInterrupt);
        }
        else
        {
            if ( Servo_Idx == 0 && Servo.FL ) FL_SERVO = 0;
            if ( Servo_Idx == 1 && Servo.FR ) FR_SERVO = 0;
            if ( Servo_Idx == 2 && Servo.BL ) BL_SERVO = 0;
            if ( Servo_Idx == 3 && Servo.BR ) BR_SERVO = 0;
            Ticks4NextInterrupt = 65535 - Ticks4Window + Servo_PWM[Servo_Idx];
            write_timer_0(Ticks4NextInterrupt);
            if ( ++Servo_Idx > 3 ) Servo_Idx = 0;
        }
        Servo.Phase = !Servo.Phase;
        INTCONbits.TMR0IF = 0;
    }
    if ( PIE1bits.RCIE && PIR1bits.RCIF )
    {
        char_recv = usart_getc();
        //usart_putc(char_recv);
        PIR1bits.RCIF = 0;
    }
}

void process_serial(void)
{
    char action = STOP;
    if (char_recv != last_char_recv){
        switch(char_recv){
        case 'F':
            action = FORWARD;
            break;
        case 'B':
            action = BACKWARD;
            break;
        case 'L':
            action = LEFT;
            break;
        case 'R':
            action = RIGHT;
            break;
        case 'S':
            action = STOP;
            break;
        case 'X':  //Extra ON
            autonomous = FALSE; // Turn autonomous mode OFF
            action = STOP;
            break;
        case 'x':  //Extra OFF
            autonomous = TRUE; // Turn autonomous mode ON
            break;
        }
        if(!autonomous)     // Only process commands in non-autonomous mode
            sumo_move(action);
    }
}

void wait_5s ( void )
{
    int t;
    for ( t = 0; t < 9; t++ )
    {
        delay10ktcy(225);
    }
    return;
}

void putch ( char data )
{
    while ( usart_busy() )
        continue;
    usart_putc(data);
}

void write_timer_0(unsigned int value){
    TMR0H = (value >> 8) & 0xFF;
    TMR0L = value & 0xFF;
}
