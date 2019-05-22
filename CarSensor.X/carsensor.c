// PIC18F4620 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define _XTAL_FREQ   4000000UL 
#define LCD_TYPE 2

#define LCD_LINE_TWO 0x40    // LCD RAM address for the second line


//unsigned char  LCD_INIT_STRING[4] = {0x20 | (LCD_TYPE << 2), 0xc, 1, 6};

struct lcd_pin_map {
    // This structure is overlayed

    unsigned un1 : 1; //unused on to an I/O port to gain, should be cleared
    unsigned rs : 1; // rd1 access to the LCD pins.          
    unsigned rw : 1; // low order up.  ENABLE will
    unsigned enable : 1; // The bits are allocated from
    unsigned data : 4; // be pins d0:,1,2,3

} lcd @ 0x0F83; //   ; PORTD

#define lcd_output_enable(x) lcd.enable = x
#define lcd_output_rs(x) lcd.rs = x
void delay_cycles(unsigned char n);
void delay_ms(unsigned char n);
void lcd_send_nibble(unsigned char n);
void lcd_send_byte(unsigned char cm_data, unsigned char n);
void lcd_init();
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_putc(char c);
void lcd_puts(char *s);
void setupPorts();
void set_pwm1_voltage(float value); // value 0--5V, ;
void init_pwm1(); //10 bit accurcy
void set_pwm1_raw(unsigned int raw_value);
void set_pwm2_voltage(float value); // value 0--5V, 
void set_pwm2_raw(unsigned int raw_value);
#include <xc.h>
#include<stdio.h>

void main(void) {
    setupPorts();
    init_pwm1();
    lcd_puts((char *) "This is done by noor and omar");
    ClrWdt();
    delay_ms(10000);
    lcd_gotoxy(1, 1);
    unsigned int time = 0;
    float distance;
    char buff[32];
    lcd_puts((char *) "                               ");
    sprintf(buff, "Distance is : ");
    lcd_puts(buff);
    while (1) {
        ClrWdt();
        T1CON = 0x30; // pre-scalar 8 off , RD16 = 0, TMRON = 0
        TMR1H = 0;
        TMR1L = 0;
        PORTDbits.RD0 = 0;
        delay_ms(5);
        PORTDbits.RD0 = 1;
        delay_ms(10);
        PORTDbits.RD0 = 0;
        while (!PORTBbits.RB0);
        T1CONbits.TMR1ON = 1;
        while (PORTBbits.RB0);
        T1CONbits.TMR1ON = 0;
        time = (TMR1H << 8) | TMR1L;
        distance = ((float) (8 * time) / 1000000.0) / 58.82;
        sprintf(buff, "  %6.2f", distance);
        lcd_gotoxy(1, 2);
        lcd_puts(buff);
        if (distance >= 300) {//pm1 for greeen  // pmw2 for red;
            set_pwm1_voltage(4);
            set_pwm2_voltage(0.5);
        } else if (distance >= 200) {
            set_pwm1_voltage(4);
            set_pwm2_voltage(1.5);
        } else if (distance >= 100) {
            set_pwm1_voltage(2.5);
            set_pwm2_voltage(3);
        } else if (distance >= 75) {
            set_pwm1_voltage(0.5);
            set_pwm2_voltage(3);
        } else if (distance >= 50) {
            set_pwm1_voltage(0.5);
            set_pwm2_voltage(4);
        } else {
            set_pwm1_voltage(0.05);
            set_pwm2_voltage(4.8);

        }

        return;
    }
}

void set_pwm1_voltage(float value) // value 0--5V, 
{
    float tmp = value * 1023.0 / 5.0;
    int raw_val = (int) (tmp + 0.5);
    if (raw_val > 1023) raw_val = 1023;
    set_pwm1_raw(raw_val);

}

void set_pwm2_voltage(float value)// value 0--5V, 
{
    float tmp = value * 1023.0 / 5.0;
    int raw_val = (int) (tmp + 0.5);
    if (raw_val > 1023) raw_val = 1023;
    set_pwm2_raw(raw_val);

}

void init_pwm1()//10 bit accurcy
{
    PR2 = 255;
    T2CON = 0;
    CCP1CON = 0x0C;
    CCP2CON = 0x0C;
    T2CONbits.TMR2ON = 1;
    TRISCbits.RC2 = 0;
    TRISCbits.RC1 = 0;
}

void set_pwm1_raw(unsigned int raw_value)//raw value 0 -- 1023
{
    CCPR1L = (raw_value >> 2) & 0x00FF;
    CCP1CONbits.DC1B = raw_value & 0x0003; //first two bits
}

void set_pwm2_raw(unsigned int raw_value)//raw value 0 -- 1023
{
    CCPR2L = (raw_value >> 2) & 0x00FF;
    CCP2CONbits.DC2B = raw_value & 0x0003; //first two bits
}

void setupPorts() {
    ADCON0 = 0;
    ADCON1 = 0x0C;
    TRISB = 0xFF;
    TRISC = 0xBE;
    TRISA = 0xFF;
    TRISD = 0x00;
    TRISE = 0x07;
    //setupSerial();
    //init_adc_no_lib();
    //initInt();
    lcd_init();
    return;
}
unsigned char LCD_INIT_STRING[4] = {0x20 | (LCD_TYPE << 2), 0xc, 1, 6};
//unsigned char  LCD_INIT_STRING[4] = {0x28, 0xc, 1, 6};

//#define lcd_output_enable(x) lcd.enable =x
//#define lcd_output_rs(x) lcd.rs =x

void delay_ms(unsigned char n) {
    int x;
    for (x = 0; x <= n; x++) {
        __delaywdt_ms(1);
        // __delay_ms(1);   // 20x10 200ms
    }
}

void delay_cycles(unsigned char n) {
    int x;
    for (x = 0; x <= n; x++) {
        CLRWDT(); //NOP();   // 20x10 200ms
    }

}
//void delay_us ()

void lcd_send_nibble(unsigned char n) {

    lcd.data = n;

    delay_cycles(1); //here 1111
    lcd_output_enable(1);
    __delaywdt_us(20); //delay_us(2);
    lcd_output_enable(0);
}

void lcd_send_byte(unsigned char cm_data, unsigned char n) {

    //  lcd_output_rs(0);
    //  delay_ms(1);     ////while ( bit_test(lcd_read_byte(),7) ) ;
    lcd_output_rs(cm_data);
    delay_cycles(1);
    // lcd_output_rw(0);
    delay_cycles(1);
    lcd_output_enable(0);
    lcd_send_nibble(n >> 4);
    lcd_send_nibble(n & 0x0f);

    delay_ms(2); //added by raed
    // lcd_output_rs(0);//added 
}

void lcd_init() {

    unsigned char i;


    lcd_output_rs(0);
    //lcd_output_rw(0); 
    lcd_output_enable(0);

    delay_ms(25); //   
    for (i = 1; i <= 3; ++i) {
        lcd_send_nibble(3);
        // lcd_send_nibble(0);
        delay_ms(6); //5
    }

    lcd_send_nibble(2);
    //  lcd_send_nibble(0);
    for (i = 0; i <= 3; ++i)
        lcd_send_byte(0, LCD_INIT_STRING[i]);
}

void lcd_gotoxy(unsigned char x, unsigned char y) {
    unsigned char address;

    if (y != 1)
        address = LCD_LINE_TWO;
    else
        address = 0;
    address += x - 1;
    lcd_send_byte(0, (unsigned char) (0x80 | address));
}

void lcd_putc(char c) {
    switch (c) {
        case '\f': lcd_send_byte(0, 1);
            delay_ms(2);
            break;
        case '\n': lcd_gotoxy(1, 2);
            break;
        case '\b': lcd_send_byte(0, 0x10);
            break;
        default: lcd_send_byte(1, c);
            break;
    }
}

void lcd_puts(char *s) {
    while (*s) {
        lcd_putc(*s);
        s++;
    }
}

