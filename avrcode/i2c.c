// I2C sensor, gyroscope, magnetometer, and 3-axis accelerometer
// Minimu 9 module: http://www.pololu.com/catalog/product/1265
// L3G4200D gyro sensor and LSM303DLM accelerometer+magnetometer

#define F_CPU 14745600
#define PI 3.14159265359

#include <stdio.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <util/delay.h>
#include "../libnerdkits/io_328p.h"
#include "../libnerdkits/delay.h"
#include "../libnerdkits/lcd.h"
#include "../libnerdkits/uart.h"

#define GYRADDR 0b1101001 // Gyrosensor address
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define ACCADDR 0b0011000 // Accelerometer address 0011000b
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24

#define MAGADDR 0b0011110 // Magnetometer address 0011110b
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Z_H_M 0x05
#define OUT_Z_L_M 0x06
#define OUT_Y_H_M 0x07
#define OUT_Y_L_M 0x08

int16_t timer; // timer = tcnt1
double time_seconds; // Timer in seconds,

int16_t out_x, out_y, out_z, out_x_h,out_y_h,out_z_h,out_x_l,out_y_l,out_z_l; // Gyroscope output
double out_x_a, out_y_a, out_z_a; // Accelerometer output
//int16_t out_x_m, out_y_m, out_z_m;
double dps_x,dps_y;
double fx2,fx1,fy2,fy1;
double degrees_tilted_gyr, degrees_tilted_acc, angle; // Degrees tilted by gyroscope, accelerometer, and combined
double degrees_tilted_gyr_2, degrees_tilted_acc_2, angle_2; // Degrees tilted by gyroscope, accelerometer, and combined on x-axis


// setups TWI (Two Wire Interface A.K.A I2C)
void TWISetup(){

    // Set up TWI module to 368,640 Hz
    // SCLfreq = Fclock/(16+2*TWBR*(prescaler))

    // Set up TWI but rate to 3
    TWBR = 3;
    // Prescaler to 4
    TWSR |= (1<<TWPS0);
    // Enable TWI
    TWCR |= (1<<TWEN);

}

// Start signal function
void TWIStart(){

    // From datasheet page 226
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); // Send start condition

    while (!(TWCR & (1<<TWINT))); // Wait for TWINT Flag set. This indicates that the START condition has been transmitted

}

void TWIStop(){

    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); // Transmit STOP condition
}

void TWIWrite(uint8_t data){

    //Load SLA_W into TWDR Register. Clear TWINT bit in
    //TWCR to start transmission of address
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);

    //Wait for TWINT Flag set. This indicates that the SLA+W has
    //been transmitted, and ACK/NACK has been received.
    while (!(TWCR & (1<<TWINT)));
}

// Read with ACK (Acknowledge bit)
uint8_t TWIReadACK(){

    //TWI Interrupt Flag, TWI Enable Bit, TWI Enable Acknowledge Bit
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    //Wait for TWINT Flag set.
    while (!(TWCR & (1<<TWINT)));
    return TWDR;

}

//read byte with NACK
uint8_t TWIReadNACK()
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    //Wait for TWINT Flag set.
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}
// Check status
uint8_t TWIReadStatus(){

    uint8_t status;
    status = TWSR & 0xF8;

    //Check value of TWI Status Register. Mask prescaler bits. If
    //status different from START go to ERROR
    /*
    if ((TWSR & 0xF8) != START)
    ERROR();
    */

    return status;

}

// Read byte from L3G4200D gyro sensor and LSM303DLM accelerometer
int16_t ReadByte(uint8_t DEVADDR,uint8_t regAddress){
    //printf_P(PSTR("\r\n"), TWIReadStatus());
  //send start
    TWIStart(); // ST

    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus());
    //select device,send read bit and send info to an specific register address
    TWIWrite((DEVADDR<<1)|0); // SAD+W (Slave address) + Write bit

    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus());
    TWIWrite(regAddress); // Register adddress (SUB)
   //send repeated start
    TWIStart(); // SR
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus());
    TWIWrite((DEVADDR<<1)|1); // SAD + R
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus());
    int16_t data = TWIReadNACK(); // NMAK
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus());

    TWIStop(); // SP
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus());
    //return data;
    return data;
    //printf_P(PSTR("Bit received: 0x%x\r\n"), data ); //send info to serial port
}

// Writes one byte to L3G4200D gyro sensor (REGISTER ADDR, DATA)
void writeByte(uint8_t DEVADDR,uint8_t regAddress, uint8_t data){

    TWIStart(); // Starts
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus()); // Check current status
    TWIWrite((DEVADDR<<1)|0); // SAD+W (Slave address) + Write bit
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus()); // Check current status
    TWIWrite(regAddress); // Register adddress (SUB)
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus()); // Check current status
    TWIWrite(data); // Data to write
    //printf_P(PSTR("Current Status: 0x%x\r\n"), TWIReadStatus()); // Check current status
    TWIStop(); // Stops I2C
    //printf_P(PSTR("Byte written to device.\r\n")); // Check current status


}

void timer_init()
{
    // set up timer with a prescaling of 1024
    TCCR1B |= (1<<CS12)|(1<<CS10);
    // initialize counter
    TCNT1 = 0; // 16-bit timer
}

void measure_time(){
    timer = TCNT1;
    time_seconds = timer/(F_CPU/(1024.00)); // 1024 = Prescaler
    TCNT1 = 0;
}

void gyro_tilt(){
    measure_time(); // Measures time between each output
    out_x_h = ReadByte(GYRADDR,OUT_X_H);
    out_y_h = ReadByte(GYRADDR,OUT_Y_H);
    out_z_h = ReadByte(GYRADDR,OUT_Z_H);

    out_x_l = ReadByte(GYRADDR,OUT_X_L);
    out_y_l = ReadByte(GYRADDR,OUT_Y_L);
    out_z_l = ReadByte(GYRADDR,OUT_Z_L);
    //TCNT1 = 0;
    fx1 = dps_x; // Previous reading of dps
    fy1 = dps_y;
    // If N_H is either 255 (negative values) or 0, then the gyro is most likely not moving (noise might cause the +-1 offset)
    // So give out_N a value of zero until it moves, and then combines the two bytes
    if( out_x_h != 255 && out_x_h != 0 ){ // out_x
        out_x = out_x_l | (out_x_h<<8);
    }
    else{
        out_x = 0;
        degrees_tilted_gyr = 0; // Restarts to zero if no rotating in x-axis detected
    }
    // out_y
    if( out_y_h != 255 && out_y_h != 0 ){
        out_y = out_y_l | (out_y_h<<8);
    }
    else{
        out_y = 0;
        degrees_tilted_gyr_2 = 0; // Restarts to zero if no rotating in y-axis detected
    }
    // out_x
    if( out_z_h != 255 && out_z_h != 0 ){
        out_z = out_z_l | (out_z_h<<8);
    }
    else{
        out_z = 0;
    }

    /* Integration */
    dps_x = (8.75/1000)*(double)out_x;
    dps_y = (8.75/1000)*(double)out_y;
    fx2 = dps_x;
    fy2 = dps_y;
    if( fx2 != 0 && fx1 != 0 ) // Do not take "area" when the trapezoid has one base = 0, (previous reading was 0)
    degrees_tilted_gyr = (fx1+fx2)*time_seconds/2; // Trapezoidal rule, integrates with respect to time
    if( fy2 != 0 && fy1 != 0 ) // Do not take "area" when the trapezoid has one base = 0, (previous reading was 0)
    degrees_tilted_gyr_2 = (fy1+fy2)*time_seconds/2; // Trapezoidal rule, integrates with respect to time

}
// Complementary filter
void complem_filt(){
    /* Since tan(90) is undefined, and when angle surpasses 90, or -90 it changes sign the following 2 lines of code will avoid
       having a really noisy angle readings because of that sign change. */
    //if( angle < 0 && degrees_tilted_acc > 0 ){ degrees_tilted_acc *=-1; }
    //if( angle > 0 && degrees_tilted_acc < 0 ){ degrees_tilted_acc *=-1; }
    angle = (0.95)*(angle + degrees_tilted_gyr) + (0.05)*degrees_tilted_acc; // Complementary digital filter
    //if( angle >= 90 ) angle = 90; // Keep interval from -90 to 90 degrees
    //if( angle <= -90 ) angle = -90; // Keep interval from -90 to 90 degrees

    // X-axis rotational tilt
    //if( angle_2 < 0 && degrees_tilted_acc_2 > 0 ){ degrees_tilted_acc_2 *=-1; }
    //if( angle_2 > 0 && degrees_tilted_acc_2 < 0 ){ degrees_tilted_acc_2 *=-1; }
    angle_2 = (0.95)*(angle_2 + degrees_tilted_gyr_2) + (0.05)*degrees_tilted_acc_2; // Complementary digital filter
    //if( angle_2 >= 90 ) angle_2 = 90; // Keep interval from -90 to 90 degrees
    //if( angle_2 <= -90 ) angle_2 = -90; // Keep interval from -90 to 90 degrees
}

int main() {

// PB1 as input, for pairing
DDRB &= ~(1<<PB1); // set PB1 as input (button)
PORTB |= (1<<PB1); //Internal resistor


// init serial port
uart_init();
FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
stdin = stdout = &uart_stream;

TWISetup();
//pinci_setup(); // Pin change interrupt for syncing mode...

// Enable x, y, z and turn off power down: (from datasheet)
// Gyrosensor (100Hz)
writeByte(GYRADDR,CTRL_REG1, 0b00001111); // |DR1|DR0|BW1|BW0|PD|Zen|Yen|Xen| ... Xen, Yen, Zen, and PD must be 1
writeByte(GYRADDR,CTRL_REG4, 0b00000000); // |BDU|BLE|FS1|FS0|-|ST1|ST0|SIM| ... (sensitivty) Fs = 250 dps, unit = 8.75 mdps/digit

// Accelerometer
writeByte(ACCADDR,CTRL_REG1_A, 0b00101111); // |PM2|PM1|PM0|DR1|DR0|Zen|Yen|Xen|....Xen, Yen, Zen activates X,Y, and Z. Normal mode, output data rate:50Hz
writeByte(ACCADDR,CTRL_REG2_A, 0b00000000); // |BOOT|HPM1|HPM0|FDS|HPen2|HPen1|HPCF1|HPCF0|...High pass filter enabled
writeByte(ACCADDR,CTRL_REG4_A, 0b00110000); // |BDU|BLE|FS1|FS0|0|0|0|---| ... full scale selection

timer_init();

//Bluetooth AT mode
//DDRB |= (1<<PB2);
//PORTB |= (1<<PB2);

// Tmp values, stores old values
double tmp1,tmp2,tmp3; // Just send different (new) data to serial port

 while(1){

    /* Gyrosensor output */
    gyro_tilt(); // Measures tilt using gyro sensor

    // Stores old data of acceleremeter, determines that new incoming data is being sent
    tmp1 = out_x_a;
    tmp2 = out_y_a;
    tmp3 = out_z_a;

    /* Accelerometer output */
    // 12-Bit left justfiid, disregards the last 4 bits because they're meaningless
    out_x_a = (ReadByte(ACCADDR,OUT_X_L_A) | (ReadByte(ACCADDR,OUT_X_H_A)<<8))>>4;
    out_y_a = (ReadByte(ACCADDR,OUT_Y_L_A) | (ReadByte(ACCADDR,OUT_Y_H_A)<<8))>>4;
    out_z_a = (ReadByte(ACCADDR,OUT_Z_L_A) | (ReadByte(ACCADDR,OUT_Z_H_A)<<8))>>4;

    // Converts to g's. Output is in 1mg/digit, so converts to g force
    out_x_a *= 3.9/1000.00; // 1 mg/digit
    out_y_a *= 3.9/1000.00; // 1 mg/digit
    out_z_a *= 3.9/1000.00; // 1 mg/digit

    if( out_y_a != 0.0 && out_x_a != 0.0){
    //degrees_tilted_acc = atan(out_y_a/out_z_a)*180/PI; // Tilt angle from accelerometer
    //degrees_tilted_acc_2 = atan(out_x_a/out_z_a)*180/PI; // Tilt angle from accelerometer
    degrees_tilted_acc = -atan(out_z_a/out_y_a)*180/PI; // Tilt angle from accelerometer, keep in mind that it's already tilted 90 degrees with respect to horizon
    degrees_tilted_acc_2 = -atan(out_z_a/out_x_a)*180/PI; // Tilt angle from accelerometer, ankle inclination 90 degrees, so makes it negative
    //if( degrees_tilted_acc >= 90 ) degrees_tilted_acc = 90.00;
    //if( degrees_tilted_acc <= -90 ) degrees_tilted_acc = -90.00;
    //if( degrees_tilted_acc_2 >= 90 ) degrees_tilted_acc_2 = 90.00;
    //if( degrees_tilted_acc_2 <= -90 ) degrees_tilted_acc_2 = -90.00;

    }
    complem_filt(); // Complementary filter

    //received = uart_read();
    // Send time through serial, for "deltatime"
    if( tmp1 != out_x_a || tmp2 != out_y_a || tmp3 != out_z_a ) // Send only new data of the accelerometer
    printf_P(PSTR("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\r\n"),out_x_a,out_y_a,out_z_a,angle,angle_2,time_seconds); // "#" Means end of line, "*" is where line begins
    //printf_P(PSTR("*%.3f,%.3f,%.3f#\r\n"),out_x_a,out_y_a,out_z_a);
    //printf_P(PSTR("%c\r\n"),received);
    //printf_P(PSTR("*%.3f,%.3f,%.3f#\r\n"),degrees_tilted_gyr,degrees_tilted_acc,angle);
    //delay_ms(1); // Makes tranmission more stable

}
 return 0;
}
