

// Modified by Bereket Kebede
// Dextrous Robotics
// Normal ATP 328 Bootloader

#include <SPI.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal_I2C.h>


// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

//Set this to what pin your "INT0" hardware interrupt feature is on
#define Motion_Interrupt_Pin 9

const int ncs = 10;  //This is the SPI "slave select" pin that the sensor is hooked up to
int16_t x = 0; // for sensor.motion burst
int16_t y = 0;

///////////////////////////////////////
// Define variables

byte initComplete=0;
volatile float total =0;
volatile int16_t xydat[4];



volatile byte movementflag=0;
byte testctr=0;
unsigned long currTime;
unsigned long timer;
unsigned long pollTimer;
unsigned long StartTime;
unsigned long CurrentTime;
unsigned long ElapsedTime;
int count_freq;

//Be sure to add the SROM file into this sketch via "Sketch->Add File"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

// Define SDA and SCL pin for LCD:
#define SDAPin A4 // Data pin
#define SCLPin A5 // Clock pin
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered):
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,20,4); //Change to (0x27,16,2) for 1602 LCD


void setup() {
  Serial.begin(9600);
  
  pinMode (ncs, OUTPUT);
  
  pinMode(Motion_Interrupt_Pin, INPUT);
  digitalWrite(Motion_Interrupt_Pin, HIGH);
  attachInterrupt(9, UpdatePointer, FALLING);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  //SPI.setClockDivider(4);
  performStartup();  
  //delay(5000);
  dispRegisters();
  initComplete=9;
  StartTime = millis();
  lcd.init();
  lcd.backlight();
  // Keyboard.begin();

}

void pmw_com_begin(){
  digitalWrite(ncs, LOW);
}

void pmw_com_end(){
  digitalWrite(ncs, HIGH);
}

byte pmw_read_reg(byte reg_addr){
  pmw_com_begin();
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  pmw_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void pmw_write_reg(byte reg_addr, byte data){
  pmw_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  pmw_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void pmw_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");

  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  pmw_write_reg(Config2, 0x20);
  
  // write 0x1d in SROM_enable reg for initializing
  pmw_write_reg(SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  pmw_write_reg(SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  pmw_com_begin();
  SPI.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  pmw_read_reg(SROM_ID);

  //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
  pmw_write_reg(Config2, 0x00);

  // set initial CPI resolution
  pmw_write_reg(Config1, 0x15);
  
  pmw_com_end();

  }


void performStartup(void){
  pmw_com_end(); // ensure that the serial port is reset
  pmw_com_begin(); // ensure that the serial port is reset
  pmw_com_end(); // ensure that the serial port is reset
  pmw_write_reg(Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  pmw_read_reg(Motion);
  pmw_read_reg(Delta_X_L);
  pmw_read_reg(Delta_X_H);
  pmw_read_reg(Delta_Y_L);
  pmw_read_reg(Delta_Y_H);
  // upload the firmware
  pmw_upload_firmware();
  delay(10);
  Serial.println("Optical Chip Initialized");
  }

void UpdatePointer(void){
  if(initComplete==9){

    //write 0x01 to Motion register and read from it to freeze the motion values and make them available
    pmw_write_reg(Motion, 0x01);
    pmw_read_reg(Motion);
    // xydat[0] = (int)pmw_read_reg(Delta_X_L);
    // xydat[1] = (int)pmw_read_reg(Delta_X_H);
    int16_t DELTA_X = pmw_read_reg(Delta_X_L) + (int16_t)(pmw_read_reg(Delta_X_H) << 8);
    int16_t DELTA_Y = pmw_read_reg(Delta_Y_L) + (int16_t)(pmw_read_reg(Delta_Y_H) << 8); 
    xydat[0] = DELTA_X;
    xydat[1] = DELTA_Y;
    
    movementflag=1;
    }
  }

void dispRegisters(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  digitalWrite(ncs,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs,HIGH);
}


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }


float direction(int8_t dx, int8_t dy){
 // RAD2DEG = angle * 180 / PI;
 float angle = atan2(dy, dx) * 180 / PI;
 return angle;
}


int8_t mag(int8_t dx, int8_t dy){
 int8_t d_avg = sqrt (pow (dx, 2) + pow (dy, 2));
 return d_avg;
}

//unsigned long clocktime;

//measure total length
// float integrate_mag(float length){
//   float total += length
//   return total;
// }


void loop() {

  //////////////////////////////////////////
  // option 1
  // currTime = millis();
  
  // if(currTime > timer){    
  //   Serial.println(testctr++);
  //   timer = currTime + 2000;
  //   }
  // UpdatePointer();
  // xydat[0] = convTwosComp(xydat[0]);
  // xydat[1] = convTwosComp(xydat[1]);
  // Serial.print("x = ");
  // Serial.print(xydat[0]);
  // Serial.print(" mm ");
  // Serial.print(" | ");
  // Serial.print("y = ");
  // Serial.print(xydat[1]);
  // Serial.println(" mm ");
  // delay(100);

   //////////////////////////////////////////
  // option 2, velocity
  // dv_x = dx*df
  // currTime = millis();
  
  // // if(currTime > timer){    
  // //   Serial.println(testctr++);
  // //   timer = currTime + 2000;
  // //   }

  // UpdatePointer();
  // xydat[0] = convTwosComp(xydat[0]);
  // xydat[1] = convTwosComp(xydat[1]);
  // Serial.print("vx = ");
  // Serial.print(xydat[0]*0.2);
  // Serial.print(" mmps ");
  // Serial.print(" | ");
  // Serial.print("vy = ");
  // Serial.print(xydat[1]*0.2);
  // Serial.println(" mmps ");
  // //delay(100);


  /////////////////////////////////////////
  // options 2

  // UpdatePointer();
  // float dx = convTwosComp(xydat[0]);
  // float dy = convTwosComp(xydat[1]);

  // // xydat[0] = convTwosComp(xydat[0]);
  // // xydat[1] = convTwosComp(xydat[1]);


  // int average = mag(dx,dy);
  // float my_angle = direction(dx,dy);
  
  // Serial.print("average:");
  // Serial.println(average);
  // Serial.print("|");
  // Serial.print("angle:");
  // Serial.println(my_angle);
  // delay(50);



  //////////////////////////
  //option 3

  //clocktime = micros();
  //Serial.println(clocktime);


  // original 
  // if(currTime > pollTimer){
  //   UpdatePointer();
  //   xydat[0] = convTwosComp(xydat[0]);
  //   xydat[1] = convTwosComp(xydat[1]);
  //     if(xydat[0] != 0 || xydat[1] != 0){
  //       Serial.print("x = ");
  //       Serial.print(xydat[0]);
  //       Serial.print(" | ");
  //       Serial.print("y = ");
  //       Serial.println(xydat[1]);
  //       }
  //   pollTimer = currTime + 20;
  //   }

  ////////////////////////////////////////
  // option 4, measure frequency
  // CurrentTime = millis();
  // ElapsedTime = CurrentTime - StartTime;

  // if (ElapsedTime < 1000){
  //   count_freq++;
  //   Serial.println(ElapsedTime);
  // }
  // else {
  //   // quit
  //   Serial.print("frequency is: ");
  //   Serial.print(count_freq);
  //   Serial.println(" Hz");
    
  // }

  // delay(100);

  //////////////////////////////////////////
  // option 5, measure total distance

  // UpdatePointer();
  // float dx = convTwosComp(xydat[0]);
  // float dy = convTwosComp(xydat[1]);

  // // xydat[0] = convTwosComp(xydat[0]);
  // // xydat[1] = convTwosComp(xydat[1]);


  // int average = mag(dx,dy);
  // float my_angle = direction(dx,dy);
  // float total = total + average;
  
  // Serial.print("total:");
  // Serial.print(total);
  // Serial.print("|");
  // Serial.print("angle:");
  // Serial.println(my_angle);
  // delay(50);

  ///////////////////////////////////////
  // option 6

   UpdatePointer();
  float dx = convTwosComp(xydat[0]);
  float dy = convTwosComp(xydat[1]);
  int average = mag(dx,dy);
  float my_angle = direction(dx,dy);
  float total = total + average;

  // if (Keyboard.press == 'n')
  // else{
  x += dx;
  y += dy;  
  
  
  

  Serial.print("total_x: ");
  Serial.print(x);
  Serial.print("|");
  Serial.print(" total_y: ");
  Serial.println(y);

  lcd.setCursor(0,0); // Set the cursor to column 1, line 1 (counting starts at zero)
  lcd.print("x:"); // Prints string "Display = " on the LCD
  lcd.print(dx); // Prints the measured distance
  lcd.print(" | y:"); // Prints string "Display = " on the LCD
  lcd.print(dy); // Prints the measured distance

  lcd.setCursor(0,1); // Set the cursor to column 1, line 1 (counting starts at zero)
  lcd.print("speed: ");
  lcd.print(dx*0.0059); // Prints string "Display = " on the LCD


  //delay(100);

    
  }

