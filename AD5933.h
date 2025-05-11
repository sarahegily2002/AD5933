/*
 * AD5933.h
 *
 * Created: 2/11/2019 12:09:37 AM
 *  Author: MOHAMED
 * Modified for Arduino compatibility
 */

#ifndef AD5933_H_
#define AD5933_H_

#include <Wire.h>
#include <math.h>

// AD5933 I2C Address
#define AD5933_ADDR 0x0D

//AD5933 control codes
#define Init 0x10        //Initialize with start Freq
#define Sweep 0x20       //Start Frequency Sweep
#define IncFreq 0x30     //Increment Frequency
#define RepFreq 0x40     //Repeat Frequency
#define MeaTemp 0x90     //Measure Temperature
#define PowerDown 0xa0   //Power down mode
#define Standby 0xb0     //Standby mode
#define Range2V 0x00     //Output Voltage range 2V
#define Range1V 0x06     //Output Voltage range 1V
#define Range400mV 0x04  //Output Voltage range 400mV
#define Range200mV 0x02  //Output Voltage range 200mV
#define gainx5 0x00      //PGA gain x5
#define gainx1 0x01      //PGA gain x1

//AD5933 Register addresses
#define Control_high 0x80
#define Control_low 0x81
#define Freq_high 0x82
#define Freq_mid 0x83
#define Freq_low 0x84
#define FreqInc_high 0x85
#define FreqInc_mid 0x86
#define FreqInc_low 0x87
#define NumInc_high 0x88
#define NumInc_low 0x89
#define NumSettle_high 0x8a
#define NumSettle_low 0x8b
#define Status 0x8f
#define Temp_high 0x92
#define Temp_low 0x93
#define Real_high 0x94
#define Real_low 0x95
#define Imag_high 0x96
#define Imag_low 0x97

// #define scale 100000000
#define scale 1.0
#define numofinc 60
#define radtodeg 52.27
#define cal_res 1000

class AD5933 {
public:
  int8_t a = 0x00, b = 0x00;
  double R = 0x00000000, I = 0x00000000;
  int phase = 0, system_phase = 0;
  unsigned long int Z = 0, Result = 0;
  double GF = 1.0;
  uint64_t sumofresult = 0, sumofphase = 0;
  uint8_t i = 0;
  int T = 0;
  int gluco = 5;
  double InverseGF=1;

  // Initialize the AD5933
  void begin() {
    Wire.begin();
  }

  // Write a byte to the AD5933 register
  void Byte_write(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(AD5933_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
    delay(1);  // Small delay for stability
  }

  // Read a byte from the AD5933 register
  uint8_t Byte_read(uint8_t reg) {
    uint8_t data;

    Wire.beginTransmission(AD5933_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)AD5933_ADDR, (uint8_t)1);
    if (Wire.available()) {
      data = Wire.read();
    }

    return data;
  }

  // Process data from two bytes into a single value
  unsigned long int Data_proc(unsigned char data_high, unsigned char data_low) {
    //concatenation
    unsigned long int ddata;
    ddata = (unsigned long int)data_high * 256 + data_low;
    if (ddata > 0x7fff) {
      ddata = 0x10000 - ddata;
    }
    return ddata;
  }

  // Calibration function
  void calib(void) {
    //start frequency register - 10 kHz for internal oscillator 16.776Mhz
    Byte_write(Freq_low, 0x45);  //30k >>>0x0EA645    //70k >>>>0x222EA2    //55k>>>>>1760127>0x1adb7f
    Byte_write(Freq_mid, 0xA6);
    Byte_write(Freq_high, 0x0E);
    //number of increments 10
    Byte_write(NumInc_low, numofinc);  //notice that the number of increments is defined as numofinc in preprocessor
    Byte_write(NumInc_high, 0x00);
    //frequency increment register - 1 kHz 0x008312
    Byte_write(FreqInc_low, 0x00);
    Byte_write(FreqInc_mid, 0x00);
    Byte_write(FreqInc_high, 0x00);

    //settling time cycles register - 15
    Byte_write(NumSettle_low, 0x0F);
    Byte_write(NumSettle_high, 0x01);

    //CONTROL register
    //AD5933 in standby mode
    Byte_write(Control_high, 0xB1);  //2v-p2p        PGA gain; 1 = ×1
    Byte_write(Control_low, 0x00);   //internal clock

    //initialize with start frequency
    Byte_write(Control_high, 0x11);
    Byte_write(Control_low, 0x00);

    //start frequency sweep
    Byte_write(Control_high, 0x21);
    Byte_write(Control_low, 0x00);

    //measure temperature
    Byte_write(Control_high, 0x91);
    Byte_write(Control_low, 0x00);

    while (!(Byte_read(Status) & 0x01)) {
      digitalWrite(13, HIGH);  // Use built-in LED instead of PORTB
    }                          //valid temp value
    while (!(Byte_read(Status) & 0x02))
      ;

    a = 0x00;
    b = 0x00;
    //temp data
    a = Byte_read(Temp_high);
    b = Byte_read(Temp_low);
    T = Data_proc(a, b) / 32;

    a = 0x00;
    b = 0x00;
    //real data
    a = Byte_read(Real_high);
    b = Byte_read(Real_low);
    R = Data_proc(a, b);
    a = 0x00;
    b = 0x00;
    //Imag data
    a = Byte_read(Imag_high);
    b = Byte_read(Imag_low);
    I = Data_proc(a, b);
    Z = sqrt(R * R + I * I);
    GF = 0.5 * (1.0*scale / (Z * cal_res));
    //  GF = (scale / (Z * cal_res));
    R = 0;
    I = 0;
    Z = 0;

    //step two to find gain factor

    //start frequency register - 10 kHz for internal oscillator 16.776Mhz
    Byte_write(Freq_low, 0x45);  //30k >>>0x0EA645    //70k >>>>0x222EA2    //55k>>>>>1760127>0x1adb7f
    Byte_write(Freq_mid, 0xA6);
    Byte_write(Freq_high, 0x0E);
    //number of increments 10
    Byte_write(NumInc_low, numofinc);  //notice that the number of increments is defined as numofinc in preprocessor
    Byte_write(NumInc_high, 0x00);
    //frequency increment register - 1 kHz
    Byte_write(FreqInc_low, 0x00);
    Byte_write(FreqInc_mid, 0x00);
    Byte_write(FreqInc_high, 0x00);

    //settling time cycles register - 15
    Byte_write(NumSettle_low, 0x0F);
    Byte_write(NumSettle_high, 0x01);

    //CONTROL register
    //AD5933 in standby mode
    Byte_write(Control_high, 0xB1);  //2v-p2p        PGA gain; 1 = ×1
    Byte_write(Control_low, 0x00);   //internal clock

    //initialize with start frequency
    Byte_write(Control_high, 0x11);
    Byte_write(Control_low, 0x00);
    delay(20);
    //start frequency sweep
    Byte_write(Control_high, 0x21);
    Byte_write(Control_low, 0x00);

    //measure temperature
    Byte_write(Control_high, 0x91);
    Byte_write(Control_low, 0x00);

    while (!(Byte_read(Status) & 0x01)) {
      digitalWrite(13, HIGH);  // Use built-in LED instead of PORTB
    }                          //valid temp value
    while (!(Byte_read(Status) & 0x02))
      ;

    a = 0x00;
    b = 0x00;
    //temp data
    a = Byte_read(Temp_high);
    b = Byte_read(Temp_low);
    T = Data_proc(a, b) / 32;

    a = 0x00;
    b = 0x00;
    //real data
    a = Byte_read(Real_high);
    b = Byte_read(Real_low);
    R = Data_proc(a, b);
    a = 0x00;
    b = 0x00;
    //Imag data
    a = Byte_read(Imag_high);
    b = Byte_read(Imag_low);
    I = Data_proc(a, b);
    Z = sqrt(R * R + I * I);
    GF = GF + 0.5 * (1.0*scale / (Z * cal_res));
    

    system_phase = (int)((atan(I / R)) * radtodeg);
    system_phase = system_phase - (int)((system_phase / 360)) * 360;
    R = 0;
    I = 0;
    Z = 0;
    
  }

  // Measurement function
  void measure(void) {
    //start frequency register - 10 kHz for internal oscillator 16.776Mhz
    Byte_write(Freq_low, 0x45);  //30k >>>0x0EA645    //70k >>>>0x222EA2    //55k>>>>>1760127>0x1adb7f
    Byte_write(Freq_mid, 0xA6);
    Byte_write(Freq_high, 0x0E);
    //number of increments 100
    Byte_write(NumInc_low, numofinc);  //notice that the number of increments is defined as numofinc in preprocessor
    Byte_write(NumInc_high, 0x00);
    //frequency increment register - 10 kHz 0x027D00
    Byte_write(FreqInc_low, 0x00);
    Byte_write(FreqInc_mid, 0x00);
    Byte_write(FreqInc_high, 0x00);

    //settling time cycles register - 15
    Byte_write(NumSettle_low, 0x0F);
    Byte_write(NumSettle_high, 0x01);

    //CONTROL register
    //AD5933 in standby mode
    Byte_write(Control_high, 0xB1);  //2v-p2p        PGA gain; 1 = ×1
    Byte_write(Control_low, 0x00);   //internal clock

    //initialize with start frequency
    Byte_write(Control_high, 0x11);
    Byte_write(Control_low, 0x00);

    //start frequency sweep
    Byte_write(Control_high, 0x21);
    Byte_write(Control_low, 0x00);

    //measure temperature
    Byte_write(Control_high, 0x91);
    Byte_write(Control_low, 0x00);

    while (!(Byte_read(Status) & 0x01)) {
      digitalWrite(13, HIGH);  // Use built-in LED instead of PORTB
    }                          //valid temp value
    while (!(Byte_read(Status) & 0x02))
      ;

    // a = 0x00;
    // b = 0x00;
    // //temp data
    // a = Byte_read(Temp_high);
    // b = Byte_read(Temp_low);
    // int T = Data_proc(a, b) / 32;

    // a = 0x00;
    // b = 0x00;
    // //real data
    // a = Byte_read(Real_high);
    // b = Byte_read(Real_low);
    // R = Data_proc(a, b);
    // a = 0x00;
    // b = 0x00;
    // //Imag data
    // a = Byte_read(Imag_high);
    // b = Byte_read(Imag_low);
    // I = Data_proc(a, b);
    // Z = sqrt(R * R + I * I);
    // Result = scale / (Z * GF);

    while (!(Byte_read(Status) & 0x04)) {
      //sweep while loop

      //increment frequency
      Byte_write(0x80, 0x31);
      Byte_write(0x81, 0x00);

      while (!(Byte_read(Status) & 0x02))
        ;
      a = 0x00;
      b = 0x00;
      //real data
      a = Byte_read(Real_high);
      b = Byte_read(Real_low);
      R = Data_proc(a, b);
      a = 0x00;
      b = 0x00;
      //Imag data
      a = Byte_read(Imag_high);
      b = Byte_read(Imag_low);
      I = Data_proc(a, b);
      Z = sqrt(R * R + I * I);
      
      Result = scale/ (Z *GF);
    
      phase = (int)((atan(I / R)) * radtodeg) - system_phase;

      i = i + 1;
      if (i <= numofinc) {
        sumofresult = sumofresult + Result;
        sumofphase = sumofphase + phase;
      }
    }

    i = 0;
    Result = sumofresult / numofinc;
    phase = sumofphase / numofinc;
    phase = phase - (int)((phase / 360)) * 360;
    sumofresult = 0;
    sumofphase = 0;
  }

  // Glucose measurement calculation
  void glucose_measure(unsigned long int result) {
    // 0.00015733*Z^2 - 0.32204 * Z + 257.14
    gluco = (int)(0.00015733 * pow((result - 1000), 2) - 0.32204 * (result - 1000) + 257.14);
  }
};

#endif /* AD5933_H_ */
