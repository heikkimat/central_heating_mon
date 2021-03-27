/*
  DS18B20_to_I2C_LCD_and_BTH
  Lukee lämpötilan DS18B20 sensoreilta ja 
  kirjoittaa I2C_LCD:lle sekä virtuaalisarjaporttiin (lähettää bluetooth:lla.)
  Heikki Mattila 25.10.2016
  
  ref: 
  http://stuff.nekhbet.ro/2009/08/23/how-to-use-the-ds18s20-and-ds18b20-temperature-sensors-with-arduino.html
  http://tutorialpedia.org/tutorials/Working+with+Dallas+DS18S20+and+DS18B20+temperature+sensors+from+Arduino.html
  http://www.pjrc.com/teensy/td_libs_OneWire.html
  http://www.arduino.cc/en/Tutorial/LiquidCrystal  
 

 
 */
 
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

//init the one wire interface on pin 11
// Data wire is plugged into port 3 on the Arduino
// Connect a 4.7K resistor between VCC and the data pin (strong pullup)
// DS18B20 Pins from left to right: GND, data, null/VCC
// All DS18B20 sensors in parallel
OneWire  myWire(3);

 
//addresses of the sensors (use Search_1_wire_devices.pde)
byte Tsensor_1[8] = {0x28, 0x65, 0xEA, 0xFA, 0x02, 0x00, 0x00, 0x74};
byte Tsensor_2[8] = {0x28, 0x60, 0x92, 0x77, 0x03, 0x00, 0x00, 0x23};
byte Tsensor_3[8] = {0x28, 0x34, 0x42, 0x77, 0x03, 0x00, 0x00, 0x8F};
byte Tsensor_4[8] = {0x28, 0xE7, 0xBA, 0xFA, 0x02, 0x00, 0x00, 0x65};
//28-60-92-77-03-00-00-23
// 28-34-42-77-03-00-00-8F

// set the LCD address to 0x27 for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
/*
 * LCD Circuit diagram:
 * GND - GND
 * VCC - 5V
 * SDA - ANALOG Pin 4
 * SCL - ANALOG pin 5
 */

// creates a "virtual" serial port/UART
// connect BT module TX to D12
// connect BT module RX to D13
// connect BT Vcc to 5V, GND to GND
SoftwareSerial BT(12, 13); 

void setup(void) {
   // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  //lcd.print("Setting up");
  
  // set the data rate for the SoftwareSerial port
  BT.begin(9600);
}
 
void writeTimeToScratchpad(byte* address){
  //reset the bus
  myWire.reset();
  //select our sensor
  myWire.select(address);
  //CONVERT T function call (44h) which puts the temperature into the scratchpad
  myWire.write(0x44,1);
  //sleep a second for the write to take place
  delay(1000);
}
 
void readTimeFromScratchpad(byte* address, byte* data){
  //reset the bus
  myWire.reset();
  //select our sensor
  myWire.select(address);
  //read the scratchpad (BEh)
  myWire.write(0xBE);
  for (byte i=0;i<9;i++){
    data[i] = myWire.read();
   // Serial.println(data[i],HEX);
  }
}
 
int getTemperature(byte* address){
  int temp;
  int tr;
  byte data[12];
 
  writeTimeToScratchpad(address);
 
  readTimeFromScratchpad(address,data);
  
  temp=(data[1]<<8)+data[0];//take the two bytes from the response relating to temperature
  temp=temp>>4;//divide by 16 to get pure celcius readout
  return temp;
  
  /*
  // apuprinttaus
  int temp;
  temp=(data[1]<<8)+data[0];//take the two bytes from the response relating to temperature
  temp=temp>>4;//divide by 16 to get pure celcius readout
  Serial.print("T=");//output the temperature to serial port
  Serial.println(temp);
  
  //put in temp all the 8 bits of LSB (least significant byte)
  tr = data[0];
 
  //check for negative temperature
  if (data[1] > 0x80){
    tr = !tr + 1; //two's complement adjustment
    tr = tr * -1; //flip value negative.
  }
 
  //COUNT PER Celsius degree (10h)
  int cpc = data[7];
  //COUNT REMAIN (0Ch)
  int cr = data[6];
 
  //drop bit 0
  tr = tr >> 1;
 
  //calculate the temperature based on this formula :
  //TEMPERATURE = TEMP READ - 0.25 + (COUNT PER Celsius Degree - COUNT REMAIN) / (COUNT PER Celsius Degree)
 
  return tr - (float)0.25 + (cpc - cr)/(float)cpc;
  */
}
 
//fahrenheit to celsius conversion
float f2c(float val){
  float aux = val - 32;
  return (aux * 5 / 9);
}
 
//celsius to fahrenheit conversion
float c2f(float val){
  float aux = (val * 9 / 5);
  return (aux + 32);
}
 
void loop(void) {
int temp_yla;
int temp_kesk;
int temp_ala;
temp_yla = getTemperature(Tsensor_1);
temp_kesk = getTemperature(Tsensor_2);
temp_ala = getTemperature(Tsensor_3);

// lämpötilojen korjaus

temp_yla = 1.26*temp_yla - 5.05;
temp_kesk = 1.26*temp_kesk - 5.05;
temp_ala = 1.26*temp_ala - 5.05;



// lämpötilat näytölle
      lcd.setCursor(0, 0);
      lcd.print("Y=");
      lcd.print(temp_yla);
      lcd.print(" C ");
      lcd.print("K=");
      lcd.print(temp_kesk);
      lcd.print(" C ");
      lcd.setCursor(0, 1);
      lcd.print("A=");
      lcd.print(temp_ala);
      lcd.print(" C ");
 
      // write to virtual serial port (BTH)
      BT.print(" " );      
      BT.println(temp_ala);
      delay(10);             
      BT.print(" " );      
      BT.println(temp_kesk+1000);       
      delay(10);  
      BT.print(" " );      
      BT.println(temp_yla+2000);       
      delay(10);

} // loop

//eof
