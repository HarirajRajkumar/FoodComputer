// V1.2.2 FOR MEGA

#include <LiquidCrystal_I2C.h> // LCD_i2c library
#include <Wire.h> // I2C communication library for LCD
// Library used to read from DHT11
//#include <DHT.h>
//Libraries included to read data from DS18B20
#include <OneWire.h> // One wire communication  
#include <DallasTemperature.h> // Library used to read temperature from DS18B20
//Libraries included for PID control
#include<PID_v1.h>
//Lib used to read data from DHT11 and object 
#include"DHT.h"

//Wire Definitions
#define DHTPIN 13 
// DHT type Usage
#define DHTTYPE DHT22       
DHT dht(DHTPIN,DHTTYPE);

#define One_wire_pin 12 // FOR DS18B20 One wire communication


// Variables used for PID control
double Kp=2,Ki=5,Kd=1;
double SetPoint,Input,Output; // FOR PID Control

// classes object
  LiquidCrystal_I2C myLCD(0x27,16,4);
// classes defined for reading temperature from ds 18b20
  OneWire myOneWire ( One_wire_pin );
  DallasTemperature mySensor(&myOneWire);
  PID myPID(&Input,&Output,&SetPoint,Kp,Ki,Kd,DIRECT);
  // for DHT11 sensor

// Pin definition and declarations



#define myFanPin 2

// For thermistor
const byte THERMISTOR_PIN_1 =A0;
const byte THERMISTOR_PIN_2 =A1;
#define NUMTEMPS 20

// USER DEFINED FUNCTION Declarations

/* USER DEFINED FUNCTION 1
 *  Function Name : printHeaterPerValue();
 *  What does it do ?
 *      Prints H
 *   
 *  Variables parsed : 
 *      xPos and yPos to print the data.
 * 
 */

void printHeaterPercValue(byte xPos , byte yPos,byte PIDoutput);

/* USER DEFINED FUNCTION 2
 *    Function Name : foodCompBegin();
 *     What does it do ?
 *        Initializes LCD by printing the version and
 *        specific data required.
 *    MUST BE CALLED IN SETUP function.
 */   

 void foodCompBegin(void);

 /*
 * USER DEFINED FUNCTION 3
 *    Function Name: LCDtempPrint();
 *    What does it do ?
 *      Prints Temperature Data in specified co-ordinates of LCD
 */
 void LCDtempPrint(byte xPos, byte yPos, byte temperature);

  /*
  * User Defined Function 4
  *     Function Name : fanStatus();
  *     What does it do ?
  *       Prints Fans status on provided LCD location.
  */
  void fanStatus(byte xPos, byte yPos, byte fanStatus, byte fanPin,byte thresholdForOFF,byte thresholdForON);

  /*
   * User Defined Function 5
   *      Funcn : getDallasTemperatureC();
   */

  int getDallasTemperatureC();

  /*
   *  User Defined Function 6
   *   thermistorTempC();
   *  Returns Temperature of 1st Thermistor in Celsius .
   *   
   */

  int thermistorTempC1();

   /*
   *  User Defined Function 6.1
   *   thermistorTempC2();
   *  Returns Temperature of 2nd Thermistor in Celsius .
   *   
   */

  int thermistorTempC2();

  /*
   * User defined Function 7
   *  DHT11Temp();
   *  Used to read data from DHT11 and returns data in Celsius.
   */
   float DHT11Temp();

   /* 
    *  User Defined Function 8
    *  DHT11Hum();
    *  Used to read humidity data from DHT11.
    */
    float DHT11Hum();

    /*
     *  User Defined Function 9
     *  LCDfanStatusPrint();
     *  Prints Fans status at the given LCD's XY position
     */
     void LCDfanStatusPrint(byte xPos , byte yPos);

    /*
     * User Defined Function 10
     * peltierControl();
     * returns both fan's RPM and turns ON peltier relay after 5 seconds
     */
     void peltierControl(unsigned long peltierMillis);

     /*
      * UDF 11
      * peltierBegin();
      * 
      */
      void peltierBegin();

      /*
       * UDF 13
       * LCDpeltierEmergenyPrint();
       * 
       */
       void LCDpeltierEmergenyPrint();
      
//Selected PWM pin for UNO
//Should change when comes to MEGA
const byte heater1PWM_pin = 3;
const byte heater2PWM_pin = 5;

// Structure
// at present pwm at pin 3 and 5 are controlled from data at pin A0
struct heaterPWM{
  //int heaterPWM_data; // takes PWM data varing from 0 to 255
  char printPIDoutputPerc_LCD[3];    // takes % of Heater's PWM data varying from 0 to 100%
  double PID_control_Input; //Input which must be DS's Temperature
  double PID_control_Output; // Output
  int tempTEMP; //Temperature temp variable
  byte fanStatus;
}htr[1]; // since 2 heaters are present htr[1] = htr[0] and htr[1] -- array of structures

// Structure to control Peltier and fan 
struct peltier{
  int RPM;
  boolean fault;
  boolean peltierRelayState;
}fan[1]; 


// Peltier Control Pins
#define Tacho_1 4
#define Tacho_2 8
#define PeltierRelay 7
#define fanRelay 9

const byte setEnvTemp = 31; // Max enviroment Temperature (inside box)
const int setHtrTemp = 200; // Max Heater Temperature 


void setup(void) {  
    Serial.begin(9600);
    foodCompBegin();// begin to print the main Skeleton for LCD 
    pinMode(heater1PWM_pin,OUTPUT); // PWM adjustment (1st)
    pinMode(heater2PWM_pin,OUTPUT); // PWM adjustment (2nd)
    pinMode(myFanPin , OUTPUT);
    mySensor.begin();  // Dallas Temperature begin

    peltierBegin(); 
    
    // PID Control begin
    SetPoint=setEnvTemp; // User Variable
    myPID.SetMode(AUTOMATIC);
    pinMode(myFanPin,OUTPUT);

    dht.begin();
    digitalWrite(myFanPin , LOW);

}//close setup

unsigned long previousMillis = 0;
unsigned long myMillis;

unsigned long previousthermMillis = 0;
unsigned long previousthermMillis_2 = 0;

void loop()   {

   myMillis= millis();

  //int tempDHT = DHT11Temp(); // get data from DHT11 at pin 13
  
  htr[0].PID_control_Input = (getDallasTemperatureC());//+tempDHT)/2; // average data of both temperature from DS18B20 and DHT11
  
  Input = htr[0].PID_control_Input;// providing input to PID function
  
 // myPID.SetSampleTime();  
  myPID.Compute();
  
  
  htr[0].PID_control_Output = Output;// Output of PID function
  htr[1].PID_control_Output = htr[0].PID_control_Output ;
  boolean ledState = 0;
  
  // Initial Direct heat up till 153*C and do PID  ( PRE HEAT !!)
  if ( htr[0].PID_control_Output < 153 )//&& htr[1].PID_control_Output < 153 )
  {
    if(myMillis - previousMillis >= 100 )
    {
      previousMillis = myMillis;
           if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
     
    }
         digitalWrite(heater1PWM_pin,ledState);
         digitalWrite(heater2PWM_pin,ledState);
         htr[0].fanStatus=0;
         digitalWrite(myFanPin , LOW);

  }
  else
  {
    analogWrite(heater1PWM_pin,htr[0].PID_control_Output); 
    analogWrite(heater2PWM_pin,htr[1].PID_control_Output);
    digitalWrite(myFanPin , HIGH);
             htr[0].fanStatus=1;

  }

  
  int thermistorTemp_1;
  // Reading data from two Thermistor
  if(myMillis - previousthermMillis > 100) // first thermistor
  {
    previousthermMillis = myMillis;
   thermistorTemp_1= thermistorTempC1();  
  }

  if(myMillis - previousthermMillis_2 > 150) // second thermistor
  { 
    previousthermMillis_2 = myMillis;
  thermistorTempC2(); // directly prints temperature
  }

  peltierControl(myMillis);
  
  DHT11Hum(); // read humidity
  LCDfanStatusPrint(10,3); // print fan status
  //Serial.print(thermistorTemp_1);
  //Serial.print("\t");
  //Serial.println(htr[0].PID_control_Input);
  
  
  printHeaterPercValue(2,2,htr[0].PID_control_Output);
  printHeaterPercValue(2,3,htr[1].PID_control_Output);
  LCDtempPrint(10,2,htr[0].PID_control_Input);
}//close loop

