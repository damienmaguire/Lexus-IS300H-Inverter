/*Basic software to run the Lexus GS300H / IS300H hybrid transmission and inverter using the open source V1 or V2 controller
 * Take an analog throttle signal and converts to a torque command to MG1 and MG2
 * Feedback provided over USB serial
 * V1 simple menu system via usb uart added.
 * 
 * Copyright 2022 T.Darby , D.Maguire
 * openinverter.org
 * evbmw.com
 * 
 * ATTENTION : THE IS300H USES A 140 BYTE MYH PACKAGE. THE ARDUINO LIBRARY LIMITS THE RINGBUFFER SIZE TO 128 BYTES. YOU MUST MODIFY THIS TO 150 IN ringbuffer.h TO RUN THIS CODE!!!
 * https://github.com/arduino/ArduinoCore-sam/blob/790ff2c852bf159787a9966bddee4d9f55352d15/cores/arduino/RingBuffer.h#L28
 * 
 */




#include <Metro.h>
#include "variant.h"
#include <due_wire.h>
#include <Wire_EEPROM.h>

#define MG2MAXSPEED 10000
#define pin_inv_req 22

#define PARK 0
#define REVERSE 1
#define NEUTRAL 2
#define DRIVE 3

#define InvPower    34
#define Out1  50


#define IN1   6
#define IN2   7

#define TransTemp A4
#define MG1Temp A5
#define MG2Temp A6

#define EEPROM_VERSION      11


byte get_gear()
{
  if(digitalRead(IN1))
  {
  return(DRIVE);
  }
  else if(digitalRead(IN2))
  {
  return(REVERSE); 
  }
  else
  {
  return(NEUTRAL); 
  }
}


Metro timer_htm=Metro(10); 

byte mth_data[140];
byte htm_data_setup[6][105]={0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,0,95,1,
                             0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,0,95,1,
                             0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,173,255,82,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,210,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,1,0,0,0,0,220,6,
                             0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,173,255,82,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,210,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,1,0,0,0,0,220,6,
                             0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,190,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,2,0,0,0,0,203,4,
                             0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,22,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,4,75,25,212,254,190,15,0,0,0,0,0,0,0,137,0,0,0,168,0,0,0,2,0,0,0,0,203,4
};


byte htm_data[105]={0,14,0,2,0,0,0,0,0,0,0,0,0,23,0,97,0,0,0,0,0,0,0,248,254,8,1,0,0,0,0,0,0,22,0,0,0,0,0,23,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,23,0,75,22,47,250,137,14,0,0,23,0,0,0,0,201,0,218,0,16,0,0,0,29,0,0,0,0,0,0
}; 


unsigned short htm_checksum=0, 
               mth_checksum=0, 
               since_last_packet=4000;

unsigned long  last_packet=0;
            
volatile byte mth_byte=0;

float dc_bus_voltage=0,temp_inv_water=0,temp_inv_inductor=0; //just used for diagnostic output

short mg1_torque=0,
      mg2_torque=0,
      mg1_speed=-1,
      mg2_speed=-1;
       
byte inv_status=1,
     gear=get_gear(); //get this from your transmission
     
bool htm_sent=0, 
     mth_good=0;


float Version=3.01;
char incomingByte;
int Throt1Pin = A0; //throttle pedal analog inputs
int Throt2Pin = A1;
int ThrotVal=0; //value read from throttle pedal analog input

/////////////temp sensor data////////////////////
float vcc = 5.0;
float adc_step = 3.3/1023.0;
float Rtop = 1800.0;
float Ro = 47000;
float To = 25+273;
float B = 3500;
float mg1_stat=0;
float mg2_stat=0;
////////////////////////////////////////////////////

typedef struct
{
uint8_t  version; //eeprom version stored
int Max_Drive_Torque=0;
int Max_Reverse_Torque=0;
unsigned int Min_throttleVal=0;
unsigned int Max_throttleVal=0;
}ControlParams;
     
ControlParams parameters;

short get_torque()
{
  //accelerator pedal mapping to torque values here
  ThrotVal=analogRead(Throt1Pin);
  if (ThrotVal<parameters.Min_throttleVal+10) ThrotVal=parameters.Min_throttleVal;//dead zone at start of throttle travel
 if(gear==DRIVE) ThrotVal = map(ThrotVal, parameters.Min_throttleVal, parameters.Max_throttleVal, 0, parameters.Max_Drive_Torque);
 if(gear==REVERSE) ThrotVal = map(ThrotVal, parameters.Min_throttleVal, parameters.Max_throttleVal, 0, -parameters.Max_Reverse_Torque);
 if(gear==NEUTRAL) ThrotVal = 0;//no torque in neutral
  return ThrotVal; //return torque
}


int ByteRef;
int ValRef;


#define SerialDEBUG SerialUSB
 template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Allow streaming

void setup() {
  
  pinMode(pin_inv_req, OUTPUT);
  digitalWrite(pin_inv_req, 1);
  pinMode(13, OUTPUT);  //led

  pinMode(InvPower, OUTPUT);  //Inverter Relay 
  pinMode(Out1, OUTPUT);  //GP output one
  digitalWrite(InvPower,HIGH);  //turn on at startup
  digitalWrite(Out1,LOW);  //turn off at startup
  pinMode(IN1,INPUT); //Input 1
  pinMode(IN2,INPUT); //Input 2
  Serial1.begin(250000);

  PIOA->PIO_ABSR |= 1<<17;
  PIOA->PIO_PDR |= 1<<17;
  USART0->US_MR |= 1<<4 | 1<<8 | 1<<18;
/*
 * These values will most likely be wrong
  htm_data[63]=(-5000)&0xFF;  // regen ability of battery
  htm_data[64]=((-5000)>>8);

  htm_data[65]=(27500)&0xFF;  // discharge ability of battery
  htm_data[66]=((27500)>>8);
 */
  SerialDEBUG.begin(115200);
  Serial2.begin(19200); //setup serial 2 for wifi access

   Wire.begin();
  EEPROM.read(0, parameters);
  if (parameters.version != EEPROM_VERSION)
  {
    parameters.version = EEPROM_VERSION;
    parameters.Max_Drive_Torque=0;
    parameters.Max_Reverse_Torque=0;
    parameters.Min_throttleVal=0;
    parameters.Max_throttleVal=0;
    EEPROM.write(0, parameters);
  }
 
}

void handle_wifi(){
/*
 * 
 * Routine to send data to wifi on serial 2
The information will be provided over serial to the esp8266 at 19200 baud 8n1 in the form :
vxxx,ixxx,pxxx,mxxxx,nxxxx,oxxx,rxxx,qxxx* where :

v=pack voltage (0-700Volts)
i=current (0-1000Amps)
p=power (0-300kw)
m=mg1 rpm (0-10000rpm)
n=mg2 rpm (0-10000rpm)
o=mg1 temp (-20 to 120C)
r=mg2 temp (-20 to 120C)
q=oil pressure (0-100%)
*=end of string
xxx=three digit integer for each parameter eg p100 = 100kw.
updates will be every 100ms approx.

v100,i200,p35,m3000,n4000,o20,r100,q50*
*/
  
//Serial2.print("v100,i200,p35,m3000,n4000,o20,r30,q50*"); //test string

digitalWrite(13,!digitalRead(13));//blink led every time we fire this interrrupt.

Serial2.print("v");//dc bus voltage
Serial2.print(dc_bus_voltage);//voltage derived from Lexus inverter
Serial2.print(",i");//dc current
//Serial2.print(Sensor.Amperes);//current derived from ISA shunt
Serial2.print(0);
Serial2.print(",p");//total motor power
//Serial2.print(Sensor.KW);//Power value derived from ISA Shunt
Serial2.print(0);
Serial2.print(",m");//mg1 rpm
Serial2.print(abs(mg1_speed));
Serial2.print(",n");//mg2 rpm
Serial2.print(abs(mg2_speed));
Serial2.print(",o");//mg1 temp. Using water temp for now
Serial2.print(temp_inv_water);
Serial2.print(",r");//mg2 temp. Using water temp for now
Serial2.print(temp_inv_water);
Serial2.print("*");// end of data indicator

}




void control_inverter() {

  int speedSum=0;

  if(timer_htm.check()) //prepare htm data
  {
    if(mth_good)
    {
      dc_bus_voltage=(((mth_data[117]|mth_data[118]<<8))/2);
      temp_inv_water=(mth_data[42]|mth_data[43]<<8);
      temp_inv_inductor=(mth_data[86]|mth_data[87]<<8);
      mg1_speed=mth_data[10]|mth_data[11]<<8;
      mg2_speed=mth_data[43]|mth_data[44]<<8;
    }
    gear=get_gear();
    mg2_torque=get_torque(); // -3500 (reverse) to 3500 (forward)
    mg1_torque=((mg2_torque*5)/4);
    if((mg2_speed>MG2MAXSPEED)||(mg2_speed<-MG2MAXSPEED))mg2_torque=0;
    if(gear==REVERSE)mg1_torque=0;

    //speed feedback
    speedSum=mg2_speed+mg1_speed;
    speedSum/=113;
    htm_data[0]=(byte)speedSum;
    
    //mg1
    htm_data[5]=(mg1_torque*-1)&0xFF;  //negative is forward
    htm_data[6]=((mg1_torque*-1)>>8);
    htm_data[11]=htm_data[5];
    htm_data[12]=htm_data[6];

    //mg2
    htm_data[31]=(mg2_torque)&0xFF; //positive is forward
    htm_data[32]=((mg2_torque)>>8);
    htm_data[37]=htm_data[26];
    htm_data[38]=htm_data[27];

    //checksum
    htm_checksum=0;
    for(byte i=0;i<103;i++)htm_checksum+=htm_data[i];
    htm_data[103]=htm_checksum&0xFF;
    htm_data[104]=htm_checksum>>8;
  }
  
  since_last_packet=micros()-last_packet;

  if(since_last_packet>=4000) //read mth
  {    
    htm_sent=0;
    mth_byte=0;
    mth_checksum=0;
    
    for(int i=0;i<140;i++)mth_data[i]=0;
    while(Serial1.available()){mth_data[mth_byte]=Serial1.read();mth_byte++;}
    
    for(int i=0;i<138;i++)mth_checksum+=mth_data[i];
    if(mth_checksum==(mth_data[138]|(mth_data[139]<<8)))mth_good=1;else mth_good=0;
    last_packet=micros();
    digitalWrite(pin_inv_req,0);
  }

  since_last_packet=micros()-last_packet;
  
  if(since_last_packet>=10)digitalWrite(pin_inv_req,1);

  if(since_last_packet>=1000)
  {
    if(!htm_sent&&inv_status==0){for(int i=0;i<105;i++)Serial1.write(htm_data[i]);}
    else if(!htm_sent&&inv_status!=0)
    {
     // for(int i=0;i<105;i++)Serial1.write(htm_data_setup[i]);
     for(int i = 0; i < 6; i++) {
      for(int j = 0; j < 105; j++) {
    Serial1.write(htm_data_setup[i][j]);
  }
}
      if(mth_data[1]!=0) inv_status--;
    }
    htm_sent=1;
  }
}




void diag_mth()
{
  ///mask just hides any MTH data byte which is represented here with a 0. Useful for debug/discovering.
  bool mth_mask[140] = {
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,};
    
  SerialDEBUG.print("\n");
  SerialDEBUG.println("\t0\t1\t2\t3\t4\t5\t6\t7\t8\t9");
  SerialDEBUG.println("   ------------------------------------------------------------------------------");  
  for (int j=0;j<14;j++)
  {
    SerialDEBUG.print(j*10);if(j==0)SerialDEBUG.print("0");SerialDEBUG.print(" |\t");
    for (int k=0;k<10;k++)
    {
      if(mth_mask[j*10+k])SerialDEBUG.print(mth_data[j*10+k]);else SerialDEBUG.print (" ");
      SerialDEBUG.print("\t");
    }
    SerialDEBUG.print("\n");
  }
  SerialDEBUG.print("\n");
    
  SerialDEBUG.print("MTH Valid: ");if(mth_good)SerialDEBUG.print("Yes"); else SerialDEBUG.print("No");SerialDEBUG.print("\tChecksum: ");SerialDEBUG.print(mth_checksum);
  SerialDEBUG.print("\n");
  
  SerialDEBUG.print("DC Bus: ");if(dc_bus_voltage>=0)SerialDEBUG.print(dc_bus_voltage);else SerialDEBUG.print("----");
  SerialDEBUG.print("v\n");
 
  SerialDEBUG.print("MG1 - Speed: ");SerialDEBUG.print(mg1_speed);
  SerialDEBUG.print("rpm\tPosition: ");SerialDEBUG.print(mth_data[12]|mth_data[13]<<8);
  SerialDEBUG.print("\n");
  
  SerialDEBUG.print("MG2 - Speed: ");SerialDEBUG.print(mg2_speed);
  SerialDEBUG.print("rpm\tPosition: ");SerialDEBUG.print(mth_data[37]|mth_data[38]<<8);
  SerialDEBUG.print("\n");
  
  SerialDEBUG.print("Water Temp:\t");SerialDEBUG.print(temp_inv_water);
  SerialDEBUG.print("c\nInductor Temp:\t" );SerialDEBUG.print(temp_inv_inductor);
  SerialDEBUG.print("c\nAnother Temp:\t");SerialDEBUG.print(mth_data[88]|mth_data[89]<<8);
  SerialDEBUG.print("c\nAnother Temp:\t");SerialDEBUG.print(mth_data[41]|mth_data[40]<<8);
  SerialDEBUG.print("c\nAInv Status:\t");SerialDEBUG.print(inv_status);
   SerialDEBUG.print("c\ntest:\t");SerialDEBUG.print(mth_data[139]);
  SerialDEBUG.print("c\n");
  
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
}

/////////////////////////////////////////////////////////////////////////////////
//Serial menu system
////////////////////////////////////////////////////////////////////////////////



void printMenu()
{
   SerialDEBUG<<"\f\n=========== EVBMW GS450H VCU Version "<<Version<<" ==============\n************ List of Available Commands ************\n\n";
   SerialDEBUG<<"  ?  - Print this menu\n ";
   SerialDEBUG<<"  d - Print recieved data from inverter\n";
   SerialDEBUG<<"  D - Print configuration data\n";
   SerialDEBUG<<"  f  - Calibrate minimum throttle.\n ";
   SerialDEBUG<<"  g  - Calibrate maximum throttle.\n ";
   SerialDEBUG<<"  i  - Set max drive torque (0-3500) e.g. typing i200 followed by enter sets max drive torque to 200\n ";
   SerialDEBUG<<"  q  - Set max reverse torque (0-3500) e.g. typing q200 followed by enter sets max reverse torque to 200\n ";
   SerialDEBUG<<"  z  - Save configuration data to EEPROM memory\n ";
   
   SerialDEBUG<<"**************************************************************\n==============================================================\n\n";
   
}

void checkforinput()
{ 
  //Checks for keyboard input from Native port 
   if (SerialDEBUG.available()) 
     {
      int inByte = SerialDEBUG.read();
      switch (inByte)
         {
          case 'z':            
          EEPROM.write(0, parameters);
           SerialDEBUG.print("Parameters stored to EEPROM");
            break;
  
          case 'f':    
            Cal_minthrottle();
            break;
            
          case 'g':    
            Cal_maxthrottle();
            break;

        
          case 'd':     //Print data received from inverter
             diag_mth();
            break;
            
          case 'D':     //Print out the raw ADC throttle value
                PrintRawData();
            break;
          
          case 'i':     
              Cal_torque_D();
            break;
          case 'q':     
              Cal_torque_R();
            break;         
           
          case '?':     //Print a menu describing these functions
              printMenu();
            break;
            
         case 'x':     //byte modifier
         ByteMod();
            break;

          case 'y':     //byte modifier
         ValMod();
            break;     
         
          }    
      }
}



//////////////////////////////////////////////////////////////////////////////

void PrintRawData()
{
  SerialDEBUG.println("");
  SerialDEBUG.println("***************************************************************************************************");
  SerialDEBUG.print("Throttle Channel 1: ");
  SerialDEBUG.println(analogRead(Throt1Pin));
  SerialDEBUG.print("Throttle Channel 2: ");
  SerialDEBUG.println(analogRead(Throt2Pin));
  SerialDEBUG.print("Commanded Torque: ");
  SerialDEBUG.println(ThrotVal);
  SerialDEBUG.print("Selected Direction: ");
  if (get_gear()==1) SerialDEBUG.println("REVERSE");
  if (get_gear()==2) SerialDEBUG.println("NEUTRAL");
  if (get_gear()==3) SerialDEBUG.println("DRIVE");
  SerialDEBUG.print("Configured Max Drive Torque: ");
  SerialDEBUG.println(parameters.Max_Drive_Torque); 
  SerialDEBUG.print("Configured Max Reverse Torque: ");
  SerialDEBUG.println(parameters.Max_Reverse_Torque); 
  SerialDEBUG.println("Current valve positions: ");
  SerialDEBUG.print("MG1 Stator temp: ");
  SerialDEBUG.println(mg1_stat);
  SerialDEBUG.print("MG2 Stator temp: ");
  SerialDEBUG.println(mg2_stat);
  SerialDEBUG.println("***************************************************************************************************");  
}

///////////////////Throttle pedal calibration//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void Cal_minthrottle()
{
  SerialDEBUG.println("");
     SerialDEBUG.print("Configured min throttle value: ");
     parameters.Min_throttleVal=(analogRead(Throt1Pin));
     if(parameters.Min_throttleVal<0) parameters.Min_throttleVal=0;//noting lower than 0 for min.
     SerialDEBUG.println(parameters.Min_throttleVal);
}

void Cal_maxthrottle()
{
  SerialDEBUG.println("");
   SerialDEBUG.print("Configured max throttle value: ");
   parameters.Max_throttleVal=(analogRead(Throt1Pin));
   if (parameters.Max_throttleVal>1000) parameters.Max_throttleVal=1000;//limit on max value
   SerialDEBUG.println(parameters.Max_throttleVal);
   
}
//////////////////////////////////////////////////////////////////////////////////////////


//////////Torque calibration//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void Cal_torque_D()
{
  SerialDEBUG.println("");
   SerialDEBUG.print("Configured drive torque: ");
     if (SerialDEBUG.available()) {
    parameters.Max_Drive_Torque = SerialDEBUG.parseInt();
  }
  if(parameters.Max_Drive_Torque>3500) parameters.Max_Drive_Torque=3500;//limit max drive torque to within range
  SerialDEBUG.println(parameters.Max_Drive_Torque); 
  }

void Cal_torque_R()
{
  SerialDEBUG.println("");
   SerialDEBUG.print("Configured reverse torque: ");
     if (SerialDEBUG.available()) {
    parameters.Max_Reverse_Torque = SerialDEBUG.parseInt();  
  }
  if(parameters.Max_Reverse_Torque>3500) parameters.Max_Reverse_Torque=3500;//limit max reverse torque to within range
  SerialDEBUG.println(parameters.Max_Reverse_Torque); 
  }
  
/////////////////////////////////////////////////////////////////////////////////////////






void processTemps()
{
mg1_stat=readThermistor(analogRead(MG1Temp));
mg2_stat=readThermistor(analogRead(MG2Temp));
  
}

void ByteMod()
{
ByteRef = SerialDEBUG.parseInt();
SerialDEBUG.print("Byte to modify : ");
SerialDEBUG.println(ByteRef);
  
}

void ValMod()
{
ValRef = SerialDEBUG.parseInt();
SerialDEBUG.print("Modified value : ");
SerialDEBUG.println(ValRef);
htm_data[ByteRef]=ValRef;  
}


//////////////Dilbert's temp sensor routine////////////////////////////
float readThermistor(int adc){


float raw = adc;
float voltage = raw*adc_step;

float Rt = (voltage * Rtop)/(vcc-voltage);

float temp = (1/(1.0/To + (1.0/B)*log(Rt/Ro)))-273;

return temp;
}
///////////////////////////////////////////////////////////////////////

Metro timer_diag = Metro(1100);

void loop() {
  
  control_inverter();

  if(timer_diag.check())
  {
    //diag_mth();
  processTemps();
   handle_wifi();
  }
    checkforinput(); //Check keyboard for user input 
}
