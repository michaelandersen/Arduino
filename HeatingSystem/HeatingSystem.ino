
/*
 Heating system prototype v0.1
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ethernet.h>
#include <SPI.h>
#include <PID_v1.h>


#define NIMBITS_ENABLED 1
#define NIMBITS_VERBOSE 1
#define NIMBITS_ACCOUNT ""
#define NIMBITS_SECRET_KEY ""

String email = String(NIMBITS_ACCOUNT);
String secretkey = String(NIMBITS_SECRET_KEY);

// SENSOR SETTINGS

#define SAMPLE_INTERVAL 5

#define MIN_TEMP_CVBEFOREBUFFER 23.0 
#define MAX_TEMP_CVBEFOREBUFFER 24.8


// INPUT
// input pins: sensors
#define SENSOR_CV_BEFOREBUFFER 22
#define NEWSENSORS 31

// OUTPUT
// relays/valves
#define CITYHEAT_VALVE 43
// leds
#define CITYHEAT_VALVELED 24


int relay_cityheat = CITYHEAT_VALVE;


// network settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  
byte ip[] = { 192, 168, 1, 150 };
byte gateway[] = { 192, 168, 1, 1 };
byte nimbitsserver[] =  { 72, 14, 204, 104};  
/* byte nimbitsserver[] = { 192, 168, 1, 115}; */
EthernetClient client;

// setup of onewire sensors
OneWire tempSensorWire(SENSOR_CV_BEFOREBUFFER);
OneWire newSensorWire(NEWSENSORS);

DallasTemperature sbus(&tempSensorWire);
DallasTemperature newsensors(&newSensorWire);

int minTempCVbeforebuffer = MIN_TEMP_CVBEFOREBUFFER * 100;
int maxTempCVbeforebuffer = MAX_TEMP_CVBEFOREBUFFER * 100;

int sample_interval = SAMPLE_INTERVAL;
int maininterval = SAMPLE_INTERVAL * 1000;

int valve_sp = 0;  // valve setpoint, 1 = on, 0 = off

struct sensor {
  String name;
  byte addr[8]; 
  float temp; 
};



// SENSORS
sensor cvbeforebuffer = { "CVBEFOREBUFFER", {0x28, 0x60, 0x14, 0x64, 0x03, 0x00, 0x00, 0xC4}} ;
sensor boiler = { "BOILER", {0x28, 0x68, 0xCC, 0x63, 0x03, 0x00, 0x00, 0x83} };
sensor sens1 = { "SENSOR1", {0x28, 0xF2, 0x0A, 0x64, 0x03, 0x00, 0x00, 0xC5} };


float Kp = 70;
float Ki = 15; 
float Kd = 0;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd,DIRECT);

int WindowSize = 70;
unsigned long windowStartTime;


void setup() {
  windowStartTime = millis();
  Serial.begin(9600);
  Serial.println("Starting system..");
  
  
  pinMode(CITYHEAT_VALVELED, OUTPUT);
  pinMode(CITYHEAT_VALVE, OUTPUT);
  
  Serial.println("--- Closing CITYHEATVALVE (default mode)"); 
  // default mode is valve closed
  digitalWrite(CITYHEAT_VALVE, LOW);
  digitalWrite(CITYHEAT_VALVELED, LOW);

  Serial.print("--- Minimum temperature CV-BEFORE-BUFFER: "); Serial.println(MIN_TEMP_CVBEFOREBUFFER);
  Serial.print("--- Maximum temperature CV-BEFORE-BUFFER: "); Serial.println(MAX_TEMP_CVBEFOREBUFFER);
  
  Serial.println("--- Setting up network interface");
  Ethernet.begin(mac, ip);
  Serial.print("--- Nimbits updates is: ");
  if (NIMBITS_ENABLED > 0 ) Serial.println("ON");
    else Serial.println("OFF");
 
 
 Serial.println("--- Searching for new sensors");
 newsensors.begin();
 if (newsensors.getDeviceCount() > 0) {
          discoverOneWireDevices(newSensorWire, newsensors);
 } else Serial.println(" >>> No new sensors found ");
 
  Serial.println("--- Starting OneWire SensorBus"); 
  sbus.begin();
  
  // report parasite power requirements
  Serial.print("--- Parasite power is: ");
  if (sbus.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");
    
  
  Serial.println("--- Looking for 1-Wire devices...");
  Serial.print("--- Number of sensors in bus: "); 
  Serial.print(sbus.getDeviceCount());
  Serial.println();
  
  
  
  Setpoint = 23.8;
  
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  
  Serial.println("--- Starting PID");
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  

  delay(2000);
  Serial.println();
      
}



void loop(void)
{
  if ( sbus.getDeviceCount() > 0)  {
      sbus.requestTemperatures();
      delay(850);
      
      
      // get temperature from sensors
      Input = retrieve_temperature(sbus, cvbeforebuffer.name, cvbeforebuffer.addr);
      //boiler.temp = retrieve_temperature(sbus, boiler.name, boiler.addr);
      myPID.Compute();
      
       if(millis() - windowStartTime>WindowSize)
          { //time to shift the Relay Window
            windowStartTime += WindowSize;
          } 

      Serial.print("[INFO]: Switching cityheat valve: "); 
      if(Output > WindowSize / 2 ) {
        Serial.println("ON"); 
        digitalWrite(relay_cityheat,HIGH);
      } else {
        Serial.println("OFF");
        digitalWrite(relay_cityheat,LOW);
      }
      //Serial.print("[INFO]: PID - Now - window: "); Serial.println(millis() - windowStartTime);
      Serial.print("[INFO]: PID - Input: "); Serial.println(Input);
      Serial.print("[INFO]: PID - Setpoint: "); Serial.println(Setpoint);
 
      Serial.print("[INFO]: PID - Output: "); Serial.println(Output);
      Serial.print("[INFO]: PID - Kp: "); Serial.println(myPID.GetKp());
      Serial.print("[INFO]: PID - Ki: "); Serial.println(myPID.GetKi());
      Serial.print("[INFO]: PID - Kd: "); Serial.println(myPID.GetKd());
      Serial.print("[INFO]: PID - Mode: "); Serial.println(myPID.GetMode());
      Serial.print("[INFO]: PID - Direction: "); Serial.println(myPID.GetDirection());
      update_nimbits("TEST_TEMP", Input);
      update_nimbits("TEST_SETPOINT", Setpoint);
      update_nimbits("TEST_OUTPUT", Output);
      if (digitalRead(relay_cityheat) == LOW) update_nimbits("TEST_VALVE", 0 );
        else update_nimbits("TEST_VALVE", 1);
        
                
      // if measured temperature is between mininum and maximum values then open valve
      // else close valve as it is the default
      //if ( cvbeforebuffer.temp * 100 < maxTempCVbeforebuffer && cvbeforebuffer.temp * 100 > minTempCVbeforebuffer ||  cvbeforebuffer.temp * 100 > maxTempCVbeforebuffer  ) {  
        //  valve_sp=0;
      // } else {
       //   valve_sp=1;
    // }
          
     
     
      // update_nimbits(cvbeforebuffer.name, cvbeforebuffer.temp);
      //update_nimbits(boiler.name, boiler.temp);
      //update_valvestate("VALVE_CITYHEAT", _cityheat);
  
  Serial.println("............................... sleeping ........");
  delay(maininterval);
 }
//endofloop 
}

float retrieve_temperature(DallasTemperature bus, String name, byte address[8]){
    
       float temp=bus.getTempC(address);
        if (  temp >= 1.0 ){
            Serial.print("[INFO]: Temperature of "); Serial.print(name); Serial.print(" : "); Serial.print(temp); Serial.println("C");
        } else {              
            Serial.print("[ERROR]: Reading from sensor: "); Serial.print(name); Serial.println(" was unsuccessful");
        }   
        
   return temp;
}



void update_valvestate(String name, int* valvedata){
  int valve = valvedata[0];
  char valve_pv = digitalRead(valve);
  if ( valve_pv == HIGH && valve_sp == 0 ){
          digitalWrite(valve, LOW);
          Serial.print("--- Valve: "); Serial.print(name); Serial.println(" turned OFF");
  } else if (valve_pv == LOW && valve_sp == 1 ){
          digitalWrite(valve, HIGH);
          Serial.print("--- Valve: "); Serial.print(name); Serial.println(" turned ON");          
  }

  update_led(valvedata);
  update_nimbits(name, valve_sp);
}

void update_led(int* pins){
  digitalWrite(pins[1], digitalRead(pins[0]));     
}


void update_nimbits(String datapoint, float value) {
 if (NIMBITS_ENABLED == 0){
     return;
 }
  
  String pointvalue = float_to_string(value);
  String email = String(NIMBITS_ACCOUNT);
  String secretkey = String(NIMBITS_SECRET_KEY);
  
  String mainurl= "GET /service/currentvalue?";
  mainurl += "point="; 
  mainurl += datapoint; 
  mainurl += "&email="; 
  mainurl += email;
  mainurl += "&format=double";
  mainurl += "&secret=";
  mainurl += secretkey;
  mainurl += "&value=";
  mainurl += pointvalue;
  mainurl += " HTTP/1.1";

if(NIMBITS_VERBOSE != 0 ){
  Serial.print("[INFO]: Updating Nimbits - datapoint: ");
  Serial.print(datapoint);
  Serial.print(" - value: ");
  Serial.print(value);
}
  if (client.connect(nimbitsserver, 80)) {
    client.println(mainurl);
    client.println("Host:nimbits1.appspot.com");  
    client.println("Accept-Language:en-us,en;q=0.5");
    client.println("Accept-Encoding:gzip,deflate");
    client.println("Connection:close");
    client.println("Cache-Control:max-age=0");
    client.println();
    client.stop();
    if(NIMBITS_VERBOSE != 0 ){ 
        Serial.print(" => OK\n");
    }    
  } else {
    if(NIMBITS_VERBOSE != 0 ){
        Serial.print(" => FAIL\n");
    }
  }

}

String float_to_string(float val){
  char temp[10];
  dtostrf(val,1,2,temp);
  return String(temp);
}  

int discoverOneWireDevices(OneWire wire, DallasTemperature sensor_bus) {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  int count = 0;
  
 while(wire.search(addr)) {
    Serial.print("--- Found \'1-Wire\' device with address: ");
    for( i = 0; i < 8; i++) {
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');
      }
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }
    if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return 0;
    }
    Serial.println();
    count++;
  }
  wire.reset_search();
  return count;
}





