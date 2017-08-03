//Libraries used are the Arduino PID library, ESP8266 built in libraries, aws-sdk-esp8266, DHT, TimeLib, and RCSwitch libraries
#include <PID_v1.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "Esp8266AWSImplementations.h"
#include "AmazonDynamoDBClient.h"
#include "AWSFoundationalTypes.h"
#include "keys.h"
#include <DHT.h>
#include <Wire.h>
#include <RCSwitch.h>
#include <Time.h>
#include <TimeLib.h>
//Not using a LCD screen anymore - deprecated
//#include <LiquidCrystal_I2C.h>

// RTC data fields - used for data restore on device reset. This is largely taken from the RTC example in the ESP8266 library.
// CRC function used to ensure data validity
uint32_t calculateCRC32(const uint8_t *data, size_t length);

// helper function to dump memory contents as hex
void printMemory();

// Structure which will be stored in RTC memory.
// First field is CRC32, which is calculated based on the
// rest of structure contents.
// Any fields can go after CRC32.
// We use byte array as an example.
struct {
  uint32_t crc32;
  float t5;
  float t90;
  float u5;
  float u90;
  float p5;
  float p90;
  float h5;
  float h90;
  int c5;
  int c90;
  unsigned long initial;
  unsigned long last90;
  unsigned long last1d;
} rtcData;

//Initializing transistor pins
int transistorPin = 14;
int incomingByte;  // a variable to read incoming serial data into
//Setpoint hardcoded, will switch to a GetItem from DynamoDB in the future
double setPoint = 69.0;
//Intializing the PID output, to avoid errors on the first PID run
double pidOutput = 0;
//Setting to store transistor state
int transistorGate;

//This section of floats and counter ints stores the current and running data for the data that will be outputted to DynamoDB 
float temp_f_sum_20s, temp_f_sum_5m, temp_f_sum_90m = 0;
float humidity_sum_20s, humidity_sum_5m, humidity_sum_90m = 0;
float pid_sum_20s, pid_sum_5m, pid_sum_90m = 0;
float heater_sum_20s, heater_sum_5m, heater_sum_90m = 0;

float temp_f_inst_5m, humidity_inst_5m, pid_inst_5m, heater_inst_5m = 0;
float temp_f_inst_90m, humidity_inst_90m, pid_inst_90m, heater_inst_90m = 0;
float temp_f_inst_1d, humidity_inst_1d, pid_inst_1d, heater_inst_1d = 0;

float heater_now;

int counter_20s = 0;
int counter_5m = 0;
int counter_90m = 0;

//NTP server variable section
unsigned int localPort = 2390;
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "0.north-america.pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

WiFiUDP udp;

//Variables to keep track of current and past upload time (format is seconds since last unix epoch)
unsigned long epoch, epoch_initial, epoch_now, epoch_last_5m, epoch_last_90m, epoch_last_1d;

//Currently not in use, this is a setting that should trip and stop average uploads if too many lower level time periods have passed (i.e. if the 5min average has taken 100 measurements of 20sec apiece)
bool flagDontPushAvgTemp = false; 

// PID tuning variables 
double KP = 25;
double KI = 0.028;
double KD = 0;   // Not yet used
unsigned long windowSize = 1800000; // 30 minutes (ish)

//Variable to keep track of PID window
unsigned long windowStartTime = 0;

//Variable to keep track of heater on or off state
boolean stateVariable = false;

//AWS constants
const char* AWS_REGION = "us-west-1";
const char* AWS_ENDPOINT = "amazonaws.com";

// Constants describing DynamoDB table and values being used
const char* TABLE_NAME1 = "Temperatures_20_sec";
const char* HASH_KEY_NAME1 = "Id";
const char* HASH_KEY_VALUE1 = "Thermostat_01"; // My sensor ID
const char* RANGE_KEY_NAME1 = "Date";

const char* TABLE_NAME3 = "Temperatures_5_min";
const char* HASH_KEY_NAME3 = "Id";
const char* HASH_KEY_VALUE3 = "Thermostat_01";
const char* RANGE_KEY_NAME3 = "Date";

const char* TABLE_NAME4 = "Temperatures_90_min";
const char* HASH_KEY_NAME4 = "Id";
const char* HASH_KEY_VALUE4 = "Thermostat_01";
const char* RANGE_KEY_NAME4 = "Date";

const char* TABLE_NAME5 = "Temperatures_1_day";
const char* HASH_KEY_NAME5 = "Id";
const char* HASH_KEY_VALUE5 = "Thermostat_01";
const char* RANGE_KEY_NAME5 = "Date";

const char* TABLE_NAME2 = "HeaterManualState";
const char* HASH_KEY_NAME2 = "Id";
const char* HASH_KEY_VALUE2 = "Website";
const char* RANGE_KEY_VALUE2 = "1";
const char* RANGE_KEY_NAME2 = "Date";
static const int KEY_SIZE = 2;

//Variable to internally track the heater state of the website
static int WebsiteHeaterState;

//DynamoDB objects
Esp8266HttpClient httpClient;
//I suspect the ESP8266 AWS DateTimeProvider is causing some of the device crashes, so I'm rolling my own
Esp8266DateTimeProvider dateTimeProvider;
GetItemInput getItemInput;
PutItemInput putItemInput;
AttributeValue hashKey;
AttributeValue rangeKey;
ActionError actionError;

AmazonDynamoDBClient ddbClient;

//LCD screen no longer used
//LiquidCrystal_I2C  lcd(0x3F, 4, 5);

RCSwitch mySwitch = RCSwitch();

//Temperature sensor variables
DHT dht(5, DHT22, 20);
double humidity, temp_f = 0;

//PID variable initial setting
PID myPID(&temp_f, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

//Loop and AWS offset time variables 
unsigned long lastTimeUpdate, lastTempOutputUpdate, lastHeaterRefresh, awsoffset = 0;

//Setup section
void setup()
{
  Serial.begin(115200);
  Serial.println("Serial initialized...");

  // Set pins for transistor and onboard LED blink
  pinMode(2, OUTPUT);
  pinMode(transistorPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize Wifi - ssid and password manually set in and then taken from keys.h in the AWS SDK library
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    yield();
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //Reset the WiFi if the NTP server can't be looked up (i.e. if DNS is unfunctional)
  if(!WiFi.hostByName(ntpServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.reset();
  }

  //Initialize UDP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  
  //Initialize RFID switch
  mySwitch.enableTransmit(2);
  mySwitch.setPulseLength(184);

  //Initialize DHT sensor
  dht.begin();

  //Initialize LCD screen on unit (deprecated) 
  //  lcd.init();
  //  lcd.backlight();
  
  //Flash builtin to alert startup
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(2, LOW);
  
  //Initialize PID limits
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  //Take first temp reading and compute PID for initial output
  temp_f = dht.readTemperature(true);
  if (!isnan(temp_f)) {
    yield();
    myPID.Compute();
  }

  Serial.print("Initial PID output: ");
  Serial.println(pidOutput);

  //Initialize DynamoDB client
  ddbClient.setAWSRegion(AWS_REGION);
  ddbClient.setAWSEndpoint(AWS_ENDPOINT);
  ddbClient.setAWSSecretKey(awsSecKey);
  ddbClient.setAWSKeyID(awsKeyID);
  ddbClient.setHttpClient(&httpClient);
  ddbClient.setDateTimeProvider(&dateTimeProvider);

  //Attempt to read the RTC memory to see if there are any long-time upload data remaining from before the device reset, and if read is valid then set the available data
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("RTC Read: ");
    printMemory();
    Serial.println();
    uint32_t crcOfData = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
    Serial.print("CRC32 of data: ");
    Serial.println(crcOfData, HEX);
    Serial.print("CRC32 read from RTC: ");
    Serial.println(rtcData.crc32, HEX);
    if (crcOfData != rtcData.crc32) {
      Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Data is probably invalid!");
    }
    else {
      Serial.println("CRC32 check ok, data is probably valid.");

  temp_f_sum_5m = rtcData.t5;    
  temp_f_sum_90m = rtcData.t90;
      
  humidity_sum_5m = rtcData.u5;    
  humidity_sum_90m = rtcData.u90;
      
  pid_sum_5m = rtcData.p5;    
  pid_sum_90m = rtcData.p90;
      
  heater_sum_5m = rtcData.h5;    
  heater_sum_90m = rtcData.h90;

  counter_5m = rtcData.c5;
  counter_90m = rtcData.c90;

  epoch_initial = rtcData.initial;
  epoch_last_90m = rtcData.last90;
  epoch_last_1d = rtcData.last1d;
  
    }
    
  }

}

//Debugger functions for troubleshooting
void yieldEspCPUTime(int x) {
//  delay(100);
  ESP.wdtFeed();
//  yield();
  Serial.print("TimeDebug");
  Serial.print(x);
  Serial.print(",");
}

void yieldEspCPUTemp(int x) {
//  delay(100);
  ESP.wdtFeed();
//  yield();
  Serial.print("TempDebug");
  Serial.print(x);
  Serial.print(",");
}

void yieldEspCPU(int x) {
  delay(100);
  ESP.wdtFeed();
  yield();
  Serial.print("StdDebug");
  Serial.print(x);
  Serial.print(",");
}

void loop() {
  //Increment program loop timer
  unsigned long now = millis();

    //'ML', 'TD', and 'TE' serial prints also used for debugging
    Serial.print("ML1, ");
  
  //NTP time request loop, executes every 10 sec
  if ( now - lastTimeUpdate > 10000) {
    Serial.println();
    Serial.print("Sending NTP request");
    Serial.println();
    sendNTPpacket(timeServerIP);
    yield();
    lastTimeUpdate = now;
  }

  Serial.print("ML2, ");
  
  //Time update, executes when a time server request has successfully returned
  uint32_t time = getTheTime();
  yield();
  if (time) {
	    epoch = time;
	    
	    //Setting inital average update times if they do not exist (after inialization or in RTC memory)
	    if (!epoch_initial) {
	    epoch_initial = epoch;
	    epoch_last_5m = epoch;
	    epoch_last_90m = epoch;
	    epoch_last_1d = epoch;
	    } 
	  if (!epoch_last_5m) {
	    //Just in case RTC memory loads a update time for 90m/1d that would be sooner than the 5m
	      epoch_last_5m = epoch;
	  }
	    epoch_now = epoch;
	    Serial.println("Time successful");
  }

    Serial.print("ML3, ");
  
  //Temperature then output update loop, executes every 20 sec 
  if ( now - lastTempOutputUpdate > 20000 ) {
    Serial.println();
    Serial.print("Get Temp-");
    Serial.println();
    gettemperature();
    yield();
    Serial.println();
    Serial.println("Temp Successful");
    Serial.println();
    Serial.print("Update Output-");
    updateOutput();
    yield();
    Serial.println("Successful");
    Serial.println();
    lastTempOutputUpdate = now;
  }
  
    Serial.print("ML4, ");
  
  //Now combined into the temperature loop - PID output/heater control update loop, executes every 20 sec
  //if ( now - lastOutputUpdate > 20000 ) {
  //}

   // Serial.print("ML5, ");

    yield();

    Serial.print("ML6, ");
}

//Checksum function to validate RTC data
uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

//RTC memory printout
void printMemory() {
  Serial.println();
  Serial.print("t5:");
  Serial.print(rtcData.t5);
  Serial.print(" t90:");
  Serial.print(rtcData.t90);
  Serial.print(" u5:");
  Serial.print(rtcData.u5);
  Serial.print(" u90:");
  Serial.print(rtcData.u90);
  Serial.print(" p5:");
  Serial.print(rtcData.p5);
  Serial.print(" p90:");
  Serial.print(rtcData.p90);
  Serial.print(" h5:");
  Serial.print(rtcData.h5);
  Serial.print(" h90:");
  Serial.print(rtcData.h90);
  Serial.print(" c5:");
  Serial.print(rtcData.c5);  
  Serial.print(" c90:");
  Serial.print(rtcData.c90);
  Serial.print(" initial:");
  Serial.print(rtcData.initial);
  Serial.print(" last90:");
  Serial.println(rtcData.last90);
  Serial.print(" last1d:");
  Serial.print(rtcData.last1d);
  Serial.println();
}

uint32_t getTheTime() { // Get the NTP time for averaging over the longer intervals - code largely similar to Arduino NTP example
  //Initial delay to wait for NTP time server response
  unsigned long tempoffset = millis();
  delay(1000);
  
  if (udp.parsePacket() == 0) {
    return 0;
  }
  
  //Initializing the AWS offset for AWS packet time purposes
  awsoffset = tempoffset; 

  udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

  Serial.print("TD22, ");
  
  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  
  Serial.print("TD23, ");

  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years for Unix time:
  unsigned long UNIXTime = secsSince1900 - seventyYears;
  return UNIXTime;
}

// send an NTP request to the time server at the given address - also largely from Arduino NTP example
void sendNTPpacket(IPAddress& address) {

    Serial.print("TD25, ");
  
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

    Serial.print("TD26, ");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123

    Serial.print("TD27, ");
  
  //I've been getting a good amount of controller resets (watchdog timer and otherwise) at this step, need more investigation
  udp.write(packetBuffer, NTP_PACKET_SIZE);

    Serial.print("TD28, ");

  udp.endPacket();
  yield();

    Serial.print("TD29, ");
}

void gettemperature() {

  Serial.print("TE 41, ");
  
  // Reading temperature for humidity takes about 250 milliseconds
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  humidity = dht.readHumidity();          // Read humidity (percent)
  temp_f = dht.readTemperature(true);     // Read temperature as Farenheight

  Serial.print("TE 42, ");

  // Check if any reads failed
  if (isnan(humidity) || isnan(temp_f)) {
    //If reads failed try to re-read sensor
    byte hu;
    for (hu = 0; hu < 30; hu++)
    {
      temp_f = dht.readTemperature(true);
      if isnan(temp_f) {

        Serial.print("TE 43, ");
	
        delay(150);
        temp_f = dht.readTemperature(true);
      }
    }
    return;
  }

  Serial.print("TE 44, ");

  //Get and set the current website heater state
  awsGetWebsiteHeaterState(&WebsiteHeaterState);
  yield();
  
  // Handling for various times and averages
  if (stateVariable)
  {
      heater_now = 100.00;
  }
  else
  {
      heater_now = 0.00;
  }

  Serial.print("TE 45, ");

  //Push 20sec data to DyanmoDB
  PushTempToDynamoDB(temp_f, humidity, pidOutput, heater_now, TABLE_NAME1, HASH_KEY_NAME1, HASH_KEY_VALUE1, RANGE_KEY_NAME1);
  yield();
  
  Serial.print("TE 46, ");
  
  //Adding cumulatively the values of the 20sec measurements
  temp_f_sum_20s = temp_f_sum_20s + temp_f;
  humidity_sum_20s = humidity_sum_20s + humidity;
  pid_sum_20s = pid_sum_20s + pidOutput;
  heater_sum_20s = heater_sum_20s + heater_now;
  
  //Incrementing 20sec measurement instance counter
  counter_20s++;

  Serial.print("TE 47, ");

  Serial.print("Epoch now: ");
    Serial.print(epoch_now);
  Serial.print(" Epoch last 5m: ");
    Serial.print(epoch_last_5m);
  Serial.print(" Epoch last 90m: ");
    Serial.print(epoch_last_90m);
  Serial.print(" Epoch last 1d: ");
    Serial.print(epoch_last_1d);

  //Handling once NTP time has passed 5min
  if (epoch_now - epoch_last_5m > 300) { 

      Serial.print("TE 48, ");

      //Averaging 20sec measurements and setting them to a 5min variable
      temp_f_inst_5m = temp_f_sum_20s / counter_20s;
      humidity_inst_5m = humidity_sum_20s / counter_20s; 
      pid_inst_5m = pid_sum_20s / counter_20s;
      heater_inst_5m = heater_sum_20s / counter_20s;

      //Resetting 20sec sums
      temp_f_sum_20s = 0;
      humidity_sum_20s = 0;
      pid_sum_20s = 0;
      heater_sum_20s = 0;
    
      //Resetting 20sec measurement instance counter
      counter_20s = 0;

      Serial.print("TE 49, ");
      
      //Incrementing 5min measurement instance counter
      counter_5m++;

	  //Writing values to RTC memory in case of crash
	  RTCMemWrite();
      
	  Serial.print("TE 50, ");
    
      //Sending 5m avg measurements
      if (!flagDontPushAvgTemp) {
       	Serial.println("Pushing 5m average to DB:\t");
        Serial.print(temp_f_inst_5m);
        Serial.print(",");
        Serial.print(humidity_inst_5m);
        Serial.print(",");
        Serial.println(counter_20s);
            // print the hour, minute and second:
        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
        Serial.print(':');
        if (((epoch % 3600) / 60) < 10) {
          // In the first 10 minutes of each hour, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ((epoch % 60) < 10) {
          // In the first 10 seconds of each minute, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print(epoch % 60); // print the second
        Serial.print(",Epoch Now");
        Serial.print(",");
        Serial.print(epoch_now);
        Serial.print(",");
        Serial.print("Epoch Last Day");
        Serial.print(",");
        Serial.print(epoch_last_1d);
        Serial.print("Epoch Last 90m");
        Serial.print(",");
        Serial.print(epoch_last_90m);
        Serial.print("Epoch Last 5m");
        Serial.print(",");
        Serial.print(epoch_last_5m);
        Serial.print("Epoch Difference Now to Last 5m");
        Serial.print(",");
        Serial.print(epoch_now - epoch_last_5m);
        Serial.println();
	
      Serial.print("TE 51, ");

        PushTempToDynamoDB(temp_f_inst_5m, humidity_inst_5m, pid_inst_5m, heater_inst_5m, TABLE_NAME3, HASH_KEY_NAME3, HASH_KEY_VALUE3, RANGE_KEY_NAME3);
  yield();
	
      Serial.print("TE 52, ");

      }
  }

  if (epoch_now - epoch_last_90m > 5400) {

      Serial.print("TE 53, ");
      
      //Averaging 90min measurements and setting them to a 90min variable
      temp_f_inst_90m = temp_f_sum_5m / counter_5m;
      humidity_inst_90m = humidity_sum_5m / counter_5m; 
      pid_inst_90m = pid_sum_5m / counter_5m;
      heater_inst_90m = heater_sum_5m / counter_5m;

      //Resetting 5min sums
      temp_f_sum_5m = 0;
      humidity_sum_5m = 0;
      pid_sum_5m = 0;
      heater_sum_5m = 0;
    
      //Resetting 5min measurement instance counter
      counter_5m = 0;

      Serial.print("TE 54, ");
      
      flagDontPushAvgTemp = false;
    
      //Ticking over 90m time counter
      epoch_last_90m = epoch_now;
    
      //Adding cumulatively the values of the 90min measurements
      temp_f_sum_90m = temp_f_sum_90m + temp_f_inst_90m;
      humidity_sum_90m = humidity_sum_90m + humidity_inst_90m;
      pid_sum_90m = pid_sum_90m + pid_inst_90m;
      heater_sum_90m = heater_sum_90m + heater_inst_90m;
      
      //Incrementing 90min measurement instance counter
      counter_90m++;

        //Writing values to RTC memory
  RTCMemWrite();

      Serial.print("TE 55, ");
    
      //Sending 90m avg measurements
      if (!flagDontPushAvgTemp) {
        Serial.println("Pushing 90m average to DB:\t");
        Serial.print(temp_f_inst_90m);
        Serial.print(",");
        Serial.print(humidity_inst_90m);
        Serial.print(",");
        Serial.println(counter_5m);
            // print the hour, minute and second:
        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
        Serial.print(':');
        if (((epoch % 3600) / 60) < 10) {
          // In the first 10 minutes of each hour, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ((epoch % 60) < 10) {
          // In the first 10 seconds of each minute, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print(epoch % 60); // print the second
        Serial.print(",Epoch Now");
        Serial.print(",");
        Serial.print(epoch_now);
        Serial.print(",");
        Serial.print("Epoch Last Day");
        Serial.print(",");
        Serial.print(epoch_last_1d);
        Serial.print("Epoch Last 90m");
        Serial.print(",");
        Serial.print(epoch_last_90m);
        Serial.print("Epoch Last 5m");
        Serial.print(",");
        Serial.print(epoch_last_5m);
        Serial.print("Epoch Difference Now to Last 90m");
        Serial.print(",");
        Serial.print(epoch_now - epoch_last_90m);
        Serial.println();

      Serial.print("TE 56, ");

        PushTempToDynamoDB(temp_f_inst_90m, humidity_inst_90m, pid_inst_90m, heater_inst_90m, TABLE_NAME4, HASH_KEY_NAME4, HASH_KEY_VALUE4, RANGE_KEY_NAME4);
  yield();

      Serial.print("TE 57, ");

      }
     
  }

    if (epoch_now - epoch_last_1d > 86400) {
      
      Serial.print("TE 58, ");
      
      //Averaging 1 day measurements and setting them to a 1 day variable
      temp_f_inst_1d = temp_f_sum_90m / counter_90m;
      humidity_inst_1d = humidity_sum_90m / counter_90m; 
      pid_inst_1d = pid_sum_90m / counter_90m;
      heater_inst_1d = heater_sum_90m / counter_90m;

      //Resetting 90min sums
      temp_f_sum_90m = 0;
      humidity_sum_90m = 0;
      pid_sum_90m = 0;
      heater_sum_90m = 0;
    
      //Resetting 90min measurement instance counter
      counter_90m = 0;
      
      flagDontPushAvgTemp = false;
    
      //Ticking over 1d time counter
      epoch_last_1d = epoch_now;
    
//      //Adding cumulatively the values of the 1 day measurements ( no need here)
//      temp_f_sum_1d = temp_f_sum_1d + temp_f_inst_1d;
//      humidity_sum_1d = humidity_sum_1d + humidity_inst_1d;
//      pid_sum_1d = pid_sum_1d + pid_inst_1d;
//      heater_sum_1d = heater_sum_1d + heater_inst_1d;
      
//      //Incrementing 1 day measurement instance counter ( no need here)
//      1d_counter++;

        //Writing values to RTC memory
  RTCMemWrite();

      Serial.print("TE 59, ");
    
      //Sending 1d avg measurements
      if (!flagDontPushAvgTemp) {
         Serial.println("Pushing 1d average to DB:\t");
        Serial.print(temp_f_inst_1d);
        Serial.print(",");
        Serial.print(humidity_inst_1d);
        Serial.print(",");
        Serial.println(counter_90m);
            // print the hour, minute and second:
        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
        Serial.print(':');
        if (((epoch % 3600) / 60) < 10) {
          // In the first 10 minutes of each hour, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ((epoch % 60) < 10) {
          // In the first 10 seconds of each minute, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print(epoch % 60); // print the second
        Serial.print(",Epoch Now");
        Serial.print(",");
        Serial.print(epoch_now);
        Serial.print(",");
        Serial.print("Epoch Last Day");
        Serial.print(",");
        Serial.print(epoch_last_1d);
        Serial.print("Epoch Last 90m");
        Serial.print(",");
        Serial.print(epoch_last_90m);
        Serial.print("Epoch Last 5m");
        Serial.print(",");
        Serial.print(epoch_last_5m);
        Serial.print("Epoch Difference Now to Last 90m");
        Serial.print(",");
        Serial.print(epoch_now - epoch_last_90m);
        Serial.println();

      Serial.print("TE 60, ");

        PushTempToDynamoDB(temp_f_inst_1d, humidity_inst_1d, pid_inst_1d, heater_inst_1d, TABLE_NAME5, HASH_KEY_NAME5, HASH_KEY_VALUE5, RANGE_KEY_NAME5);
  yield();

      Serial.print("TE 61, ");

      }

  }

//Currently not using the counter overflow watchdog - NTP time checking seems to be sufficient 
//  if ((counter_20s > 20) || (counter_5m > 20) || (counter_90m > 20)) {
//    Serial.println("Too many measurements for time period!");
//    Serial.println("20sec measurements (per 5 min) now stored: ");
//    Serial.println(counter_20s);
//    Serial.println("5m measurements (per 90 min) now stored: ");
//    Serial.println(counter_5m);
//    Serial.println("90m measurements (per 1 day) now stored: ");
//    Serial.println(counter_90m);
//    flagDontPushAvgTemp = true;
//    Serial.println("Halting publish of avg measurements to DynamoDB tables");
//  }

}

void getWebsiteHeaterState(int *WebsiteHeaterState, GetItemOutput getItemOutput) {
  char szC[6];

  Serial.print("TE 7, ");

  /* Get the "item" from the getItem output. */
  MinimalMap < AttributeValue > attributeMap = getItemOutput.getItem();
  AttributeValue av;

  Serial.print("TE 8, ");
      
  attributeMap.get("WebsiteHeaterState", av);
  *WebsiteHeaterState = atoi(av.getS().getCStr());

  delay(10);
}

void awsGetWebsiteHeaterState(int *WebsiteHeaterState) {

  Serial.print("TE 31, ");

  /* Pull Heater State. */
  AttributeValue id;
  id.setS(HASH_KEY_VALUE2);
  rangeKey.setN(RANGE_KEY_VALUE2);

  Serial.print("TE 32, ");

  MinimalKeyValuePair < MinimalString, AttributeValue > pair1(HASH_KEY_NAME2, id);
  MinimalKeyValuePair < MinimalString, AttributeValue > pair2(RANGE_KEY_NAME2, rangeKey);
  MinimalKeyValuePair<MinimalString, AttributeValue> keyArray[] = { pair1, pair2 };
  getItemInput.setKey(MinimalMap < AttributeValue > (keyArray, KEY_SIZE));
  
  MinimalString attributesToGet[] = { "WebsiteHeaterState" };
  getItemInput.setAttributesToGet(MinimalList < MinimalString > (attributesToGet, 1));

  // Set Table Name
  getItemInput.setTableName(TABLE_NAME2);

  Serial.print("TE 33, ");

  // Perform getItem and check for errors.
  GetItemOutput getItemOutput = ddbClient.getItem(getItemInput, actionError);
  yield();

  Serial.print("TE 34, ");  

  switch (actionError) {

  Serial.print("TE 35, ");
    
    case NONE_ACTIONERROR:
      getWebsiteHeaterState(WebsiteHeaterState, getItemOutput);
      yield();
      break;

    case INVALID_REQUEST_ACTIONERROR:
      Serial.print("ERROR: ");
      Serial.println(getItemOutput.getErrorMessage().getCStr());
      break;
    case MISSING_REQUIRED_ARGS_ACTIONERROR:
      Serial.println("ERROR: Required arguments were not set for GetItemInput");
      break;
    case RESPONSE_PARSING_ACTIONERROR:
      Serial.println("ERROR: Problem parsing http response of GetItem\n");
      break;
    case CONNECTION_ACTIONERROR:
      Serial.println("ERROR: Connection problem");
      break;
  }
  
  Serial.print("TE 36, ");
}

void PushTempToDynamoDB(float temp_f_update, float humidity_update, float pid_update, float heater_update, const char* T_N, const char* HK_N, const char* HK_V, const char* RK_N) {

  Serial.print("TE 37, ");
  
    /* Create an Item. */
  AttributeValue id;
  id.setS(HK_V);
  AttributeValue timest;

  //Calculating my own time value for AWS given the last NTP time
  time_t awstime = epoch + (awsoffset - millis()) / 1000; 
  char awstimestr[14];
  sprintf(awstimestr, "%04d%02d%02d%02d%02d%02d", year(awstime), month(awstime), day(awstime), hour(awstime), minute(awstime), second(awstime));
  Serial.print(awstimestr);
  timest.setN(awstimestr);
//  timest.setN(dateTimeProvider.getDateTime()); //Now using my own time system
//  yield();

  /* Create an AttributeValue for 'temp', convert the number to a
     string (AttributeValue object represents numbers as strings), and
     use setN to apply that value to the AttributeValue. */

  int d1 = temp_f_update;
  float f2 = temp_f_update - d1;
  int d2 = f2 * 100 + 1;

  char numberBuffer[20];
  AttributeValue tempAttributeValue;
  snprintf(numberBuffer, 20, "%d.%02d", d1, d2);
  tempAttributeValue.setN(numberBuffer);

  //AttributeValue for PID output
  int d3 = pid_update;
  float f3 = pid_update - d3;
  int d4 = f3 * 100 + 1;

  char pidBuffer[20];
  AttributeValue pidAttributeValue;
  snprintf(pidBuffer, 20, "%d.%02d", d3, d4);
  pidAttributeValue.setN(pidBuffer);

  //AttributeValue for heater output
  int d7 = heater_update;
  float f5 = heater_update - d7;
  int d8 = f5 * 100 + 1;

  char heaterbuffer[20];
  AttributeValue heaterOnState;
  snprintf(heaterbuffer, 20, "%d.%02d", d7, d8);
  heaterOnState.setN(heaterbuffer);

  //AttributeValue for humidity
  int d5 = humidity_update;
  float f4 = humidity_update - d5;
  int d6 = f4 * 100 + 1;

  char humidityBuffer[20];
  AttributeValue humidityAttributeValue;
  snprintf(humidityBuffer, 20, "%d.%02d", d5, d6);
  humidityAttributeValue.setN(humidityBuffer);

  /* Create the Key-value pairs and make an array of them. */
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att1(HK_N, id);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att2(RK_N, timest);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att3("temp", tempAttributeValue);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att4("pidOutput", pidAttributeValue);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att5("heaterState", heaterOnState);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att6("humidity", humidityAttributeValue);
  MinimalKeyValuePair<MinimalString, AttributeValue> itemArray[] = { att1,
                                                                     att2, att3, att4, att5, att6
                                                                   };

  Serial.print("TE 38, ");

  if (!isnan(temp_f)) {
    
    /* Set values for putItemInput. */
    putItemInput.setItem(MinimalMap < AttributeValue > (itemArray, 6));
    putItemInput.setTableName(T_N);
    
  Serial.print("TE 39, ");
  
    /* Perform putItem and check for errors. */

    PutItemOutput putItemOutput = ddbClient.putItem(putItemInput,
                                  actionError);
  
   Serial.print("TE 39.5, "); 

    yield();

  Serial.print("TE 40, ");
    
    switch (actionError) {
      case NONE_ACTIONERROR:
//        Serial.println("PutItem succeeded!");
        break;
      case INVALID_REQUEST_ACTIONERROR:
        Serial.print("ERROR: Invalid request");
        Serial.println(putItemOutput.getErrorMessage().getCStr());
        break;
      case MISSING_REQUIRED_ARGS_ACTIONERROR:
        Serial.println(
          "ERROR: Required arguments were not set for PutItemInput");
        break;
      case RESPONSE_PARSING_ACTIONERROR:
        Serial.println("ERROR: Problem parsing http response of PutItem");
        break;
      case CONNECTION_ACTIONERROR:
        Serial.println("ERROR: Connection problem");
        break;
    }
    /* wait to not double-record */
  }
    Serial.print("DUBRECWAIT, ");

  delay(2000);

  Serial.print("PDUBRECWAIT, ");
  
}



//Helper function to write the long-running data to RTC memory
void RTCMemWrite() {
  
  rtcData.t5 = temp_f_sum_5m;    
  rtcData.t90 = temp_f_sum_90m;
      
  rtcData.u5 = humidity_sum_5m;    
  rtcData.u90 = humidity_sum_90m;
      
  rtcData.p5 = pid_sum_5m;    
  rtcData.p90 = pid_sum_90m;
      
  rtcData.h5 = heater_sum_5m;    
  rtcData.h90 = heater_sum_90m;

  rtcData.c5 = counter_5m;
  rtcData.c90 = counter_90m;

  rtcData.initial = epoch_initial;
  rtcData.last90 = epoch_last_90m;
  rtcData.last1d = epoch_last_1d;

    // Update CRC32 of data
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
  // Write struct to RTC memory
  if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("Write: ");
    printMemory();
    Serial.println();
  }
  
}

void updateOutput() {
  unsigned long now2 = millis();

  //Don't compute the PID output if temp is invalid (will give an overflow as the PID output)
  if (!isnan(temp_f)) {
    myPID.Compute();
  }

  //Shift the window once past 30min
  if (now2 - windowStartTime > windowSize) {
    windowStartTime += windowSize;
  }

  //Heater control updates based on setting state and PID output - you can see at the start that this was based on the RF switch/outlet system, given the 'Outlet' helper function names
  if ((WebsiteHeaterState == 1) && (stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Manual Control set to Off & Heater is On - Turning it Off");
    OffOutlet();
    yield();
  }
  else if ((WebsiteHeaterState == 3) && (!stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Manual Control set to On & Heater is Off - Turning it On");
    OnOutlet();
    yield();
  }
  //I need to investigate whether the thermostat will turn the heater back on during a cycle that it has already turned it off within - ideally, the heater should only come on at the beginning of the window - I could set an additional 'HasTurnedOff' bool if need be to fix this if it's an issue
  else if ((WebsiteHeaterState == 2) && (pidOutput * windowSize > ((now2 - windowStartTime) * 100)) && (!stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Automatic Control - Beginning of Window - Turning On");
    OnOutlet();
    yield();
  }
  else if ((WebsiteHeaterState == 2) && (pidOutput * windowSize < ((now2 - windowStartTime) * 100)) && (stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Automatic Control - Turning Off");
    OffOutlet();
    yield();
  }
}


//Flash LED and turn heater on - transistor setting read before and after setting it for debugging purposes
void OnOutlet() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting before evaluation: ");
  Serial.println(transistorGate);
  delay(100);
  digitalWrite(transistorPin, HIGH);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting after evaluation: ");
  Serial.println(transistorGate);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Turning On Heater!");
  //Currently not using the RF switch functionality
  //  mySwitch.send("000101010001110100000011");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, LOW);
  //  mySwitch.send("000101010001110100000011");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
}

//Flash LED and turn heater off - transistor setting read before and after setting it for debugging purposes
void OffOutlet() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting before evaluation: ");
  Serial.println(transistorGate);
  delay(100);
  digitalWrite(transistorPin, LOW);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting after evaluation: ");
  Serial.println(transistorGate);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Turning Off Heater!"); //
  //Currently not using the RF switch functionality
  //  digitalWrite(LED_BUILTIN, LOW);
  //  mySwitch.send("000101010001110100001100");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, LOW);
  //  mySwitch.send("000101010001110100001100");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
}
