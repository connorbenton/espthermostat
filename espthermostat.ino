#include <PID_v1.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "Esp8266AWSImplementations.h"
#include "AmazonDynamoDBClient.h"
#include "AWSFoundationalTypes.h"
#include "keys.h"

#include <DHT.h>
#include <Wire.h>
//Not using a LCD screen anymore - deprecated
//#include <LiquidCrystal_I2C.h>
#include <RCSwitch.h>

// RTC fields
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
//  byte data[508];
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


String inData;

//int sendPin = 2;
int transistorPin = 14;
int incomingByte;  // a variable to read incoming serial data into
//int ledPin = 13; // the pin that the LED is attached to
//int cycle;
double setPoint = 69.0;
double pidOutput, currentWindowPidOutput = 0;
int transistorGate;

float temp_f_sum_20s, temp_f_sum_5m, temp_f_sum_90m = 0;
float humidity_sum_20s, humidity_sum_5m, humidity_sum_90m = 0;
float pid_sum_20s, pid_sum_5m, pid_sum_90m = 0;
float heater_sum_20s, heater_sum_5m, heater_sum_90m = 0;

float temp_f_inst_5m, humidity_inst_5m, pid_inst_5m, heater_inst_5m = 0;
float temp_f_inst_90m, humidity_inst_90m, pid_inst_90m, heater_inst_90m = 0;
float temp_f_inst_1d, humidity_inst_1d, pid_inst_1d, heater_inst_1d = 0;

float heater_now;

// don't think I need these if the arrays themselves are stored in EEPROM - actually now RTC memory
// float reset_humidity_5m, reset_humidity_90m, reset_temp_f_5m, reset_temp_f_90m, reset_pid_sum_5m, reset_pid_sum_90m, reset_heater_inst_5m, reset_heater_inst_90m = 0;

int counter_20s = 0;
int counter_5m = 0;
int counter_90m = 0;

//NTP server variable section
unsigned int localPort = 2390;
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "0.north-america.pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

unsigned long epoch, epoch_initial, epoch_now, epoch_last_5m, epoch_last_90m, epoch_last_1d;

bool flagDontPushAvgTemp = false; 

// PID tuning
double KP = 25;  // X degrees out = 100% heating
double KI = 0.028; // X% per degree per minute
double KD = 0;   // Not yet used
unsigned long windowSize = 1800000; // 30 minutes (ish)

unsigned long windowStartTime;
boolean stateVariable = false;
//int heaterOn;


#define DELAYSHORT 160
#define DELAYLONG  500
#define VIRTUAL_PIN V1
#define VIRTUAL_PIN V2
#define VIRTUAL_PIN V3
#define REST      1000

//AWS constants
const char* AWS_REGION = "us-west-1";
const char* AWS_ENDPOINT = "amazonaws.com";

// Constants describing DynamoDB table and values being used
const char* TABLE_NAME1 = "Temperatures_20_sec";
const char* HASH_KEY_NAME1 = "Id";
const char* HASH_KEY_VALUE1 = "Thermostat_01"; // Our sensor ID, to be REPLACED in case of multiple sensors.
const char* RANGE_KEY_NAME1 = "Date";

const char* TABLE_NAME3 = "Temperatures_5_min";
const char* HASH_KEY_NAME3 = "Id";
const char* HASH_KEY_VALUE3 = "Thermostat_01"; // Our sensor ID, to be REPLACED in case of multiple sensors.
const char* RANGE_KEY_NAME3 = "Date";

const char* TABLE_NAME4 = "Temperatures_90_min";
const char* HASH_KEY_NAME4 = "Id";
const char* HASH_KEY_VALUE4 = "Thermostat_01"; // Our sensor ID, to be REPLACED in case of multiple sensors.
const char* RANGE_KEY_NAME4 = "Date";

const char* TABLE_NAME5 = "Temperatures_1_day";
const char* HASH_KEY_NAME5 = "Id";
const char* HASH_KEY_VALUE5 = "Thermostat_01"; // Our sensor ID, to be REPLACED in case of multiple sensors.
const char* RANGE_KEY_NAME5 = "Date";

const char* TABLE_NAME2 = "HeaterManualState";
const char* HASH_KEY_NAME2 = "Id";
const char* HASH_KEY_VALUE2 = "Website"; // Our sensor ID, to be REPLACED in case of multiple sensors.
const char* RANGE_KEY_VALUE2 = "1";
const char* RANGE_KEY_NAME2 = "Date";
static const int KEY_SIZE = 2;

static int WebsiteHeaterState;

Esp8266HttpClient httpClient;
Esp8266DateTimeProvider dateTimeProvider;

/* Reused objects. */
GetItemInput getItemInput;
PutItemInput putItemInput;
AttributeValue hashKey;
AttributeValue rangeKey;
ActionError actionError;

AmazonDynamoDBClient ddbClient;

//LiquidCrystal_I2C  lcd(0x3F, 4, 5);

RCSwitch mySwitch = RCSwitch();

DHT dht(5, DHT22, 20);
double humidity, temp_f = 0;

PID myPID(&temp_f, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

// Generally, you should use "unsigned long" for variables that hold time
unsigned long lastTimeUpdate, lastTempUpdate, lastOutputUpdate, lastHeaterRefresh = 0;

int value = 0;
long rssi;
IPAddress ip;

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial initialized...");

  // Set pins for transistor and LED blink
  pinMode(transistorPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize Wifi - ssid and password taken from keys.h
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
  
  //Initialize PID limits
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  //Take first temp reading and compute PID for initial output
  temp_f = dht.readTemperature(true);
  if (!isnan(temp_f)) {
    yield();
    myPID.Compute();
  }
  currentWindowPidOutput = pidOutput;

  Serial.print("Initial PID output: ");
  Serial.println(currentWindowPidOutput);

  //Initialize DynamoDB client
  ddbClient.setAWSRegion(AWS_REGION);
  ddbClient.setAWSEndpoint(AWS_ENDPOINT);
  ddbClient.setAWSSecretKey(awsSecKey);
  ddbClient.setAWSKeyID(awsKeyID);
  ddbClient.setHttpClient(&httpClient);
  ddbClient.setDateTimeProvider(&dateTimeProvider);

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
void yieldEspCPUSetup(int x) {
  delay(100);
  ESP.wdtFeed();
  yield();
  Serial.print("SetupDebug");
  Serial.print(x);
  Serial.print(",");
}

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

void getWebsiteHeaterState(int *WebsiteHeaterState, GetItemOutput getItemOutput) {
  char szC[6];
  // Serial.println("GetItem succeeded!");
  yieldEspCPUTemp(7);

  /* Get the "item" from the getItem output. */
  MinimalMap < AttributeValue > attributeMap = getItemOutput.getItem();
  AttributeValue av;

  yieldEspCPUTemp(8);
      
  // Get the rgb values and set the led with them. //
  attributeMap.get("WebsiteHeaterState", av);
  *WebsiteHeaterState = atoi(av.getS().getCStr());
//  Serial.print("Website Heater State:   ");
//  Serial.println(*WebsiteHeaterState);

  //        attributeMap.get("G", av);
  //        *G = atoi(av.getS().getCStr());
  //        Serial.print("Green value read: ");
  //        Serial.println(*G);
  //
  //        attributeMap.get("B", av);
  //        *B = atoi(av.getS().getCStr());
  //        Serial.print("Blue value read:  ");
  //        Serial.println(*B);

  delay(10);
  //        Serial.print("\n\n");
}


void loop() {
  unsigned long now = millis();

    Serial.print("ML9, ");
//  yieldEspCPU(9);

  
  if ( now - lastTimeUpdate > 10000) {
    Serial.println();
    Serial.print("Sending NTP request");
    Serial.println();
//    yieldEspCPU(10);
    sendNTPpacket(timeServerIP);
//    getTheTime();
//    yieldEspCPU(11);
//    Serial.println();
//    Serial.println("Time Successful");
//    Serial.println();
    lastTimeUpdate = now;
  }

  uint32_t time = getTheTime();
  if (time) {
    Serial.print("X");
    Serial.print(time);
    epoch = time;
    Serial.print(epoch);
    
    //Setting inital average update times if they do not exist (RTC memory)
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

    Serial.print("ML12, ");
//  yieldEspCPU(12);
  
  if ( now - lastTempUpdate > 20000 ) {
    //    String MyIp;
    //    String Both;
    //    MyIp =  "IP: " + String(WiFi.localIP()[0]) + "." + String(WiFi.localIP()[1]) + "." + String(WiFi.localIP()[2]) + "." + String(WiFi.localIP()[3]);
    //    String MyWiFi;
    //    MyWiFi = "WiFi: " + String(WiFi.RSSI());
    //    Serial.println(MyIp);
    //    Serial.println(MyWiFi);
    Serial.println();
    Serial.print("Get Temp-");
    Serial.print("ML13, ");
//    yieldEspCPU(13);
    Serial.println();
    gettemperature();
    Serial.println();
    Serial.print("ML14, ");
//    yieldEspCPU(14);
    Serial.println("Temp Successful");
    //    Serial.print("Sending temperature:");
    //    Serial.println(temp_f);
    //    Serial.print("Sending humidity:");
    //    Serial.println(humidity);
    lastTempUpdate = now;
  }
  
    Serial.print("ML15, ");
//  yieldEspCPU(15);
  
  if ( now - lastOutputUpdate > 20000 ) {
    Serial.println();
    Serial.print("Update Output-");
    Serial.print("ML16, ");
//    yieldEspCPU(16);
    updateOutput();
    Serial.print("ML17, ");
//    yieldEspCPU(17);
    Serial.println("Successful");
    Serial.println();
    lastOutputUpdate = now;
  }

//  delay(1000);

}

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

void printMemory() {
//  char buf[3];
//  for (int i = 0; i < sizeof(rtcData); i++) {
//    sprintf(buf, "%02X", rtcData.data[i]);
//    Serial.print(buf);
//    if ((i + 1) % 32 == 0) {
//      Serial.println();
//    }
//    else {
//      Serial.print(" ");
//    }
//  }
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

uint32_t getTheTime() { // Get the NTP time for averaging over the longer intervals
  delay(1000);
  
  if (udp.parsePacket() == 0) {
    return 0;
  }
//  int cb = udp.parsePacket();
   
  udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

  Serial.print("TD22, ");
//  yieldEspCPUTime(22);
  
  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  //  Serial.print("Seconds since Jan 1 1900 = " );
  //  Serial.println(secsSince1900);
  
  Serial.print("TD23, ");
  Serial.print(secsSince1900);
//  yieldEspCPUTime(23);

  // now convert NTP time into everyday time:
  //  Serial.print("Unix time = ");
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  unsigned long UNIXTime = secsSince1900 - seventyYears;
  return UNIXTime;
  // print Unix time:
//        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
//        Serial.print(':');
//        if (((epoch % 3600) / 60) < 10) {
//          // In the first 10 minutes of each hour, we'll want a leading '0'
//          Serial.print('0');
//        }
//        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
  yieldEspCPUTime(24);


//      Serial.println("Seconds since 1900, now");
//      Serial.println(epoch_now);
//      Serial.println("Seconds since 1900, last 5min check");
//      Serial.println(epoch_last_5m);
//      Serial.println("Seconds between");
//      Serial.println(epoch_now - epoch_last_5m);
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
//  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
//  yieldEspCPUTime(25);
    Serial.print("TD25, ");
  
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
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
//  yieldEspCPUTime(27);
//  ESP.wdtDisable();
  udp.beginPacket(address, 123); //NTP requests are to port 123
//  ESP.wdtEnable(0);
//    Serial.print("28, ");
    Serial.print("TD27, ");
//  yieldEspCPUTime(28);
//  ESP.wdtDisable();
  udp.write(packetBuffer, NTP_PACKET_SIZE);
//  ESP.wdtEnable(0);
//    Serial.print("29, ");
    Serial.print("TD28, ");
//  yieldEspCPUTime(29);
//  ESP.wdtDisable();
  udp.endPacket();
  yield();
//  ESP.wdtEnable(0);
    Serial.print("TD29, ");
//  yieldEspCPUTime(30);
}


void updateOutput() {
  unsigned long now2 = millis();
//  yieldEspCPU();
//
//  Serial.print("Before Computation: ");
//  Serial.print("temp_f: ");
//  Serial.print(temp_f);
//  Serial.print(" pidOutput: ");
//  Serial.print(pidOutput);
//  Serial.print(" setPoint: ");
//  Serial.print(setPoint);
//  Serial.print(" KP: ");
//  Serial.print(KP);
//  Serial.print(" KI: ");
//  Serial.print(KI);
//  Serial.print(" KD: ");
//  Serial.println(KD);

  //  &temp_f, &pidOutput, &setPoint, KP, KI, KD,

  if (!isnan(temp_f)) {
    myPID.Compute();
  }

  //  &temp_f, &pidOutput, &setPoint, KP, KI, KD,

//  Serial.print("After  Computation: ");
//  Serial.print("temp_f: ");
//  Serial.print(temp_f);
//  Serial.print(" pidOutput: ");
//  Serial.print(pidOutput);
//  Serial.print(" setPoint: ");
//  Serial.print(setPoint);
//  Serial.print(" KP: ");
//  Serial.print(KP);
//  Serial.print(" KI: ");
//  Serial.print(KI);
//  Serial.print(" KD: ");
//  Serial.println(KD);

  //  Serial.print("Current PID output: ");
  //  Serial.println(pidOutput);

  if (now2 - windowStartTime > windowSize) {
    //time to shift the window
    windowStartTime += windowSize;
    currentWindowPidOutput = pidOutput;
  }



  //  Serial.print("Window Start Time: ");
  //  Serial.println(windowStartTime);
//  Serial.print("Now2 - Window Start Time: ");
//  Serial.println(now2 - windowStartTime);
  //
  //  Serial.print("Now2 - Window Start Time * 100: ");
  //  Serial.println((now2 - windowStartTime) * 100);
  //  Serial.print("currentwPidOutput * windowSize: ");
  //  Serial.println(currentWindowPidOutput * windowSize);
  //
//  Serial.print("Switch setting before evaluation: ");
//  Serial.println(stateVariable);


  //  Serial.print("test: ");
  //  Serial.print(currentWindowPidOutput * windowSize > ((now2 - windowStartTime) * 100));
  //  Serial.print(stateVariable);
  //  Serial.print(!stateVariable);
  //  Serial.println((currentWindowPidOutput * windowSize > ((now2 - windowStartTime) * 100)) && (!stateVariable));

  if ((WebsiteHeaterState == 1) && (stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Manual Control set to Off & Switch is On - Turning it Off");
    OffOutlet();
  }
  else if ((WebsiteHeaterState == 3) && (!stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Manual Control set to On & Switch is Off - Turning it On");
    OnOutlet();
  }
  else if ((WebsiteHeaterState == 2) && (currentWindowPidOutput * windowSize > ((now2 - windowStartTime) * 100)) && (!stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Automatic Control - Turning On");
    OnOutlet();
  }
  else if ((WebsiteHeaterState == 2) && (currentWindowPidOutput * windowSize < ((now2 - windowStartTime) * 100)) && (stateVariable))
  {
    stateVariable = !stateVariable;
    Serial.println("Automatic Control - Turning Off");
    OffOutlet();
  }


//  Serial.print("Switch setting after evaluation : ");
//  Serial.println(stateVariable);


  //  // Every 400 cycles (about 8 seconds) refresh the heater state
  //  if( now2 - lastHeaterRefresh > 8000 ) {
  //    setHeaterState(heaterOn);
  //    lastHeaterRefresh = now2;
  //  }
}

//void setHeaterState(boolean desiredState) {
//  if (desiredState = true){
//    OnOutlet();
//  }
//  else
//  {
//    OffOutlet();
//  }
//}

void awsGetWebsiteHeaterState(int *WebsiteHeaterState) {

  Serial.print("TE 31, ");
//  yieldEspCPUTemp(31);
  /* Pull Heater State. */
  AttributeValue id;
  id.setS(HASH_KEY_VALUE2);
  rangeKey.setN(RANGE_KEY_VALUE2);

  Serial.print("TE 32, ");
//  yieldEspCPUTemp(32);

  MinimalKeyValuePair < MinimalString, AttributeValue > pair1(HASH_KEY_NAME2, id);
  MinimalKeyValuePair < MinimalString, AttributeValue > pair2(RANGE_KEY_NAME2, rangeKey);
  MinimalKeyValuePair<MinimalString, AttributeValue> keyArray[] = { pair1, pair2 };
  getItemInput.setKey(MinimalMap < AttributeValue > (keyArray, KEY_SIZE));

//  yieldEspCPUTemp();
  
  MinimalString attributesToGet[] = { "WebsiteHeaterState" };
  getItemInput.setAttributesToGet(MinimalList < MinimalString > (attributesToGet, 1));

  // Set Table Name
  getItemInput.setTableName(TABLE_NAME2);

  Serial.print("TE 33, ");
//  yieldEspCPUTemp(33);

  // Perform getItem and check for errors.
//  ESP.wdtDisable();
  yieldEspCPU(62);
  GetItemOutput getItemOutput = ddbClient.getItem(getItemInput, actionError);
  yieldEspCPU(63);
//  ESP.wdtEnable(0);

  Serial.print("TE 34, ");  
//  yieldEspCPUTemp(34);

  ESP.wdtDisable();
  switch (actionError) {

  Serial.print("TE 35, ");
//    yieldEspCPUTemp(35);
    
    case NONE_ACTIONERROR:
      getWebsiteHeaterState(WebsiteHeaterState, getItemOutput);
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
//  yieldEspCPUTemp(36);
  ESP.wdtEnable(0);
}

void PushTempToDynamoDB(float temp_f_update, float humidity_update, float pid_update, float heater_update, const char* T_N, const char* HK_N, const char* HK_V, const char* RK_N) {

  Serial.print("TE 37, ");
//  yieldEspCPUTemp(37);
  
    /* Create an Item. */
  AttributeValue id;
  id.setS(HK_V);
  AttributeValue timest;
  timest.setN(dateTimeProvider.getDateTime());

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


//  yieldEspCPUTemp();


  int d3 = pid_update;
  float f3 = pid_update - d3;
  int d4 = f3 * 100 + 1;

  char pidBuffer[20];
  AttributeValue pidAttributeValue;
  snprintf(pidBuffer, 20, "%d.%02d", d3, d4);
  pidAttributeValue.setN(pidBuffer);


//  yieldEspCPUTemp();


  int d7 = heater_update;
  float f5 = heater_update - d7;
  int d8 = f5 * 100 + 1;

  char heaterbuffer[20];
  AttributeValue heaterOnState;
  snprintf(heaterbuffer, 20, "%d.%02d", d7, d8);
  heaterOnState.setN(heaterbuffer);


//  yieldEspCPUTemp();

  int d5 = humidity_update;
  float f4 = humidity_update - d5;
  int d6 = f4 * 100 + 1;

  char humidityBuffer[20];
  AttributeValue humidityAttributeValue;
  snprintf(humidityBuffer, 20, "%d.%02d", d5, d6);
  humidityAttributeValue.setN(humidityBuffer);

//  char piddebugbuffer[100];
//  AttributeValue PIDdebug;
//  snprintf(piddebugbuffer, 100, "temp_f: %f pidOutput: %f setPoint: %f KP: %f KI: %f KD: %f", temp_f, pidOutput, setPoint, KP, KI, KD);
//  PIDdebug.setS(piddebugbuffer);
//
//
//  yieldEspCPU();

//  Serial.println(numberBuffer);
//  Serial.println(pidBuffer);
//  Serial.println(heaterbuffer);
//  Serial.println(humidityBuffer);
  

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
//  yieldEspCPUTemp(38);

  ESP.wdtDisable();
  ESP.wdtEnable(0);
  if (!isnan(temp_f)) {
    
    /* Set values for putItemInput. */
    putItemInput.setItem(MinimalMap < AttributeValue > (itemArray, 6));
    putItemInput.setTableName(T_N);
    
  Serial.print("TE 39, ");
//    yieldEspCPUTemp(39);
    /* Perform putItem and check for errors. */
//    ESP.wdtDisable();
  Serial.print("TE 60, ");
//    yieldEspCPU(60);
    PutItemOutput putItemOutput = ddbClient.putItem(putItemInput,
                                  actionError);
  Serial.print("TE 61, ");                              
//    yieldEspCPU(61);
//    ESP.wdtEnable(0);
  Serial.print("TE 40, ");
//    yieldEspCPUTemp(40);
    
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
//  ESP.wdtDisable();
  delay(2000);
  Serial.print("PDUBRECWAIT, ");
//  ESP.wdtEnable(0);
  
}

void gettemperature() {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  //   the sensor is bigger than the interval you set, read the sensor
  //   Works better than delay for things happening elsewhere also
  /*unsigned long currentMillis = millis();

    if(currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor
    previousMillis = currentMillis;   */
  Serial.print("TE 41, ");
//  yieldEspCPUTemp(41);
  
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  humidity = dht.readHumidity();          // Read humidity (percent)
  temp_f = dht.readTemperature(true);     // Read temperature as Farenheight
  Serial.print("TE 42, ");
//  yieldEspCPUTemp(42);

  //    // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temp_f)) {
//    Serial.println("Failed to read from DHT sensor! Humidity:");
//    Serial.println(humidity);
//    Serial.println("temp:");
//    Serial.println(temp_f);
    byte hu;
    for (hu = 0; hu < 30; hu++)
    {
      temp_f = dht.readTemperature(true);
      if isnan(temp_f) {
        Serial.print("TE 43, ");
//        yieldEspCPU(43);
        delay(150);
        temp_f = dht.readTemperature(true);
      }
    }
    return;
  }


  //    double TempF;
  //    int Humid;
  //    TempF = double(temp_f);
  //    Humid = int(humidity);

  //    myPID.Compute();

  //  lcd.backlight();
  //  lcd.setCursor(0, 0);
  //  lcd.print("Temp Set  Power");
  //  lcd.setCursor(0, 1);
  //  lcd.print(int(temp_f), 1);
  //  lcd.setCursor(5, 1);
  //  lcd.print(int(setPoint), 1);
  //  lcd.setCursor(10, 1);
  //  lcd.print(pidOutput, 1);
  //  lcd.print("%");
  //  if (stateVariable) {
  //    lcd.setCursor(15, 1);
  //    lcd.print("H");
  //  }

  Serial.print("TE 44, ");
//  yieldEspCPUTemp(44);

//  Serial.print("getWebsiteHeaterState,");
  awsGetWebsiteHeaterState(&WebsiteHeaterState);
//  Serial.print("WebsiteHeaterStateGot,");
  
//  yieldEspCPU();
  
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
//  yieldEspCPUTemp(45);

//  Serial.print("pushTempToDB20s,");
  PushTempToDynamoDB(temp_f, humidity, pidOutput, heater_now, TABLE_NAME1, HASH_KEY_NAME1, HASH_KEY_VALUE1, RANGE_KEY_NAME1);
//  Serial.print("20spushdone,");
  
  
  Serial.print("TE 46, ");
//  yieldEspCPUTemp(46);
  
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
//  yieldEspCPUTemp(47);

  if (epoch_now - epoch_last_5m > 300) { 

      yieldEspCPUTemp(48);

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

      yieldEspCPUTemp(51);
        
      flagDontPushAvgTemp = false;
    
      //Ticking over 5m time counter
      epoch_last_5m = epoch_now;
    
      //Adding cumulatively the values of the 5min measurements
      temp_f_sum_5m = temp_f_sum_5m + temp_f_inst_5m;
      humidity_sum_5m = humidity_sum_5m + humidity_inst_5m;
      pid_sum_5m = pid_sum_5m + pid_inst_5m;
      heater_sum_5m = heater_sum_5m + heater_inst_5m;
      
      //Incrementing 5min measurement instance counter
      counter_5m++;

  //Writing values to RTC memory
  RTCMemWrite();
    
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
        yieldEspCPUTemp(49);
//        Serial.print("pushTempToDB5m,");
        PushTempToDynamoDB(temp_f_inst_5m, humidity_inst_5m, pid_inst_5m, heater_inst_5m, TABLE_NAME3, HASH_KEY_NAME3, HASH_KEY_VALUE3, RANGE_KEY_NAME3);
        yieldEspCPUTemp(50);
      }
  }

//  yieldEspCPU();
  
  if (epoch_now - epoch_last_90m > 5400) {

      yieldEspCPUTemp(52);
      
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

      yieldEspCPUTemp(55);
      
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
        yieldEspCPUTemp(53);
        PushTempToDynamoDB(temp_f_inst_90m, humidity_inst_90m, pid_inst_90m, heater_inst_90m, TABLE_NAME4, HASH_KEY_NAME4, HASH_KEY_VALUE4, RANGE_KEY_NAME4);
        yieldEspCPUTemp(54);
      }
     
  }

//  yieldEspCPU();

    if (epoch_now - epoch_last_1d > 86400) {
      
      yieldEspCPUTemp(56);
      
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

      yieldEspCPUTemp(59);
      
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
        yieldEspCPUTemp(57);
        PushTempToDynamoDB(temp_f_inst_1d, humidity_inst_1d, pid_inst_1d, heater_inst_1d, TABLE_NAME5, HASH_KEY_NAME5, HASH_KEY_VALUE5, RANGE_KEY_NAME5);
        yieldEspCPUTemp(58);
      }

  }

//  yieldEspCPU();
  
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
  
  //  lcd.noBacklight();

  //    setHeaterState(heaterOn);

}


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
  //  mySwitch.send("000101010001110100000011");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, LOW);
  //  mySwitch.send("000101010001110100000011");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
}

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
