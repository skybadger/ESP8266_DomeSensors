/*
  Program to talk to DHT one-wire temp/hum sensor and send output to MQTT topic /skybadger/device/temperature
  Supports web interface on port 80 returning json string

  To do:
  Done: DHT bit works with local device.
  Done: BMP 280 works + BME280 library
  Done :BMP280 is desirable since it also supports humidity for direct dewpoint measurement and has greater precision for temperature.
  DONE: QMC5883L - works with local device. Not tested over long cable.
  Done: Added CMPS03 library - needs testing

  fix all .begin errors to represent their status as to whether present on the bus .
  Fix the size issue that causes the sesnor pack to become unresponsive. I suspect one of the functions is taking tooo long.

  Layout:
  Pin 13 to DHT data
  GPIO 4,2 to SDA
  GPIO 5,0 to SCL
  All 3.3v logic.
*/

#include "DebugSerial.h"
#include <esp8266_peri.h> //register map and access
#include <ESP8266WiFi.h>
#include <Wire.h>         //https://playground.arduino.cc/Main/WireLibraryDetailedReference
#include <Time.h>         //Look at https://github.com/PaulStoffregen/Time for a more useful internal timebase library
#define MQTT_MAX_PACKET_SIZE 300
#include <PubSubClient.h> //https://pubsubclient.knolleary.net/api.html
#include <WiFiUdp.h>
#include <ArduinoJson.h>  //https://arduinojson.org/v5/api/
#include <Math.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <GDBStub.h> //Debugging stub for GDB

//Publishing function definitions
bool publishDHT(void);
bool publishBMP(void);
bool publishHTU(void);
bool publishQMC(void);
bool publishCMP(void);
bool publishHealth(void);

//Select sensors - can also select by explicitly turning off the 'present' flag
int sensorPresence = 0;
int reportFlags = 0;
bool (*pubFuncs[8])(void) = { publishDHT, publishHTU, publishBMP, publishQMC, publishCMP, NULL, NULL, NULL };

#define _DHT11_INCLUDED_    0x01   //Self-detection doesn't work
//#define _HTU21D_INCLUDED_ 0x02
#define _BMP280_INCLUDED_   0x04
#define _QMC5883L_INCLUDED_ 0x08   //Self-detection doesn't work
//#define _CMPS03_INCLUDED_ 0x10
#define _USE_MQTT_

//Ntp dependencies - available from v2.4
#include <time.h>
#include <sys/time.h>
#include <coredecls.h>
#define TZ              0       // (utc+) TZ in hours
#define DST_MN          60      // use 60mn for summer time in some countries
#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)
time_t now; //use as 'gmtime(&now);'

//Wifi and MQTT settings stay out of Git.
#include "skybadger_strings.h"
char* defaultHostname        = "espSEN01";
char* thisID                 = "espSEN01";
char* myHostname             = "espSEN01";

WiFiClient espClient;
PubSubClient client(espClient);
volatile bool callbackFlag = 0;
//Used by MQTT reconnect
volatile bool timerSet  = false;

// Create an instance of the server
// specify the port to listen on as an argument
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer updater;

//Hardware device system functions - reset/restart etc
EspClass device;
uint32_t originalRam;
uint32_t lastRam;
ETSTimer timer, timeoutTimer;
volatile bool newDataFlag = false;
volatile bool timeoutFlag = false;


//Processor time
double usProfileTimeCount = 0;
long int startTime; //Time when profile was activated
long int currentTime;//Time of last profile index update
//long int nowTime = system_get_time();

void onTimer(void);
void syncTime();
String& getTimeAsString(String& );
String& getTimeAsString2(String& );
void callback(char* topic, byte* payload, unsigned int length) ;
void reconnect( void );
void reconnectNB( void );

//Web Handler function definitions
void handleRoot(void);
void handleNotFound();
void handleBearingGet();
void handleOffsetsPut();

//Presence markers
bool dht11Present = false;
bool htuPresent = false;
bool bmp280Present = false;
bool qCompassPresent = false;
bool cCompassPresent = false;

#if defined _DHT11_INCLUDED_
//Humidity, tempa
#include "DHTesp.h"
float humidity = 0.0F;
float dewpoint = 0.0F;
float aTemperature = 0.0F;
DHTesp dht;
#endif

#if defined _HTU21D_INCLUDED_
#include "HTU21D.h"
HTU21D htu;
float htuHumidity = 0.0F;
float htuTemperature = 0.0F;
#endif

#if defined _BMP280_INCLUDED_
//Pressure, tempb,
//BMP 280 device uses sensor library
#include "i2c.h"
#include "i2c_BMP280.h"
BMP280 bmp280;
float pressure = 0.0F;
float bTemperature = 0.0f;
int32_t iTemperature = 0;
#endif

//Magnetism & bearing, tempc
//void handleBearing(void);
//void handleBearingGet(void );

#if defined _QMC5883L_INCLUDED_
#include <QMC5883L.h>
QMC5883L qCompass;
int error = 0;
Vector raw;
float cTemperature = 0.0f;
float qBearing = 0.0f;
#endif

#if defined _CMPS03_INCLUDED_
/*Alternative compass uses DevaSys 5v compass device - also I2C */
#include "CMPS03.h"
CMPS03 cCompass( (uint8_t) CMPS03_ADDRESS, Wire );
float cBearing = 0.0f;
#endif

void setup_wifi()
{
  int zz = 0;
  WiFi.hostname( myHostname );
  WiFi.mode(WIFI_STA);

  //Experimental
  //WiFi.setOutputPower( 20.5F );//full power WiFi
  //WiFi.persistent( false );

  WiFi.begin(ssid1, password1 );
  Serial.print("Searching for WiFi..");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    zz++;
    if (zz >= 400) 
    {
      device.restart();
    }    
  }

  Serial.println("WiFi connected");
  Serial.printf("SSID: %s, Signal strength %i dBm \n\r", WiFi.SSID().c_str(), WiFi.RSSI() );
  Serial.printf("Hostname: %s\n\r",       WiFi.hostname().c_str() );
  Serial.printf("IP address: %s\n\r",     WiFi.localIP().toString().c_str() );
  Serial.printf("DNS address 0: %s\n\r",  WiFi.dnsIP(0).toString().c_str() );
  Serial.printf("DNS address 1: %s\n\r",  WiFi.dnsIP(1).toString().c_str() );
  delay(5000);

  //Setup sleep parameters
  //WiFi.mode(WIFI_NONE_SLEEP);
  wifi_set_sleep_type(NONE_SLEEP_T);

}

void setup()
{
  float val = 0.0F;

  Serial.begin( 115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println(F("ESP starting."));
  gdbstub_init();

  //Start NTP client
  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");

  //Setup defaults first - via EEprom.
  //TODO

  //Pins mode and direction setup for i2c on ESP8266-01
  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);

  //I2C setup SDA pin 0, SCL pin 2 on ESP-01
  //I2C setup SDA pin 5, SCL pin 4 on ESP-12
  Wire.begin(0, 2); //Give Wire time to setup by placing it before WiFi
  Wire.setClock(100000 );//100KHz target rate

  // Connect to wifi
  setup_wifi();

  Serial.println("Pins setup & interrupts attached.");

  //Open a connection to MQTT
  client.setServer( mqtt_server, 1883 );
  client.connect( thisID, pubsubUserID, pubsubUserPwd ); 
  
  //Create a timer-based callback that causes this device to read the local i2C bus devices for data to publish.
  client.setCallback( callback );
  client.subscribe( inTopic );
  publishHealth( );
  Serial.println("MQTT setup complete.");

#if defined _DHT11_INCLUDED_
  Serial.println("Probe DHT11: ");
  //GPIO 3 (normally RX on -01) swap the pin to a GPIO. Use it for the DHT
  pinMode(3, OUTPUT);

  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead:
  dht.setup(3, DHTesp::DHT11); // Connect DHT sensor to GPIO 3
  sensorPresence |= _DHT11_INCLUDED_;
  dht11Present = true;
#endif

#if defined _HTU21D_INCLUDED_
  Serial.print("Probe HTU21D: ");
  htu.begin( );
  float val = htu.readHumidity() ;
  if ( val != 999  && val != 998 )
  {
    htuPresent = true;
  }

  htuPresent = false; //force for now
  if ( !htuPresent )
  {
    Serial.println("HTU21D Sensor not found");
  }
  else
  {
    htuHumidity = val;
    htuTemperature = htu.readTemperature();
    sensorPresence |= _HTU21D_INCLUDED_;
    Serial.println("HTU21D Sensor found");
  }
#endif

#if defined _BMP280_INCLUDED_
  Serial.print("Probe BMP280: ");
  bmp280Present = bmp280.initialize();
  if ( !bmp280Present )
  {
    Serial.println("BMP280 Sensor missing");
  }
  else
  {
    Serial.println("BMP280 Sensor found");
    sensorPresence |= _BMP280_INCLUDED_;
    // onetime-measure:
    bmp280.setEnabled();
    bmp280.triggerMeasurement(); //kick off measurement acquire for loop
  }
#endif

#if defined _QMC5883L_INCLUDED_
  Serial.println("Setup qCompass");
  qCompassPresent = qCompass.begin();
  qCompassPresent = true; //force for now until fix detection
  if ( !qCompassPresent ) // If there is an error, print it out.
  {
    Serial.println( "qCompass not found");
  }
  else
  {
    //settings specific to this device determmined by separate rotational calibration
    qCompass.setOffset( 1404, 5720, -5000 );
    qBearing = qCompass.getBearing();
    sensorPresence |= _QMC5883L_INCLUDED_;
    Serial.println( "qCompass detected and initialised");
  }
#endif

#if defined _CMPS03_INCLUDED_
  Serial.println("Setup cCompass");
  cCompassPresent = cCompass.begin();
  cCompassPresent = true; //force for now until fix detection
  if ( !cCompassPresent ) // If there is an error, print it out.
    Serial.println( "cCompass not found");
  else
  {
    Serial.println( "cCompass detected and initialised");
    sensorPresence |= _CMPS03_INCLUDED_;
  }
#endif

  //Setup webserver handler functions
  server.on("/", handleRoot);
  server.on("/bearing", HTTP_GET, handleBearingGet );
  server.on("/offsets", HTTP_PUT, handleOffsetsPut );
  //server.on("/bearing/reset", HTTP_GET, handleBearingResetGet );
  server.onNotFound(handleNotFound);

  updater.setup( &server);
  server.begin();
  Serial.println( "Setup webserver handlers");

  //Setup timers
  //setup interrupt-based 'soft' alarm handler for periodic acquisition of new bearing
  ets_timer_setfn( &timer, onTimer, NULL );
  ets_timer_setfn( &timeoutTimer, onTimeoutTimer, NULL ); 
  
  //fire timer every 250 msec
  //Set the timer function first
  ets_timer_arm_new( &timer, 500, 1/*repeat*/, 1);
  //ets_timer_arm_new( &timeoutTimer, 2500, 0/*one-shot*/, 1);

  originalRam = device.getFreeHeap();
  lastRam = originalRam;
  DEBUGSL1( "Setup complete" );
}

//Timer handler for 'soft'
void onTimer( void * pArg )
{
  newDataFlag = true;
}

//Used to complete timeout actions.
void onTimeoutTimer( void* pArg )
{
  //Read command list and apply.
  timeoutFlag = true;
}

//Main processing loop
void loop()
{
  String output;
  static int loopCount = 0;
  int i=0;

  if( WiFi.status() != WL_CONNECTED)
  {
      device.restart();
  }
  
  //Time profiling
  startTime = system_get_time();

  if ( newDataFlag == true )
  {
#if defined _DHT11_INCLUDED_
    //Only sample humidity < once per sec === 4* 250 ms default rate;
    //Spread the time impact rather than try to do it all at once.
    //Check this is valid - under the hood it may do both in a onner.
    if ( dht11Present )
    {
      loopCount++;
      if ( loopCount == 4 )
      {
        humidity  = dht.getHumidity();
      }
      if ( loopCount >= 8 )
      {
        aTemperature = dht.getTemperature();
        dewpoint = dht.computeDewPoint( aTemperature, humidity, false );
        loopCount = 0;
      }
    }
#endif

#if defined _HTU21D_INCLUDED_
    if ( htuPresent )
    {
      //These calls incorporate a 10ms delay internally each to wait for valid data
      //potential to break them up into seperate calls completed by one-off timers
      htuHumidity = htu.readHumidity();
      htuTemperature = htu.readTemperature();
    }
#endif

#if defined _BMP280_INCLUDED_
    //Get the pressure and turn into dew point info
    if ( bmp280Present )
    {
      bmp280.awaitMeasurement();
      bmp280.getTemperature( bTemperature  );
      bmp280.getPressure( pressure );
      bmp280.triggerMeasurement();
    }
    else
    {
      pressure = 103265.0;
    }
#endif

#if defined _QMC5883L_INCLUDED_
    if ( qCompassPresent )
    {
      // Retrieve the raw values from the compass (not scaled).
      raw = qCompass.readRaw();
      qBearing = qCompass.getBearing();

      //Serial.printf( "raw: %f %f, qBearing: %f \n", raw.XAxis, raw.YAxis, qBearing );
      cTemperature = qCompass.getTemperature();
    }
#endif

#if defined _CMPS03_INCLUDED_
    if ( cCompassPresent)
    {
      cBearing = cCompass.getBearing();
      Serial.printf( " cBearing -  %f \n", cBearing );
    }
#endif

    //end loop
    newDataFlag = false;
    
    //Check heap for memory bugs 
    uint32_t ram = ESP.getFreeHeap();
    if( lastRam != ( ram - originalRam ) )
    {
      lastRam = ram - originalRam;
      Serial.printf("RAM: %d  change %d\n", ram, lastRam );
    }
  }
  if ( client.connected() )
  {
    if (callbackFlag == true )
    {
      //publish results
      //Reset flags after last loop - also reset publish flag
      if ( client.connected() )
      {
         for ( i=0; i<8; i++ )
         {
            if( sensorPresence & (1 << i) )
            {
              (*pubFuncs[i])();
            }
            delay(1);
         }
        publishHealth();
        callbackFlag = false;
        DEBUGSL1("Callback response:");     
      }
    }
    client.loop();
  }
  else 
  {
    reconnectNB();
  }
  
  //Handle web requests
  server.handleClient();
  
  //Time profiling
  //currentTime = system_get_time();
  //DEBUGS1( currentTime - startTime );DEBUGSL1( " ms" );
}

/*
   Web server handler functions
*/
void handleNotFound()
{
  String message = "URL not understood\n";
  message.concat( "Simple status read: http://");
  message.concat( myHostname );
  message.concat ( "\n");
  message.concat( "Bearing read: http://");
  message.concat( myHostname );
  message.concat ( "/bearing \n");
  server.send(404, "text/plain", message);
}

void handleOffsetsPut()
{
  String timeString = "", message = "";
  DynamicJsonBuffer jsonBuffer(400);
  JsonObject& root = jsonBuffer.createObject();
  String sOffsets[3];
  int iOffsets[3] = {0,0,0};
  int status = 0;
  int i = 0;

  root["time"] = getTimeAsString( timeString );

  if (server.hasArg("Xoffset") && server.hasArg("Yoffset") && server.hasArg("Zoffset") )
  {
    iOffset[0] = server.arg("Xoffset").toInt();
    iOffset[1] = server.arg("Yoffset").toInt();
    iOffset[2] = server.arg("Zoffset").toInt();
    for (status = 0, i = 0; i < 3; i++ )
    {
      //Check for in range - mark if not
      if ( abs(iOffset) > 10000 )
        status |= 1;
    }
    if( !status )
    {
      //update the magnetometer offsets
      qCompass.setOffset( iOffset[0], iOffset[1], iOffset[2] );
      root["message"] = "Offsets updated" ;
      root.printTo( message );
      status = 200;
    }
    else
    {
      root["message"] = "Offsets out of range" ;
      status = 404;
    }      
  }
  else
  {
    root["message"] = "Offsets not found" ;
    status = 404;
  }
  root.printTo( message);
  server.send( status, "application/json", message);        
}

void handleBearingGet()
{
  String timeString = "", message = "";
  DynamicJsonBuffer jsonBuffer(400);
  JsonObject& root = jsonBuffer.createObject();

  root["time"]     = getTimeAsString2( timeString );

#if defined _QMC5883L_INCLUDED_
#if defined _CMPS03_INCLUDED_
  if ( cCompassPresent && qCompassPresent )
  {
    JsonArray& data = root.createNestedArray("bearing");
    data.add( qBearing );
    data.add( cBearing );
  }
  else if ( qCompassPresent && !cCompassPresent)
    root["bearing"]  = qBearing;
  else if ( cCompassPresent && !qCompassPresent )
    root["bearing"]  = cBearing;
  else
    root["bearing"]  = 0.0F;
#endif
#endif

#if defined _QMC5883L_INCLUDED_
#ifndef _CMPS03_INCLUDED_
  if ( qCompassPresent )
    root["bearing"] = qBearing;
#endif
#endif

#if defined _CMPS03_INCLUDED_
#ifndef _QMC5883L_INCLUDED_
  if ( cCompassPresent )
    root["bearing"]  = cBearing;
#endif
#endif

  root.printTo(message);
  server.send(200, "application/json", message);
}

//Return sensor status
void handleRoot()
{
  String timeString;
  String message;
  DynamicJsonBuffer jsonBuffer(256);
  JsonObject& root = jsonBuffer.createObject();

  root["time"] = getTimeAsString2( timeString );
#if defined _DHT11_INCLUDED_
  if ( dht11Present )
  {
    root["humidity"] = humidity;
    root["dewpoint"] = dewpoint;
  }
#endif

#if defined _HTU21D_INCLUDED_
  if ( htuPresent )
  {
    root["humidity"] = htuHumidity;
    root["htuTemperature"] = htuTemperature;
  }
#endif

#if defined _BMP280_INCLUDED_
  if ( bmp280Present )
  {
    root["pressure"] = pressure;
  }
#endif

#if defined _QMC5883L_INCLUDED_ && defined _CMPS03_INCLUDED_
  if ( qCompassPresent && cCompassPresent )
  {
    JsonArray& bearings = root.createNestedArray("bearing");
    bearings.add( qBearing );
    bearings.add( cBearing );
  }
  else if ( qCompassPresent && !cCompassPresent )
    root["bearing"] = qBearing;
  root["bearing"] = cBearing;
#elif defined _QMC5883L_INCLUDED_
  if ( qCompassPresent )
  {
    root["bearing"] = qBearing;

    JsonArray& mags = root.createNestedArray("BFields");
    mags.add( raw.XAxis );
    mags.add( raw.YAxis );
    mags.add( raw.ZAxis );
  }
#elif defined _CMPS03_INCLUDED_
  if ( cCompassPresent )
    root["bearing"] = cBearing;
#endif

  JsonArray& temps = root.createNestedArray("temperatures");
#if defined _DHT11_INCLUDED_
  if ( dht11Present )
    temps.add( aTemperature );
#endif
#if defined _HTU21D_INCLUDED_
  if ( htuPresent )
    temps.add( htuTemperature );
#endif
#if defined _BMP280_INCLUDED_
  if ( bmp280Present )
    temps.add( bTemperature );
#endif
#if defined _QMC5883L_INCLUDED_
  if ( qCompassPresent )
    temps.add( cTemperature );
#endif

  //root.printTo( Serial );
  root.printTo(message);
  server.send(200, "application/json", message);
}

/* MQTT callback for subscription and topic.
   Only respond to valid states ""
   Publish under ~/skybadger/sensors/<sensor type>/<host>
   Note that messages have an maximum length limit of 18 bytes - set in the MQTT header file.
*/
void callback(char* topic, byte* payload, unsigned int length)
{
  //set callback flag
  callbackFlag = true; 
}

bool publishHealth(void)
{
  bool pubState = false;
  DynamicJsonBuffer jsonBuffer(250);
  JsonObject& root = jsonBuffer.createObject();
  String outString;
  String outTopic;
  String timestamp;

  DEBUGSL1( "Entered publishHealth" );
  getTimeAsString2( timestamp );
  //Put a notice out regarding device health
  root["hostname"] = myHostname;
  root["timestamp"] = timestamp;
  root["message"] = String( "Measurement published" );
  root.printTo( outString );
  outTopic = outHealthTopic;
  outTopic.concat( myHostname );
  pubState = client.publish( outTopic.c_str(), outString.c_str(), true );
  Serial.printf( "Health topic published to %s , content: %s \n", outTopic.c_str(), outString.c_str() );
  
  DEBUGSL1( "leaving publishHealth" );
  return pubState; 
}

bool publishBMP(void)
{
  bool pubState = false;
  //publish to our device topic(s)
  DynamicJsonBuffer jsonBuffer(250);
  JsonObject& root = jsonBuffer.createObject();
  String outString;
  String outTopic;
  String timestamp;
  timestamp = getTimeAsString2( timestamp );

#if defined _BMP280_INCLUDED_
//  DEBUGSL1( "Entered publishBMP" );
  root["sensor"] = "bmp280";
  root["timestamp"] = timestamp;
  root["pressure"] = pressure;

  outTopic = outSenseTopic;
  outTopic.concat("pressure/");
  outTopic.concat(myHostname);

  root.printTo( outString );
  pubState = client.publish( outTopic.c_str(), outString.c_str() );

  /*
      if( !pubState)
        Serial.print( "Failed to publish BMP280 pressure sensor measurement ");
      else
        Serial.println( "BMP280 pressure sensor measurement published");
      Serial.print( outTopic );Serial.println( output );
  */
  outString = "";
  root.remove("pressure");
  root["temperature"] = bTemperature;
  root.printTo( outString );

  outTopic = "";
  outTopic = outSenseTopic;
  outTopic.concat("temperature/");
  outTopic.concat(myHostname);

  pubState = client.publish( outTopic.c_str(), outString.c_str() );
  /*
      if( !pubState)
      {
        Serial.print( "Failed to publish BMP280 temperature sensor measurement ");
      }
      else
        Serial.println( "BMP280 temperature sensor measurement published");
      Serial.print( outTopic );Serial.println( output );
  */
  //DEBUGSL1( "Leaving publishBMP" );
#endif
  return pubState;
}

bool publishHTU(void)
{
  bool pubState = false;
  DynamicJsonBuffer jsonBuffer(250);
  JsonObject& root = jsonBuffer.createObject();
  String outString;
  String outTopic;
  String timestamp;
  timestamp = getTimeAsString2( timestamp );

#if defined _HTU21D_INCLUDED_
  //DEBUGSL1( "Entering publishHTU" );
  root["sensor"] = "HTU21D";
  root["timestamp"] = timestamp;
  root["Humidity"] = htuHumidity;

  outTopic = outSenseTopic;
  outTopic.concat("humidity/");
  outTopic.concat(myHostname);

  root.printTo( outString );
  pubState = client.publish( outTopic.c_str(), outString.c_str() );

  /*
      if( !pubState)
        Serial.print( "Failed to publish HTU21D humidity sensor measurement ");
      else
        Serial.println( "Published HTU21D humidity sensor measurement ");
      Serial.print( outTopic );Serial.println( output );
  */
  //DEBUGSL1( "Leaving publishHTU" );
#endif
  return pubState;
}

bool publishDHT( void )
{
  bool pubState = false;
  DynamicJsonBuffer jsonBuffer(250);
  JsonObject& root = jsonBuffer.createObject();
  String outString;
  String outTopic;
  String timestamp;
  timestamp = getTimeAsString2( timestamp );

#if defined _DHT11_INCLUDED_
  //DEBUGSL1( "Entering publishDHT" );
  root["sensor"] = "DHT11";
  root["timestamp"] = timestamp;
  root["humidity"] = humidity;
  root.printTo( outString );

  outTopic = outSenseTopic;
  outTopic.concat("humidity/");
  outTopic.concat(myHostname);
  pubState = client.publish( outTopic.c_str() , outString.c_str() );

  /*
       if( !pubState)
        Serial.print( "failed to publish DHT11 humidity sensor measurement ");
      else
        Serial.println( "DHT11 humidity sensor measurement published");
      Serial.print( outTopic );Serial.println( output );
  */

  root.remove("humidity");
  outString = ""; //reset
  root["sensor"] = "DHT11";
  root["temperature"] = aTemperature;
  root.printTo( outString );

  outTopic = outSenseTopic;
  outTopic.concat("temperature/");
  outTopic.concat(myHostname);
  pubState = client.publish( outTopic.c_str(), outString.c_str() );
  /*
      if( !pubState)
        Serial.print( "failed to publish DHT11 temperature sensor measurement ");
      else
        Serial.println( "DHT11 temperature sensor measurement published");
      Serial.print( outTopic );Serial.println( output );
  */
  root.remove("temperature");
  outString = ""; //reset
  root["dewpoint"] = dewpoint;
  root.printTo( outString );

  outTopic = outSenseTopic;
  outTopic.concat("dewpoint/");
  outTopic.concat(myHostname);
  pubState = client.publish( outTopic.c_str(), outString.c_str() );

  /*
      if( !pubState)
        Serial.print( "failed to publish DHT11 dewpoint sensor measurement ");
      else
        Serial.println( "DHT11 dewpoint sensor measurement published");
      Serial.print( outTopic );Serial.println( output );
  */
  //DEBUGSL1( "Leaving publishDHT" );  
#endif
  return pubState;
}

bool publishQMC(void)
{
  bool pubState = false;
  DynamicJsonBuffer jsonBuffer(250);
  JsonObject& root = jsonBuffer.createObject();
  String outString;
  String outTopic;
  String timestamp;
  timestamp = getTimeAsString2( timestamp );

#if defined _QMC5883L_INCLUDED_
  //DEBUGSL1( "Entering publishQMC" );
  root["sensor"] = "QMC5883L";
  root["timestamp"] = timestamp;

  JsonArray& mags = root.createNestedArray("BFields");
  mags.add( raw.XAxis );
  mags.add( raw.YAxis );
  mags.add( raw.ZAxis );
  root.printTo( outString );

  outTopic = outSenseTopic;
  outTopic.concat("BField/");
  outTopic.concat(myHostname);
  pubState = client.publish( outTopic.c_str(), outString.c_str() );

  /*
      if( !pubState)
      {
        Serial.print( "Failed to publish QMC5883L mag field sensor measurement ");
      }
      else
        Serial.println( "QMC5883L mag field sensor  measurement published");
      Serial.print( outTopic );Serial.println( output );
  */
  root.remove("BFields");

  outString = "";
  root["bearing"] = qBearing;
  root.printTo( outString );
  outTopic = outSenseTopic;
  outTopic.concat("bearing/");
  outTopic.concat(myHostname);
  pubState = client.publish( outTopic.c_str(), outString.c_str() );

  /*
    if( !pubState)
    {
    Serial.print( "Failed to publish qCompass bearing");
    }
    else
    Serial.println( "qCompass bearing published");
    Serial.print( outTopic );Serial.println( output );
  */

  outString = ""; //reset
  root.remove("bearing");
  root["temperature"] = cTemperature;
  root.printTo( outString );

  outTopic = outSenseTopic;
  outTopic.concat("temperature/");
  outTopic.concat(myHostname);
  pubState = client.publish( outTopic.c_str(), outString.c_str() );
  /*
    if( !pubState)
    Serial.println( "qBearing temperature sensor measurement published");
    else
    Serial.println( "Failed to publish qBearing temperature sensor measurement ");
    Serial.print( outTopic );Serial.println( output );
  */
  //DEBUGSL1( "Leaving publishQMC" );
#endif
  return pubState;
}

bool publishCMP(void )
{
  bool pubState = false;
  DynamicJsonBuffer jsonBuffer(250);
  JsonObject& root = jsonBuffer.createObject();
  String outString;
  String outTopic;
  String timestamp;
  timestamp = getTimeAsString2( timestamp );

#if defined _CMPS03_INCLUDED_
  //DEBUGSL1( "Entering publishCMP" );
  root["sensor"] = "CMPS03";
  root["timestamp"] = timestamp;  
  root["bearing"] = cBearing;
  root.printTo( outString );

  outTopic = outSenseTopic;
  outTopic.concat("bearing/");
  outTopic.concat(myHostname);
  pubState = client.publish( outTopic.c_str(), outString.c_str() );

  /*
      if( !pubState)
        Serial.println( "CMPS03Bearing sensor measurement published");
      else
        Serial.println( "Failed to publish CMPS03Bearing sensor measurement ");
      Serial.print( outTopic );Serial.println( output );
  */
  DEBUGSL1( "Leaving publishCMP" );
#endif

  return pubState;
}


/*
  Returns the current state of the client. If a connection attempt fails, this can be used to get more information about the failure.
  int - the client state, which can take the following values (constants defined in PubSubClient.h):
  -4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
  -3 : MQTT_CONNECTION_LOST - the network connection was broken
  -2 : MQTT_CONNECT_FAILED - the network connection failed
  -1 : MQTT_DISCONNECTED - the client is disconnected cleanly
  0 : MQTT_CONNECTED - the client is connected
  1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
  2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
  3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
  4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
  5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect
*/
void reconnectNB() 
{
  //Non-blocking version
   if ( timerSet ) //timer is running but not timed out. 
   {
     if( timeoutFlag ) //Timeout - try again
     {   
         Serial.print("Repeating MQTT connection attempt...");
         if ( !client.connect(thisID, pubsubUserID, pubsubUserPwd ) )
         {  //Set a one-off timer to try next time around. 
            Serial.print("connect failed, rc=");
            Serial.println(client.state());
            timeoutFlag = false;
            timerSet = true;
            ets_timer_arm_new( &timeoutTimer, 5000, 0/*one-shot*/, 1);           
         }
         else
         { //Stop - all connected again
            timerSet = false;
         }
      }
   }
   else //timer not set 
   {
     Serial.print("Attempting MQTT connection...");
     if ( !client.connect(thisID, pubsubUserID, pubsubUserPwd ) )
     {  
        Serial.print("connect failed, rc=");
        Serial.println(client.state());

        //Set a one-off timer to try next time around. 
        timeoutFlag = false;
        timerSet = true;
        ets_timer_arm_new( &timeoutTimer, 5000, 0/*one-shot*/, 1);           
     }
     else
     {
        publishHealth();
        client.subscribe(inTopic);
        Serial.println("MQTT connection regained.");
     }
   }
return;
}

void reconnect( void )
{
  String output;
  DEBUGSL1( "Entering reconnect" );
  // Loop until we're reconnected
  while ( !client.connected() )
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(thisID, pubsubUserID, pubsubUserPwd ) )
    {
      //publish to our device topic(s)
      Serial.println("connected");
      // Resubscribe
      client.subscribe(inTopic);
    }
    else
    {
      Serial.printf("failed, rc= %i, try again in 5 seconds \r\n", client.state() );
//      // Wait 5 seconds before retrying
//      for (int i = 0; i < 5000; i++)
//      {
//        delay(1);
//        //delay(20);
//        //yield();
//      }
    }
  }
  Serial.println( "Actually connected");
  DEBUGSL1( "Leaving reconnect" );
}

/*
   Helper functions
*/
void syncTime()
{
  //The reason to do this is that we timestamp the data with the network time. 
  //However the network time is only recorded to the second since 1970 on the ESP8266
  //
  struct timeval tv;
  gettimeofday( &tv, nullptr );  
  
  struct timezone tzone;
  tzone.tz_minuteswest = 0;
  tzone.tz_dsttime = DST_MN;
  
  //Look for the boundary change of second
  time_t now, last;
  now = time(nullptr);
  struct tm* gnow = gmtime( &now );
  tv.tv_sec = (((gnow->tm_hour*60) + gnow->tm_min )*60) + gnow->tm_sec;
  last = now;
  while( ( now - last ) == 0 )
  {
      last = now;
      now = time( nullptr );
  }

  //As soon as gmtime seconds change - update the system clock usec counter.
  tv.tv_usec = 0;
  settimeofday( &tv, &tzone );
}
 
String& getTimeAsString2(String& output)
{
   //relies on the system clock being synchronised with the sntp clock at the ms level. 
   char buf[64];
   now = time( nullptr );
   sprintf( buf, "%lu%03i", now + (millis() % 1000 ) );
   output = String( buf);
   return output;
}

String& getTimeAsString(String& output)
{
  /* ISO Dates (Date-Time)
     ISO dates can be written with added hours, minutes, and seconds (YYYY-MM-DDTHH:MM:SSZ)
     Z indicates Zulu or GMT time,
     For other timezones use +/-HH:MM instead of Z
     Can replace 'T' with ' ' - works in javascript nicely.
  */
  struct timeval tv;
  gettimeofday( &tv, nullptr );
  //double milliTimestamp = ((double) (tv.tv_sec))*1000.0 + tv.tv_usec/1000.0;
  //Serial.println( tv.tv_sec );//works in seconds - get 10 digits
  //Serial.println( tv.tv_usec/1000 );//works in usecs - get 5 digits
  //Serial.println( milliTimestamp, 3 ); //Doesnt work when you put them together - overflows

  //get time, maintained by NTP, but only to the second.
  //Hence we'd like to use the system time in ms from 1970 - which is what the js date parser would like too, but esp cant provide
  //an integer long enough it seems.
  now = time(nullptr);
  struct tm* gnow = gmtime( &now );
  //output += String(gnow->tm_year + 1900) + "-" + \
  String(gnow->tm_mon ) + "-" + \
  String(gnow->tm_mday ) + " " + \
  String(gnow->tm_hour) + ":" + \
  String(gnow->tm_min) + ":" + \
  String( (float) gnow->tm_sec + tv.tv_usec / 1000000.0 ) + "Z";

  //This works but its heavy for an embedded device.
  char buf[128];
  sprintf( buf, "%4i-%02i-%02i %02i:%02i:%02.3fZ", gnow->tm_year + 1900, gnow->tm_mon + 1, gnow->tm_mday + 1, gnow->tm_hour, gnow->tm_min, (float) gnow->tm_sec + tv.tv_usec / 1000000.0 );

  //String and Serial classes can't handle 'long long' - missing SerialHelper overload
  output = String(buf);
  //Serial.print("timestamp value:");Serial.println( output );
  return output;
}
