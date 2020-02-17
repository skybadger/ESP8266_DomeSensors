#if !defined _DOMESENSEWEBHANDLERS_H_
#define _DOMESENSEWEBHANDLERS_H_
#include "ProjectDefs.h"
/*
   Web server handler functions
*/

//Web Handler function definitions
void handleRoot(void);
void handleNotFound();
void handleBearingGet();
void handleOffsetsPut();

/*
#include "ESP8266WebServer"
#include "ArduinoJSON.h"
#include "handlerfuncdefs.h"
extern ESP8266WebServer server;

extern QMC5883L qCompass;
extern CMPS03 cCompass;
extern char* myHostname;
extern int* iOffsets;

//Add external device-specific variables too - temperature , pressure etc. 

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

//Handler for loading new calibration offsets for QMC8=5883L compass. 
//Accepts in the range 0-+/- 10000
void handleOffsetsPut()
{
  String timeString = "", message = "";
  DynamicJsonBuffer jsonBuffer(400);
  JsonObject& root = jsonBuffer.createObject();
  int offsets[N_OFFSETS] = {0,0,0};
  int status = 0;
  int i = 0;

  root["time"] = getTimeAsString( timeString );

  if (server.hasArg("Xoffset") && server.hasArg("Yoffset") && server.hasArg("Zoffset") )
  {
    offsets[0] = server.arg("Xoffset").toInt();
    offsets[1] = server.arg("Yoffset").toInt();
    offsets[2] = server.arg("Zoffset").toInt();
    for (status = 0, i = 0; i < 3; i++ )
    {
      //Check for in range - mark if not
      if ( abs(offsets[i] ) > 10000 )
        status |= 1;
    }
    if( !status )
    {
      //All values in range - update the magnetometer offsets
      for (status = 0, i = 0; i < 3; i++ )
      {
         iOffsets[i] = offsets[i];
      }      
      qCompass.setOffset( iOffsets[0], iOffsets[1], iOffsets[2] );
      saveToEeprom();
      root["message"] = "Offsets updated" ;
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
#endif