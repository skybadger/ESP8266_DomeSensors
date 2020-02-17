#if !defined _DOMESENSEEEPROM_H_
#define _DOMESENSEEEPROM_H_
//Function definitions
void saveToEeprom( void );
void setupFromEeprom( void );
void setDefaults( void );

/* Make a stand alone header file and code
#include "projectDefs.h"
#include "EEPROM.h"
#include "EEPROMAnything.h"
extern char* myHostname;
extern xonst char* defaultHostname;
extern int iOffsets;
extern QMC5883L qCompass;

*/

void setDefaults( void )
{
  int i=0;
  DEBUGSL1( "setDefaults: entered");
  
  myHostname = (char*) calloc( sizeof(char), MAX_NAME_LENGTH );
  memcpy( myHostname, defaultHostname, MAX_NAME_LENGTH * sizeof(char) );
  DEBUGS1( "hostname:  ");DEBUGSL1( myHostname );
  
  thisID = (char*) calloc( sizeof(char), MAX_NAME_LENGTH );
  memcpy( thisID, defaultHostname, MAX_NAME_LENGTH * sizeof(char) );
  DEBUGS1( "MQTT ID:  ");DEBUGSL1( thisID );
    
  //Offsets
  iOffsets = (int*)  calloc( sizeof(int),   (unsigned) N_OFFSETS );
  for ( i = 0; i< N_OFFSETS ; i++ )
  {
    iOffsets[i] = 0;
    DEBUGS1( "setDefaults: calibration Offsets ");
    DEBUGSL1( iOffsets[i] );
  }
  
 DEBUGSL1( "setDefaults: exiting" );
}

void saveToEeprom( void )
{
  int i = 0;
  int eepromAddr = 1;

  DEBUGSL1( "savetoEeprom: Entered ");
  
  //hostname
  EEPROMWriteString( eepromAddr = 1, myHostname, MAX_NAME_LENGTH );
  eepromAddr += MAX_NAME_LENGTH;  
  DEBUGS1( "Written hostname: ");DEBUGSL1( myHostname );

  //qCompass calibration constant offsets
  for ( i=0; i < N_OFFSETS; i++ )
  {
    EEPROMWriteAnything( eepromAddr, iOffsets[i] ); 
    eepromAddr += sizeof( iOffsets[i]);
    DEBUGS1( "Written calibration Offsets[]: ");DEBUGSL1( iOffsets[i] );
  }
  
  EEPROMWriteAnything( eepromAddr=0, byte('#') );
  DEBUGSL1( "saveToEeprom: exiting ");

  EEPROM.commit();

#ifdef _EEPROMTEST_
  //Test readback of contents
  char ch;
  String input;
  for (i=0; i< 512 ; i++ )
  {
    ch = (char) EEPROM.read( i );
    if ( ch == '\0' )
      ch = '_';
    input.concat( ch );
  }

  Serial.printf( "EEPROM contents: %s \n", input.c_str() );
#endif
}

void setupFromEeprom()
{
  int eepromAddr = 0;
  byte bTemp = 0;
  int i=0;
    
  DEBUGSL1( "setUpFromEeprom: Entering ");
  
  //Setup internal variables - read from EEPROM.
  bTemp = EEPROM.read( eepromAddr );
  DEBUGS1( "Read init byte: ");DEBUGSL1( (char) bTemp );
  if ( (byte) bTemp != '#' ) //initialise
  {
    setDefaults();
    saveToEeprom();
    DEBUGSL1( "Failed to find init byte - wrote defaults ");
    return;
  }    
    
  //hostname - directly into variable array 
  if( myHostname != NULL ) free ( myHostname);
  myHostname = (char*) calloc( sizeof(char), MAX_NAME_LENGTH );
  i = EEPROMReadString( eepromAddr=1, myHostname, MAX_NAME_LENGTH );
  eepromAddr += MAX_NAME_LENGTH;
  DEBUGS1( "Read hostname: ");DEBUGSL1( myHostname );

  //MQTT ID  - copy hostname
  if( thisID != NULL ) free (thisID);
  thisID = (char*) calloc( sizeof(char), MAX_NAME_LENGTH );
  strcpy( thisID, myHostname );
  DEBUGS1( "Read MQTT ID: ");DEBUGSL1( thisID );
    
  //Compass calibration offsets
  if( iOffsets != NULL ) free( iOffsets);
  iOffsets = (int*) calloc( N_OFFSETS, sizeof(int) );  
  for ( i=0; i < N_OFFSETS; i++ )
  {
     EEPROMReadAnything( eepromAddr, iOffsets[i] ); 
     eepromAddr += sizeof( iOffsets[i]);
     DEBUGS1( "Read calibration Offsets[] ");DEBUGSL1( iOffsets[i] );
  }
  
 DEBUGS1( "setupFromEeprom: exiting having read " );DEBUGS1( eepromAddr );DEBUGSL1( " bytes." );
}
#endif
