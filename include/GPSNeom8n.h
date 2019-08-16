#include "TinyGPS++.h"
#include <HardwareSerial.h>
#define DEBUG_PORT Serial


HardwareSerial ss(1);

/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/


// The TinyGPS++ object
TinyGPSPlus gps;




static void repeat( char c, int8_t len )
{
  for (int8_t i=0; i<len; i++)
    DEBUG_PORT.write( c );
}

static void printInvalid( int8_t len )
{
  DEBUG_PORT.write( ' ' );
  repeat( '*', abs(len)-1 );
}

static void print( float val, bool valid, int8_t len, int8_t prec )
{
  if (!valid) {
    printInvalid( len );
  } else {
    char s[16];
    dtostrf( val, len, prec, s );
    DEBUG_PORT.print( s );
  }
}

static void print( int32_t val, bool valid, int8_t len )
{
  if (!valid) {
    printInvalid( len );
  } else {
    char s[16];
    ltoa( val, s, 10 );
    repeat( ' ', len - strlen(s) );
    DEBUG_PORT.print( s );
  }
}

static void print( const __FlashStringHelper *str, bool valid, int8_t len )
{
  if (!valid) {
    printInvalid( len );
  } else {
    int slen = strlen_P( (const char *) str );
    repeat( ' ', len-slen );
    DEBUG_PORT.print( str );
  }
}



//------------------------------------------------------------
//  This snippet is from NMEAaverage.  It keeps all the
//    compass direction strings in FLASH memory, saving RAM.

const char nCD  [] PROGMEM = "N";
const char nneCD[] PROGMEM = "NNE";
const char neCD [] PROGMEM = "NE";
const char eneCD[] PROGMEM = "ENE";
const char eCD  [] PROGMEM = "E";
const char eseCD[] PROGMEM = "ESE";
const char seCD [] PROGMEM = "SE";
const char sseCD[] PROGMEM = "SSE";
const char sCD  [] PROGMEM = "S";
const char sswCD[] PROGMEM = "SSW";
const char swCD [] PROGMEM = "SW";
const char wswCD[] PROGMEM = "WSW";
const char wCD  [] PROGMEM = "W";
const char wnwCD[] PROGMEM = "WNW";
const char nwCD [] PROGMEM = "NW";
const char nnwCD[] PROGMEM = "NNW";

const char * const dirStrings[] PROGMEM =
  { nCD, nneCD, neCD, eneCD, eCD, eseCD, seCD, sseCD, 
    sCD, sswCD, swCD, wswCD, wCD, wnwCD, nwCD, nnwCD };

const __FlashStringHelper *compassDir( uint16_t bearing ) // degrees CW from N
{
  const int16_t directions    = sizeof(dirStrings)/sizeof(dirStrings[0]);
  const int16_t degreesPerDir = 360 / directions;
        int8_t  dir           = (bearing + degreesPerDir/2) / degreesPerDir;

  while (dir < 0)
    dir += directions;
  while (dir >= directions)
    dir -= directions;

  return (const __FlashStringHelper *) pgm_read_ptr( &dirStrings[ dir ] );

} // compassDir


void setupGPSm8n()
{
  DEBUG_PORT.begin(9600);
  
  DEBUG_PORT.println
    (
      F( "Testing NeoGPS library\n\n"
         "Sats HDOP Latitude  Longitude  Date       Time     Alt    Speed  Heading    -- To London --    Chars Sentences Errors\n"
         "          (deg)     (deg)                          (m)                      Dist    Dir\n" )
    );

  repeat( '-', 133 );

  ss.begin(38400, SERIAL_8N1, 16, 17);
}
void printGPS()
{
  if (ss.available() > 0 ){
    //gps_gps gps = gps.read();

    

    print(gps.satellites.value(), gps.satellites.isValid(), 5);
    print(gps.location.lat(), gps.location.isValid(), 11, 6);
    print(gps.location.lng(), gps.location.isValid(), 12, 6);

    //print(             gps.hdop/1000.0      , gps.valid.hdop      , 6, 2          );
    DEBUG_PORT.println();

  }
}

//-----------------
//  Print utilities

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
