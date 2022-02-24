#include "src/version.h"
#include "src/macros.h"
#include "src/XpressNetMaster.h"
#include <EEPROM.h> // needs to be removed in the future, preferebly
#include "eeprom.h"

#define RS485DIR 2

#define   F0_F4  0x00
#define   F5_F8  0x01
#define  F9_F12  0x02
#define F13_F20  0x03

#define CURVED   0x0000
#define STRAIGHT 0x8000

#define BASE_ADDRESS 100 
#define MAX_ADDRESS 1024

#ifndef debug
XpressNetMasterClass Xnet ;
#endif


uint8   knob ;
volatile uint8   recording ;
volatile uint8   gettingStreetIndex ;
volatile uint8   streetIndex ;

volatile uint16 eeAddress  = 0 ;

volatile unsigned long long oldState ; // 64 bits


volatile uint8   pointIndex = 100 ;      // something high


void settingPoints()
{
    if( pointIndex > nPointsPerStreet ) return ;

    REPEAT_MS( 500 )
    {
    nextPoint:
        uint16 pointNumber = points[ pointIndex ] & 0x03FF ;
        uint8  state       = points[ pointIndex ] >> 15 ;
        pointIndex ++ ;

        if( pointIndex > nPointsPerStreet ) return ;
        //if( pointNumber >= MAX_ADDRESS )   goto nextPoint ;

        #ifndef debug
        Xnet.SetTrntPos( pointNumber - 1, state, 1 ) ;                      // set new state
        delay(20) ;
        Xnet.SetTrntPos( pointNumber - 1, state, 0 ) ;
        #endif

    } END_REPEAT
}
void setStreet( uint8 streetNumber )
{
    pinMode(13, INPUT_PULLUP );
    uint16 eeAddress = streetNumber * nPointsPerStreet * 2 ;                      // calculate address
   
    EEPROM.get( eeAddress, points ) ;                                             // fetch array from EEPROM
    pointIndex = 0 ;                                                                // reset this index for setting a street
}
void notifyXNetTrnt( uint16_t Address, uint8_t data )                               // setting point 101, gives us 100 back
{
    if( Address == (BASE_ADDRESS - 1) )
    {
        bool state = data & 1 ;
        if( state == 1 ) { recording =  true ; gettingStreetIndex = true ; }
        else             { recording = false ; }
        return ;
    }

    if( recording )
    {
        if( gettingStreetIndex == true )
        {
            if( Address < BASE_ADDRESS || Address >= (BASE_ADDRESS + nStreets ) ) return ; // check for valid address
           
            streetIndex = Address - BASE_ADDRESS ;
            gettingStreetIndex = false ;
            clear( streetIndex ) ;
            /* TODO
            * whipe part for EEPROM for new point street
            */
        }
        else
        {
            /* TODO
            * check if address is not there yet
            * add address and state to EEPROM, 
            *  
            */
        }
    }
    else                                                                        // if not recording, lay the streets.
    {
        pinMode(13, INPUT_PULLUP );
        if( Address < BASE_ADDRESS || Address >= (BASE_ADDRESS + 20) ) return ;

        setStreet( Address - BASE_ADDRESS ) ;
    }
}


void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )                   
{
    static uint8 state = 0 , prevKnob = 0xFF ;
    int8_t speed ;

    speed = Speed & 0x7F ;

    if( speed > 0 ) speed -- ;
    if( Speed & 0x80 ) speed = -speed ;
   
    if(         speed <  -100                ) knob = 4 ;
    else if(    speed >= -100 && speed < -20 ) knob = 3 ;
    else if(    speed >   -20 && speed <  20 ) knob = 2 ;
    else if(    speed <=  100 && speed >  20 ) knob = 1 ;
    else if(    speed >   100                ) knob = 0 ;
}


void setFunc( uint8 val )
{
    EEPROM.write( eeAddress++, val ) ;
}


void functionPressed ( uint16 Address, uint8 func, uint8 bank ) // bank is verivied, address is verivied, Address is verivied
{
    if( Address != 1) return ;

    volatile static uint8 prevState[4];
    volatile uint8  fKey ;
    volatile uint16 maskMax ;

    switch( bank )
    {
    case   F0_F4 : fKey =  0 ; maskMax =  0x20 ; break ;
    case   F5_F8 : fKey =  4 ; maskMax =  0x10 ; break ;
    case  F9_F12 : fKey =  8 ; maskMax =  0x10 ; break ;
    case F13_F20 : fKey = 12 ; maskMax = 0x100 ; break ;
    }

    for( uint8 bitMask = 0x01 ; bitMask < maskMax ; bitMask <<= 1 )
    {
        fKey ++ ;
        if( (func & bitMask) != (prevState[bank] & bitMask ) )
        {
            if( func & bitMask ) { prevState[bank] |=  bitMask ; }
            else                 { prevState[bank] &= ~bitMask ; }

            if( fKey == 5 && bank == F0_F4 ) fKey = 0 ;                         // F0 is on the 5th bit

            oldState ^= (1 << fKey );                                           // toggle previous state
            bool state = oldState >> fKey ;                                     // get state 

            uint8 pointNumber = fKey += ( knob * 10 ) ;                         // 5 groups

            Xnet.SetTrntPos( pointNumber - 1, state, 1 ) ;                      // set new state
            delay(20) ;
            Xnet.SetTrntPos( pointNumber - 1, state, 0 ) ;
            
            //setFunc( fKey ) ;                                                 // VERIVIED
            //setFunc( state ) ;                                                // VERIVIED
            return ;
        }
    }
}

void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { functionPressed( Address, Func1,   F0_F4 ) ; } //            F0    F4  F3  F2  F1
void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { functionPressed( Address, Func2,   F5_F8 ) ; } //                  F8  F7  F6  F5
void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) { functionPressed( Address, Func3,  F9_F12 ) ; } //                 F12 F11 F10  F9
void notifyXNetLocoFunc4( uint16_t Address, uint8_t Func4 ) { functionPressed( Address, Func4, F13_F20 ) ; } // F20 F19 F18 F17 F16 F15 F14 F13



void setup()
{
    uint16_t eeAddress = 0 ;
    // EEPROM.put( eeAddress++, CURVED   | 1   ) ; eeAddress++ ; this seems to work well
    // EEPROM.put( eeAddress++, CURVED   | 2   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 3   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 4   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 5   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 6   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 7   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 8   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 9   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 10  ) ; eeAddress++ ;

    // EEPROM.put( eeAddress++, STRAIGHT | 1   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 2   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 3   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 4   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, STRAIGHT | 5   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 6   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 7   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 8   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 9   ) ; eeAddress++ ;
    // EEPROM.put( eeAddress++, CURVED   | 10  ) ; eeAddress++ ;

    // points[ 0 ] = CURVED   | 1 ;
    // points[ 1 ] = CURVED   | 2 ;
    // points[ 2 ] = CURVED   | 3 ;
    // points[ 3 ] = CURVED   | 4 ;
    // points[ 4 ] = CURVED   | 5 ;
    // points[ 5 ] = STRAIGHT | 6 ;
    // points[ 6 ] = STRAIGHT | 7 ;
    // points[ 7 ] = STRAIGHT | 8 ;
    // points[ 8 ] = STRAIGHT | 9 ;
    // points[ 9 ] = STRAIGHT | 10 ;

    // points[ 10 ] = 0xFFFF ;

    // pointIndex = 100 ;

    #ifndef debug
    Xnet.setup( Loco28, RS485DIR ) ;
    #else
    Serial.begin( 115200 ) ;
    //uint16 eeAddress = 0 ;
    for( uint16 i = 0 ; i < 100 ; i ++ )
    {
      
        uint8 a = EEPROM.read( eeAddress ++ ) ;
        uint8 b = EEPROM.read( eeAddress ++ ) ;
        // uint8 c = EEPROM.read( eeAddress ++ ) ;
        // uint8 d = EEPROM.read( eeAddress ++ ) ;
        Serial.print( a ) ; Serial.print("   "); Serial.println( b ) ;// Serial.print("   "); Serial.print( c ) ;  Serial.print("   "); Serial.println( d ) ;
        
    }
    while(1);
    #endif
}

void loop()
{

    #ifndef debug
    Xnet.update() ;
    settingPoints() ;    
    #endif
}