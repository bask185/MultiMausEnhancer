#include "src/version.h"
#include "src/macros.h"
#include "src/XpressNetMaster.h"
#include <EEPROM.h>




#define RS485DIR 2
#define  F1_F4  0x00
#define  F5_F8  0x40
#define F9_F10  0x80

#define BASE_ADDRESS 500 

#ifndef debug
XpressNetMasterClass Xnet ;
#endif

const int nPointsPerStreet = 20 ;

uint8   knob ;
volatile uint16 eeAddress  = 0 ;

unsigned long long prevState ; // 64 bits

uint16  points[ nPointsPerStreet ] ;
uint8   pointIndex ;


void settingPoints()
{
    if( pointIndex) > nPointsPerStreet
}
void setStreet( uint8 streetNumber )
{
    uint16 eeAddress = streetNumber * nPointsPerStreet * 2 ;                    // calculate address

    pointIndex = 0 ;                                                            // reset this index for setting a street
    EEPROM.get( eeAddress, points ) ;                                           // fetch array from EEPROM
}
void notifyXNetTrnt( uint16_t Address, uint8_t data )
{
    if( Address <= BASE_ADDRESS || Address > (BASE_ADDRESS + 20) ) return ;

    setStreet( Address - BASE_ADDRESS ) 
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

void setPoint( uint8_t Address, uint8_t functions )
{
    if( Address != 1) return ;   

    static uint16 prevFunctions[3][5] ;
    uint8 number ;
    uint8 index ;

    switch( functions & 0xC0 )
    {
    case  F1_F4 : index = 0 ; functions &= 0x0F ; number = 0 ;/* Serial.println( "F1_F4" ) ;*/ break ;
    case  F5_F8 : index = 1 ; functions &= 0x0F ; number = 4 ;/* Serial.println( "F5_F8" ) ;*/ break ;
    case F9_F10 : index = 2 ; functions &= 0x03 ; number = 8 ;/* Serial.println("F9_F10" ) ;*/ break ;
    }

    for( int bitMask = 0x01 ; bitMask > 0x10 ; bitMask <<= 1 )                        // check which of the 4 bits has changed
    {       
        number ++ ;  

        if( (functions & bitMask) != (prevFunctions[index][knob] & bitMask ) )              // check all 4 bits for F1 - F4, if atleast 1 bit has changed
        {
            if( functions & bitMask ) { prevFunctions[index][knob]  |= bitMask ;/* printNumberln("setting:  ", number); */ }
            else                      { prevFunctions[index][knob] &= ~bitMask ;/* printNumberln("clearing: ", number); */ }      

            uint8 pointNumber = number += ( knob * 10 ) ;                       // 5 groups

            bool state = (prevState >> pointNumber) & 1 ;                       // get last state
            state ^= 1 ;                                                        // toggle state

            #ifndef debug
            Xnet.SetTrntPos( pointNumber - 1, state, 1 ) ;                      // set new state
            delay(20) ;
            Xnet.SetTrntPos( pointNumber - 1, state, 0 ) ;

            if( state == 0 ) prevState &= ~( 1 << pointNumber ) ;               // store new state
            else             prevState |=  ( 1 << pointNumber ) ; 

            #else
            printNumber_( "point: ", pointNumber ) ; Serial.println( state ) ;
            #endif
            
            return ;
        }
    }
}

// void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { setPoint( Address, Func1 |  F1_F4 ) ; } // Gruppe1        000 F0    F4  F3  F2  F1
// void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { setPoint( Address, Func2 |  F5_F8 ) ; } // Gruppe2          0000    F8  F7  F6  F5
// void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) { setPoint( Address, Func3 | F9_F10 ) ; } // Gruppe3          0000   F12 F11 F10  F9
// void notifyXNetLocoFunc4( uint16_t Address, uint8_t Func4 )                                           // Gruppe4 F20 F19 F18 F17 F16 F15 F14 F13

void setFunc( uint8 val )
{
    EEPROM.write( eeAddress++, val ) ;
}

void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) // F0  F4  F3  F2  F1
{
    if( Address != 1) return ;

    static uint8 prevState ;
    uint8 number = 0 ;

    for( uint8 bitMask = 0x01 ; bitMask > 0x20 ; bitMask <<= 1 )
    {
        number ++ ;
        if( (Func1 & bitMask) != (prevState & bitMask ) )
        {
            if( Func1 & bitMask ) { prevState |=  bitMask ; }
            else                  { prevState &= ~bitMask ; }

            if( number == 5) number = 0 ;   // F0 is on the 5th bit

            setFunc( number ) ;
        }
    }
}

void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) // F8  F7  F6  F5
{
    if( Address != 1) return ;

    static uint8 prevState ;
    uint8 number = 4 ;

    for( uint8 bitMask = 0x01 ; bitMask > 0x10 ; bitMask <<= 1 )
    {
        number ++ ;
        if( (Func2 & bitMask) != (prevState & bitMask ) )
        {
            if( Func2 & bitMask ) { prevState |=  bitMask ; }
            else                  { prevState &= ~bitMask ; }

            setFunc( number ) ;
        }
    }
}

void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) // F12 F11 F10  F9
{
    if( Address != 1) return ;

    static uint8 prevState ;
    uint8 number = 8 ;

    for( uint8 bitMask = 0x01 ; bitMask > 0x10 ; bitMask <<= 1 )
    {
        number ++ ;
        if( (Func3 & bitMask) != (prevState & bitMask ) )
        {
            if( Func3 & bitMask ) { prevState |=  bitMask ; }
            else                  { prevState &= ~bitMask ; }

            setFunc( number ) ;
        }
    }
}

void notifyXNetLocoFunc4( uint16_t Address, uint8_t Func4 )  //F20 F19 F18 F17 F16 F15 F14 F13
{
    if( Address != 1) return ;

    static uint8 prevState ;
    uint8 number = 12 ;

    for( uint16 bitMask = 0x01 ; bitMask > 0x100 ; bitMask <<= 1 )
    {
        number ++ ;
        if( (Func4 & bitMask) != (prevState & bitMask ) )
        {
            if( Func4 & bitMask ) { prevState |=  bitMask ; }
            else                  { prevState &= ~bitMask ; }

            setFunc( number ) ;
        }
    }
}


void setup()
{
    #ifndef debug
    Xnet.setup( Loco28, RS485DIR ) ;
    #else
    Serial.begin( 115200 ) ;
    for( int i = 0 ; i < 50 ; i ++ )
    {
        int8 b = EEPROM.read(i) ;
        Serial.print( b ) ; Serial.print("   "); Serial.print( b, HEX ) ; Serial.print("   "); Serial.println( b, BIN ) ;
        
    }
    while(1);
    #endif
}

void loop()
{

    #ifndef debug
    Xnet.update() ;

    #else
    for( knob = 0 ; knob <= 4 ; knob ++ )
    {

        setPoint( 1, 0b0001 | F1_F4 ) ; 
        setPoint( 1, 0b0011 | F1_F4 ) ;
        setPoint( 1, 0b0111 | F1_F4 ) ;
        setPoint( 1, 0b1111 | F1_F4 ) ;

        setPoint( 1, 0b0001 | F5_F8 ) ;
        setPoint( 1, 0b0011 | F5_F8 ) ;
        setPoint( 1, 0b0111 | F5_F8 ) ;
        setPoint( 1, 0b1111 | F5_F8 ) ;

        setPoint( 1, 0b0001 | F9_F10 ) ;
        setPoint( 1, 0b0011 | F9_F10 ) ;

        Serial.println() ;

    }
    delay(10000000) ;

    
    #endif
}