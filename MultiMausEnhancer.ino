#include "src/version.h"
#include "src/macros.h"
#include "src/XpressNetMaster.h"
#include <EEPROM.h>

#define RS485DIR 2
#define  F1_F4  0x00
#define  F5_F8  0x40
#define F9_F10  0x80

#ifndef debug
XpressNetMasterClass Xnet ;
#endif

uint8   knob ;
volatile uint16 eeAddress  = 0 ;


void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )                   
{
    static uint8 state = 0 , prevKnob = 0xFF ;
    int8_t speed ;

    speed = Speed & 0x7F ;

    if( speed > 0 ) speed -- ;
    if( Speed & 0x80 ) speed = -speed ;
   
    if(         speed <  -100               ) knob = 4 ;
    else if(    speed >= -100 && speed <-20 ) knob = 3 ;
    else if(    speed >   -20 && speed < 20 ) knob = 2 ;
    else if(    speed <=  100 && speed > 20 ) knob = 1 ;
    else if(    speed >   100               ) knob = 0 ;

    // if( prevKnob != knob )
    // {   prevKnob  = knob ;

    //     EEPROM.write( eeAddress, knob ) ;
    //     eeAddress ++ ;
    //     // Xnet.SetTrntPos( 2, state, 1 ) ;
    //     // delay(20) ;
    //     // Xnet.SetTrntPos( 2, state, 0 ) ;
    //     // state ^= 1 ;
    // }
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

    for( int bitMask = 0x01 ; bitMask < 0x10 ; bitMask <<= 1 )                        // check which of the 4 bits has changed
    {       
        number ++ ;  

        if( (functions & bitMask) != (prevFunctions[index][knob] & bitMask ) )              // check all 4 bits for F1 - F4, if atleast 1 bit has changed
        {
            uint8_t state ;

            if( functions & bitMask ) { state = 0 ; prevFunctions[index][knob]  |= bitMask ;/* printNumberln("setting:  ", number); */ }                             // curved
            else                      { state = 1 ; prevFunctions[index][knob] &= ~bitMask ;/* printNumberln("clearing: ", number); */ }                            // straight        

            uint8 pointNumber = number += ( knob * 10 ) ;                                // 5 groups
            
            #ifndef debug
            Xnet.SetTrntPos( pointNumber - 1, state, 1 ) ;
            delay(20) ;
            Xnet.SetTrntPos( pointNumber - 1, state, 0 ) ;

            #else
            printNumber_( "point: ", pointNumber ) ; Serial.println( state ) ;
            #endif
            
            return ;
        }
    }
}

void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { setPoint( Address, Func1 |  F1_F4 ) ; } // Gruppe1 0 0 0 F0    F4  F3  F2  F1
void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { setPoint( Address, Func2 |  F5_F8 ) ; } // Gruppe2     0000    F8  F7  F6  F5
void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) { setPoint( Address, Func3 | F9_F10 ) ; } // Gruppe3     0000   F12 F11 F10  F9

void setup()
{
    #ifndef debug
    Xnet.setup( Loco28, RS485DIR ) ;
    #else
    Serial.begin( 115200 ) ;
    for( int i = 0 ; i < 1024 ; i ++ )
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