#include "src/version.h"
#include "src/io.h"
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

#ifndef debug
XpressNetMasterClass Xnet ;
#endif


uint8   knob ;

uint16 eeAddress1  = 0 ;

volatile unsigned long long oldState ; // 64 bits


void setPoint( uint16 pointAddress, uint8 state )
{
    #ifndef debug
    Xnet.SetTrntPos( pointAddress - 1, state, 1 ) ;                  // BUG needs to be wrapper function, no acces to Xnet object here
    delay(20) ;
    Xnet.SetTrntPos( pointAddress - 1, state, 0 ) ;
    #endif
}

void setFunc( uint8 val )
{
   // static uint16_t eeAddress = 0 ; 
   // pinMode(A7, INPUT) ;  

   // EEPROM.write( eeAddress++, val ) ;
}
// void notifyXNetTrnt(uint16_t Address, uint8_t data)
// {
//   //  static uint8 counter = 0 ;
//   //  pinMode(A7, INPUT) ;                                                        // desperate method to prevent linker from optimizing this function away.
//  //   Address ++;
    
//     if (bitRead(data,3) == 0x01)
//     { 
//         if( data & 0x01 ) digitalWrite( led, HIGH ) ;
//         else              digitalWrite( led,  LOW ) ;
//     }// passPoint( Address | (data<<15) ) ;


//         //setFunc( counter ++ ) ;
//         //setFunc( Address & 0xFF ) ;
//         //setFunc( data ) ;
//     // EEPROM.write( eeAddress1++, eeAddress1 ) ;
//     // EEPROM.write( eeAddress1++, Address ) ;
//     // EEPROM.write( eeAddress1++, data ) ;

// }
/*
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
*/


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

            setPoint( pointNumber, state ) ;
            
            //setFunc( fKey ) ;                                                 // VERIVIED
            //setFunc( state ) ;                                                // VERIVIED
            return ;
        }
    }
}

void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { functionPressed( Address, Func1,   F0_F4 ) ; } //              F0  F4  F3  F2  F1
void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { functionPressed( Address, Func2,   F5_F8 ) ; } //                  F8  F7  F6  F5
void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) { functionPressed( Address, Func3,  F9_F12 ) ; } //                 F12 F11 F10  F9
void notifyXNetLocoFunc4( uint16_t Address, uint8_t Func4 ) { functionPressed( Address, Func4, F13_F20 ) ; } // F20 F19 F18 F17 F16 F15 F14 F13


// void notifyXNetPower(uint8_t State)
// {
//     if( State == csNormal ) digitalWrite(led, HIGH);
//     else                    digitalWrite(led,  LOW);
// }
void setup()
{

    initIO() ; 

    for (int i = 0; i < 10; i++)
    {
            digitalWrite( led, HIGH ) ;
    delay(100);
    digitalWrite( led,LOW ) ;
    delay(100);
    }
    


    #ifndef debug
    Xnet.setup( Loco28, RS485DIR ) ;
    //beginEeprom() ;
    #else
    Serial.begin( 115200 ) ;
    for( uint16 i = 0 ; i < 100 ; i ++ )
    {
      
        uint8 a = EEPROM.read( eeAddress1 ++ ) ;
        uint8 b = EEPROM.read( eeAddress1 ++ ) ;
        uint8 c = EEPROM.read( eeAddress1 ++ ) ;
        // uint8 d = EEPROM.read( eeAddress ++ ) ;
        Serial.print( a ) ; Serial.print("   "); Serial.print( b ) ; Serial.print("   "); Serial.println( c ) ;//  Serial.print("   "); Serial.println( d ) ;
        
    }
    while(1);
    #endif
}

void loop()
{

    #ifndef debug
    Xnet.update() ;
    //handlePoints() ;
    #endif
}