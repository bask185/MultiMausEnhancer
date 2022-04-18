#include "src/version.h"
#include "src/io.h"
#include "src/macros.h"
#include "src/io.h"
#include "XpressNetMaster.h"
#include <EEPROM.h> // needs to be removed in the future, preferebly
#include <SoftwareSerial.h>
#include "eeprom.h"
#include "event.h"

SoftwareSerial debugPort(3,4) ;

#define RS485DIR 2

#define   F0_F4  0x00
#define   F5_F8  0x01
#define  F9_F12  0x02
#define F13_F20  0x03

#define CURVED   0x0000
#define STRAIGHT 0x8000

#define POINT_DELAY( interval ) uint32_t prevTime = millis() ; \
                                while( millis() - prevTime <= interval ) { Xnet.update(); }

XpressNetMasterClass Xnet ;

bool    dirChange = 1 ;
uint8   setSpeed ;
uint8   knob ;

uint16 eeAddress1  = 0 ;

volatile unsigned long long oldState ; // 64 bits

void message( String mess, int val1, int val2 )
{
    debugPort.print( mess ) ; debugPort.write(' ') ;
    debugPort.print( val1 ) ; debugPort.write(' ') ;
    debugPort.println( val2 ) ;
}

void setPoint( uint16 pointAddress, uint8 state )
{
    message("Xnet Point set:", pointAddress, state ) ;
    //setEvent( pointSet ) ;
    Xnet.SetTrntPos( pointAddress - 1, state, 1 ) ;                             // BUG needs to be wrapper function, no acces to Xnet object here
    POINT_DELAY( 20 ) ;
    Xnet.SetTrntPos( pointAddress - 1, state, 0 ) ;
}

// void notifyXNetFeedback(uint16_t Address, uint8_t data)
// {
//     //message("feedback", Address, data) ;
// }

// void setFunc( uint8 val )
// {
//     pinMode(A7, INPUT) ;                                                     // desperate method to prevent linker from optimizing this function away.
//     passPoint( (Address+1) | (data<<15) ) ;
// }


   // EEPROM.write( eeAddress++, val ) ;

// void notifyXNetTrnt(uint16_t Address, uint8_t data)
// {
//   //  static uint8 counter = 0 ;
//     pinMode(A7, INPUT) ;                                                        // desperate method to prevent linker from optimizing this function away.
    
//     if( bitRead(data,3) == 0x01 )
//     { 
//         pinMode( A7, INPUT ) ; 

//         data &= 0x1 ;                                                           // clears all but last bit

//         if( data & 0x01 ) { /*digitalWrite( led, HIGH ) ;*/ }
//         else              { /*digitalWrite( led,  LOW ) ;*/ }

//         passPoint( Address | (data<<15) ) ;

//         message( "Xnet Point received", Address, data ) ;
//     }
// }

void notifyXNetTrnt(uint16_t Address, uint8_t data) 
{
    if( bitRead(data,3) == 1 )
    { 
        data &= 0x1 ;
        passPoint( Address | (data<<15) ) ;

        message( "Xnet Point received", Address, data ) ;
    }
}

void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )                   
{
    //return ; // DELETE ME

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

    if( knob != prevKnob )
    {
        prevKnob = knob ;
        message("knob ", knob, Speed ) ;
    }

    if( Address == 6 )
    {
        setSpeed = Speed ;
    }
}



void functionPressed ( uint16 Address, uint8 func, uint8 bank ) // bank is verivied, address is verivied, Address is verivied
{
    if( Address != 1)
    {
        message("wrong address, ignoring function", Address, func ) ;
        return ;
    }

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

            message("point set:", pointNumber, state ) ;
            //message("Fkey & bank", fKey, bank ) ;

            setPoint( pointNumber, state ) ;
            return ;
        }
    }
}

void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { functionPressed( Address, Func1,   F0_F4 ) ; } //              F0  F4  F3  F2  F1
void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { functionPressed( Address, Func2,   F5_F8 ) ; } //                  F8  F7  F6  F5
void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) { dirChange ^= 1 ; functionPressed( Address, Func3,  F9_F12 ) ; } //                 F12 F11 F10  F9
void notifyXNetLocoFunc4( uint16_t Address, uint8_t Func4 ) { functionPressed( Address, Func4, F13_F20 ) ; } // F20 F19 F18 F17 F16 F15 F14 F13


void notifyXNetPower(uint8_t State)
{

    message("POWER", State , 0xFF ) ;
    //if( State == csNormal ) { /*digitalWrite(led, HIGH);*/ }
    //else                    { /*digitalWrite(led,  LOW);*/ }
}
void setup()
{
    uint16_t eeAddress = 0 ;
    setMode( idling ) ;


    //initIO() ;

    Xnet.setup( Loco28,  2) ;
    debugPort.begin( 9600 ) ;   
    message("multimause enhancer booted",3,5);
}

void readSerialBus()                                                            // in debug mode, we can manually send switch commands to store in EEPROM
{
    // if( debugPort.available() > 0)
    // {
    //     uint16 recv = 0 ;

    //     POINT_DELAY( 2000 ) ;                                                   // some time to receive more bytes,  updates Xnet in the meantime
    //     while( debugPort.available() > 0 )
    //     {
    //         recv *= 10 ;
    //         recv += ( debugPort.read() - '0' ) ;
    //     }
    //     uint16 address = recv / 10 ;
    //     uint16   state = recv % 10 ;

    //     passPoint( address | (state << 15) ) ;    

    //     message( "Point ", address, state ) ;    
    // }
}


void loop()
{
    handlePoints() ;
    //eventHandler() ;

    Xnet.update() ;
    //readSerialBus() ;

    REPEAT_MS( 200 )
    {
        static uint8 prevSpeed = 0 ;
        if( prevSpeed != setSpeed )
        {   prevSpeed  = setSpeed ;

            int8 speedTemp = setSpeed ;
            Xnet.setSpeed( 7, Loco128, speedTemp ) ;
            Xnet.setSpeed( 12, Loco128, speedTemp ) ;
        }
    } END_REPEAT

}