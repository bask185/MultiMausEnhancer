#include "src/version.h"
#include "src/io.h"
#include "src/macros.h"
#include "src/io.h"
#include "src/debounceClass.h"
#include "XpressNetMaster.h"
#include <EEPROM.h> // needs to be removed in the future, preferebly
#include <SoftwareSerial.h>
#include "points.h"
#include "shuttle.h"
#include "event.h"

SoftwareSerial debugPort(3,4) ;
Debounce sensor( 5 ) ;

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


uint8   setSpeed ;
uint8   knob ;
uint8   newSensor ;


/****** recording / playing programs *******/
enum eventModes
{
	idle,
	playing,
	recording,
} ;

uint32  nextInterval ;
uint32  prevTime ;
Event   event ;
uint8   recordingDevice = idle ;
bool    playingAllowed ;


volatile unsigned long long oldState ; // 64 bits

void message( String mess, int val1, int val2 )
{
    debugPort.print( mess ) ; debugPort.write(' ') ;
    debugPort.print( val1 ) ; debugPort.write(' ') ;
    debugPort.println( val2 ) ;
}

void setPoint( uint16 pointAddress, uint8 state )
{
    // message(F("Xnet Point set:"), pointAddress, state ) ;                    // works fine
    //setEvent( pointSet ) ;
    Xnet.SetTrntPos( pointAddress - 1, state, 1 ) ;                             // BUG needs to be wrapper function, no acces to Xnet object here
    POINT_DELAY( 20 ) ;
    Xnet.SetTrntPos( pointAddress - 1, state, 0 ) ;
}

// void notifyXNetFeedback(uint16_t Address, uint8_t data)                      // to be used for future debugging
// {
//     //message("feedback", Address, data) ;
// }

void notifyXNetTrnt(uint16_t Address, uint8_t data) 
{
    if( bitRead(data,3) == 1 )
    { 
        data &= 0x1 ;
        passPoint( Address | (data<<15) ) ;

        message(F( "Xnet Point received"), Address, data ) ;
        
        if(      Address == 997 && data == 0 ) { message(F("playing started"  ), 0, 0 ) ; playingAllowed = true ; startPlaying() ; recordingDevice = playing ; event = getEvent() ; }
        else if( Address == 997 && data == 1 ) { message(F("playing stopped"  ), 0, 0 ) ; playingAllowed = false ; }
        else if( Address == 998 && data == 0 ) { message(F("recording started"), 0, 0 ) ; storeEvent( event_start, 0, 0 ) ; recordingDevice = recording ;}
        else if( Address == 998 && data == 1 ) { message(F("recording stopped"), 0, 0 ) ; storeEvent( event_stop,  0, 0 ) ; recordingDevice = idle ; }
        else if( recordingDevice == recording )
        {
            storeEvent( event_point, Address, data ) ;
        }
    }   
}

void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )                   
{
    //return ; // DELETE ME

    if( recordingDevice == recording )
    {
        storeEvent( event_speed, Address, Speed ) ;
        return ;
    }

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
        message(F("knob "), knob, Speed ) ;
    }

    if( Address == 6 )
    {
        setSpeed = Speed ;
    }
}



void functionPressed ( uint16 Address, uint8 func, uint8 bank )                 // bank is verivied, address is verivied, Address is verivied
{
    if( Address != 1)
    {
        if( recordingDevice == recording )
        {
            switch( bank )
            {
            case    F0_F4: storeEvent(   event_F0_F4, Address, func ) ; break ;
            case    F5_F8: storeEvent(   event_F5_F8, Address, func ) ; break ;
            case   F9_F12: storeEvent(  event_F9_F12, Address, func ) ; break ;
            case  F13_F20: storeEvent( event_F13_F20, Address, func ) ; break ;
            }
        }
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

            message(F("point set:"), pointNumber, state ) ;
            //message("Fkey & bank", fKey, bank ) ;

            setPoint( pointNumber, state ) ;
            return ;
        }
    }
}

void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { functionPressed( Address, Func1,   F0_F4 ) ; } //              F0  F4  F3  F2  F1
void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { functionPressed( Address, Func2,   F5_F8 ) ; } //                  F8  F7  F6  F5
void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) { functionPressed( Address, Func3,  F9_F12 ) ; } //                 F12 F11 F10  F9
void notifyXNetLocoFunc4( uint16_t Address, uint8_t Func4 ) { functionPressed( Address, Func4, F13_F20 ) ; } // F20 F19 F18 F17 F16 F15 F14 F13


void notifyXNetPower(uint8_t State)
{

    message(F("POWER"), State , 0xFF ) ;
    //if( State == csNormal ) { /*digitalWrite(led, HIGH);*/ }
    //else                    { /*digitalWrite(led,  LOW);*/ }
}
void setup()
{
    uint16_t eeAddress = 0 ;
    // setMode( idling ) ;


    //initIO() ;

    Xnet.setup( Loco28,  2) ;
    debugPort.begin( 9600 ) ;   
    message(F("multimause enhancer booted"),3,5);
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
    REPEAT_MS( 20 )
    {
        sensor.debounce() ;

    } END_REPEAT ;

    if( sensor.getState() == FALLING )
    {
        if( recordingDevice == recording )
        {
            storeEvent( event_feedback, 123, 1 ) ;                              // hardcoded sensor to 123 for testing
        }
        else if( recordingDevice == playing )
        {
            newSensor = 123 ;                                                   // hardcoded sensor to 123 for testing
        }
    }

    uint32 currTime = millis() ;

    if( recordingDevice == playing && (currTime - prevTime) >= nextInterval )
    {
        if( nextInterval == 0 ) // if interval is 0, we are waiting on a sensor or feedback thing to continu.
        {
            if( newSensor == event.address ) nextInterval = 1 ;
            return ;
        }
        switch( event.type )
        {
        case event_start:    message(F("player: started"), 0, 0 ) ; break ;
        case event_speed:    message(F("player: setting speed"), event.address, event.data ) ; Xnet.setSpeed( event.address, Loco128, event.data ) ; break ;
        case event_F0_F4:    message(F("player: F0-F4"),         event.address, event.data ) ; Xnet.setFunc0to4(   event.address, event.data ) ;     break ;
        case event_F5_F8:    message(F("player: F5-F8"),         event.address, event.data ) ; Xnet.setFunc5to8(   event.address, event.data ) ;     break ;
        case event_F9_F12:   message(F("player: F9-F12"),        event.address, event.data ) ; Xnet.setFunc9to12(  event.address, event.data ) ;     break ;
        case event_F13_F20:  message(F("player: F13-F20"),       event.address, event.data ) ; Xnet.setFunc13to20( event.address, event.data ) ;     break ;
        case event_point:    message(F("player: setting point"), event.address, event.data ) ;           setPoint( event.address, event.data ) ;     break ;
        case event_feedback: message(F("player: feedback"),      event.address, event.data ) ;                                                       break ;
        case event_stop:
            if( playingAllowed == false ) {
                             message(F("player: program stopped"),   0, 0 ) ;                   recordingDevice = idle ; }
            else {           message(F("player: program resetting"), 0, 0 ) ;                   startPlaying() ;         }                           break ;

        }

        prevTime = currTime ;
        event = getEvent() ;     
        nextInterval = event.time2nextEvent ;                                   // load timer to next event ;
        newSensor = 0 ;
        message(F("next event"), event.type, nextInterval/100 ) ;               // display message 0.1s
    }

    handlePoints() ;
    //eventHandler() ;                                                          // handles LED to give status indictation

    Xnet.update() ;
}