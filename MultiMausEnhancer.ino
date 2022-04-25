#include "src/version.h"
#include "src/io.h"
#include "src/macros.h"
#include "src/io.h"
#include "src/debounceClass.h"
#include "XpressNetMaster.h"
#include <EEPROM.h> // needs to be removed in the future, preferebly
#include <SoftwareSerial.h>
#include "points.h"
//#include "shuttle.h"
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
EventHandler eventHandler( 0, INTERNAL_EEPROM ) ;

uint8   setSpeed ;
uint8   knob ;
uint8   prevNibble[256] ;


enum events
{
    accessoryEvent = 3,       // 0,1,2 are used for feedback, start and stop
    speedEvent,
    F0_F4Event,
    F5_F8Event,
    F9_F12Event,
    F13_F20Event,
} ;

uint32  nextInterval ;
uint32  prevTime ;
volatile unsigned long long oldState ; // 64 bits

void message( String mess, uint16 val1, uint16 val2 )
{
    debugPort.print( mess ) ; debugPort.write(' ') ;
    debugPort.print( val1 ) ; debugPort.write(' ') ;
    debugPort.println( val2 ) ;
}

void setPoint( uint16 pointAddress, uint8 state )
{
    // message(F("Xnet Point set:"), pointAddress, state ) ;                    // works fine
    //setEvent( pointSet ) ;
    Xnet.SetTrntPos( pointAddress, state, 1 ) ;                             // BUG needs to be wrapper function, no acces to Xnet object here
    POINT_DELAY( 20 ) ;
    Xnet.SetTrntPos( pointAddress, state, 0 ) ;
}


void notifyXNetTrnt(uint16_t Address, uint8_t data) 
{
    if( bitRead(data,3) == 1 )
    { 
        data &= 0x1 ;
        passPoint( Address | (data<<15) ) ;

        //message(F( "Xnet Point received"), Address, data ) ;
        
        if(      Address == 997 && data == 0 ) { eventHandler.startPlaying()   ; } // may be replaced to a loco address with functions?
        else if( Address == 997 && data == 1 ) { eventHandler.stopPlaying()    ; }
        else if( Address == 998 && data == 0 ) { eventHandler.startRecording() ; }
        else if( Address == 998 && data == 1 ) { eventHandler.stopRecording()  ; }
        else
        {
            eventHandler.storeEvent( accessoryEvent, Address, data ) ;
        }
    }   
}

void notifyXNetLocoDrive28( uint16_t Address, uint8_t Speed )                   
{
    eventHandler.storeEvent( speedEvent, Address, Speed ) ;  /// NOTE, z21 used this, DR5000 forces everything to 128?

    static uint8 state = 0 , prevKnob = 0xFF ;
    int8_t speed ;

    speed = Speed & 0x7F ;

    if( speed > 0 ) speed -- ;
    if( Speed & 0x80 ) speed = -speed ;
   
    if(         speed <  -20              ) knob = 4 ;
    else if(    speed >= -5 && speed < -5 ) knob = 3 ;
    else if(    speed >  -5 && speed <  5 ) knob = 2 ;
    else if(    speed <=  5 && speed >  5 ) knob = 1 ;
    else if(    speed >   20              ) knob = 0 ;

    if( knob != prevKnob )
    {
        prevKnob = knob ;
        message(F("knob 28 "), knob, Speed ) ;
    }
}

void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )                   
{
    //return ; // DELETE ME

    eventHandler.storeEvent( speedEvent, Address, Speed ) ;

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
        message(F("knob 128"), knob, Speed ) ;
    }
}



void functionPressed ( uint16 Address, uint8 func, uint8 bank )                 // bank is verivied, address is verivied, Address is verivied
{
    if( Address == 6 && bank == F0_F4 )
    {
        if( func & 0b0001 ) eventHandler.startPlaying() ;
        if( func & 0b0010 ) eventHandler.stopPlaying() ;
        if( func & 0b0100 ) eventHandler.startRecording() ;
        if( func & 0b1000 ) eventHandler.stopRecording() ;
        return ;
    }

    if( Address != 1 )
    {
        switch( bank )
        {
        case    F0_F4: eventHandler.storeEvent(   F0_F4Event, Address, func ) ; break ;
        case    F5_F8: eventHandler.storeEvent(   F5_F8Event, Address, func ) ; break ;
        case   F9_F12: eventHandler.storeEvent(  F9_F12Event, Address, func ) ; break ;
        case  F13_F20: eventHandler.storeEvent( F13_F20Event, Address, func ) ; break ;
        }
        return ;
    }
    
    // following code is only for address 1

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

            oldState ^= (1 << fKey );                                           // toggle previous state SK: HOW CAN THIS WORK?
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

void notifyEvent( uint8 type, uint16 address, uint8 data )                            // CALL BACK FUNCTION FROM EVENT.CPP
{
    switch( type )
    {
        case START:          message(F("player: started"), 0, 0 ) ;                                                               break ; // flash an led?
        case speedEvent:     message(F("player: setting speed"), address, data ) ; Xnet.setSpeed(      address, Loco128, data ) ; break ; 
        case F0_F4Event:     message(F("player: F0-F4"),         address, data ) ; Xnet.setFunc0to4(   address,data ) ;           break ;
        case F5_F8Event:     message(F("player: F5-F8"),         address, data ) ; Xnet.setFunc5to8(   address,data ) ;           break ;
        case F9_F12Event:    message(F("player: F9-F12"),        address, data ) ; Xnet.setFunc9to12(  address,data ) ;           break ;
        case F13_F20Event:   message(F("player: F13-F20"),       address, data ) ; Xnet.setFunc13to20( address,data ) ;           break ;
        case accessoryEvent: message(F("player: setting point"), address, data ) ;           setPoint( address,data ) ;           break ;
        case FEEDBACK:       message(F("player: feedback"),      address, data ) ;                                                break ;
        case STOP:           message(F("player: program stopped"),   0, 0 ) ;                                                     break ; // flash an led?
    }
}


// ITT N ZZZZ

// I   If this bit is 1, the switching command requested has not been completed and the turnout
// has not reached its end position.
// For feedback modules this bit will always be 0 because the inputs of these modules can become
// only be 0 or 1.
//  TT = 0 0 : Address is accessory decoder without feedback
//  TT = 0 1 : Address is accessory decoder with feedback
//  TT = 1 0 : Address is a feedback module
//  N   This bit describes which nibble of a turnout or feedback module this response describes.
//  N=0 is the lower nibble, N=1 the upper nibble.
void notifyXNetFeedback( uint16_t address, uint8_t state )                      // Xnet knows 127 feedback module addresses with 8 bits per address
{    
    if( state & 0b01000000 )                                                    // ITT = 010 for feedback modules
    {
        address >>= 2 ;
        address &= 0x0007 ;
        address <<= 3 ;

        uint8 newNibble = state & 0x0F ;

        uint8 index = address * 2 ;                                             // 2 nibbles per address
        if (state & 0b10000)
        {
            index ++ ;                                                          // if bit N is set, correct index to upper nibble Note index should be fine
            address += 4 ;
        }
        

        for( uint8 bitMask = 0x01 ; bitMask < 0x40 ; bitMask <<= 1 )            // check 4 bits
        {
            address ++ ;
            if( (newNibble & bitMask) != (prevNibble[index] & bitMask ) )
            {
                if( newNibble & bitMask )
                {
                    prevNibble[index] |=  bitMask ;
                    message("feedback", address, state) ;
                    eventHandler.storeEvent( FEEDBACK, address, 1  ) ;                           // for recording
                    eventHandler.sendFeedbackEvent( address ) ;   
                }
                else
                {
                    prevNibble[index] &= ~bitMask ;
                }
            }
        }
    }
}


void setup()
{
    uint16_t eeAddress = 0 ;

    //initIO() ;

    Xnet.setup( Loco128,  2) ;  // NOTE TEST ME WITH lOCO28 ON z21
    debugPort.begin( 9600 ) ;   
    message(F("multimause enhancer booted"),3,5);

    //Xnet.ReqLocoBusy( 99 ) ; 
}

void loop()
{
    // REPEAT_MS( 20 )
    // {
    //     sensor.debounce() ;

    // } END_REPEAT ;

    // if( sensor.getState() == FALLING )
    // {
    //     if( recordingDevice == recording )
    //     {
    //         storeEvent( event_feedback, 123, 1 ) ;                              // hardcoded sensor to 123 for testing LINKED TO INPUT PIN D5
    //     }
    //     else if( recordingDevice == playing )
    //     {
    //         newSensor = 123 ;                                                   // hardcoded sensor to 123 for testing
    //     }
    // }

    handlePoints() ;
    eventHandler.update() ;                                                            // handles playing program
    Xnet.update() ;
}