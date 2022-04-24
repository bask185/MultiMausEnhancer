#include <EEPROM.h>
#include "event.h"


//extern void message( String mess, uint16 val1, uint16 val2 ) ;

typedef struct someName 				// 8 bytes per event
{
	uint8 	data1 ;
	uint16 	data2 ;
	uint8	data3 ;
	uint32  time2nextEvent ;
} Event ;

Event   event ;

enum eventModes
{
    idle,
    playing,
    recording,
    finishing,
} ;


const int       baseEEaddress = 0x14 ;
static uint16   eeAddress ;
static uint32   prevTime ;
static uint16   newSensor ;
uint8           recordingDevice = idle ;

void startRecording() 
{
    if( recordingDevice == idle )
    {
        eeAddress = 0 ;                     // set EEPROM adres to 0
        recordingDevice = recording ;       
        prevTime = millis() ;               // record starting time
        storeEvent( START, 1, 1 ) ;
    }
}

void stopRecording() 
{
    storeEvent( STOP, 1, 1 ) ;
    if( recordingDevice == recording )
    {
        recordingDevice = idle ;
        storeEvent( STOP, 1, 1 ) ;
    }
}

Event getEvent()
{
    Event localEvent ;

    EEPROM.get( eeAddress, localEvent ) ;

    eeAddress += sizeof( localEvent ) ;            // increase EEPROM address for next event ;

    return localEvent ;
}


void startPlaying() 
{
    if( recordingDevice == idle )
    {
        eeAddress = 0 ;
        event = getEvent() ;            // should load the start event

        prevTime = millis() ;
        recordingDevice = playing ;
    }
}
void stopPlaying() 
{
    if( recordingDevice == playing )
    {
        recordingDevice = finishing ;
    }
}

//    feedback 0
//    start
//    stop
//    accessoryEvent = 3,       // 0,1,2 are used for feedback, start and stop
//    speedEvent 4,
//    F0_F4Event, 5
//    F5_F8Event, 6
//    F9_F12Event, 7
//    F13_F20Event, 8



void storeEvent( uint8 _data1, uint16 _data2, uint8 _data3 )
{
    if( recordingDevice != recording ) return ;

    Event     localEvent ;
    uint32    currTime = millis() ;

    localEvent.data1 = _data1 ;
    localEvent.data2 = _data2 ;
    localEvent.data3 = _data3 ;

    if( _data1 == FEEDBACK ) { localEvent.time2nextEvent = 0 ; }                       // feedback has 0 time
    else                     { localEvent.time2nextEvent = currTime - prevTime ; }

    prevTime = millis() ;

    EEPROM.put( eeAddress, localEvent ) ;

    eeAddress += sizeof( localEvent ) ;            // increase EEPROM address for next event ;
}



void sendFeedbackEvent( uint16 number )
{
    newSensor = number ;
}

void eventHandler()
{
    uint32 currTime = millis() ;

    if( (recordingDevice == playing || recordingDevice == finishing)
    && (currTime - prevTime) >= event.time2nextEvent )
    {
        // if( event.time2nextEvent == 0 )                                         
        // {
        //     if( newSensor == event.data2 ) event.time2nextEvent = 1 ; NOT PRESENT IN THIS UNIT
        //     return ;
        // }
                                       //    8bit         16bit        8bit       // for here and now, data1, is type, data2 is address and data 3 just data.
        if( notifyEvent ) notifyEvent( event.data1, event.data2, event.data3 ) ;

        prevTime = currTime ;
        
        if( event.data1 == STOP )
        {
            if( recordingDevice == finishing )
            {
                recordingDevice = idle ;
                return ;
            }
            else
            {
                recordingDevice = idle ;
                startPlaying() ;
            }
        }

        event = getEvent() ; 

        newSensor = 0 ;
    }
}