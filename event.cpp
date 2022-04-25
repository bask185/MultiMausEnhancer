#include <EEPROM.h>
#include "event.h"

//extern void message( String mess, uint16 val1, uint16 val2 ) ;



enum eventModes
{
    idle,
    playing,
    recording,
    finishing,
} ;

EventHandler::EventHandler( uint32 _I2Caddress, uint8 _eepromType )
{
    recordingDevice = idle ;
    I2Caddress  = _I2Caddress ;
    eepromType  = _eepromType ;

    if( eepromType == I2C_EEPROM )                                              // move this stuff to a 'begin' function
    {
        static bool initI2cBus = false ;    
        if( initI2cBus == false )
        {   initI2cBus  = true ;

            // call the function to init I2C bus
        }
    }
}

void EventHandler::startRecording() 
{
    if( recordingDevice == idle )
    {
        eeAddress = 0 ;                     // set EEPROM adres to 0
        recordingDevice = recording ;       
        prevTime = millis() ;               // record starting time
        storeEvent( START, 1, 1 ) ;
    }
}

void EventHandler::stopRecording() 
{
    storeEvent( STOP, 1, 1 ) ;
    if( recordingDevice == recording )
    {
        recordingDevice = idle ;
        storeEvent( STOP, 1, 1 ) ;
    }
}

Event EventHandler::getEvent()
{
    Event localEvent ;

    if( eepromType == INTERNAL_EEPROM )   EEPROM.get( eeAddress, localEvent ) ;
    //else                               i2cEeprom.get( eeAddress, localEvent ) ;

    eeAddress += sizeof( localEvent ) ;            // increase EEPROM address for next event ;

    return localEvent ;
}


void EventHandler::startPlaying() 
{
    if( recordingDevice == idle )
    {
        eeAddress = 0 ;
        event = getEvent() ;                                                    // should load the start event

        prevTime = millis() ;
        recordingDevice = playing ;
    }
}
void EventHandler::stopPlaying() 
{
    if( recordingDevice == playing )
    {
        recordingDevice = finishing ;
    }
}

void EventHandler::resetProgram() 
{
    recordingDevice = idle ;
}


void EventHandler::storeEvent( uint8 _data1, uint16 _data2, uint8 _data3 )
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

    if( eepromType == INTERNAL_EEPROM )   EEPROM.put( eeAddress, localEvent ) ;
    //else                               i2cEeprom.put( eeAddress, localEvent ) ;

    eeAddress += sizeof( localEvent ) ;            // increase EEPROM address for next event ;
}


void EventHandler::sendFeedbackEvent( uint16 number )
{
    newSensor = number ;
}

void EventHandler::update()
{
    uint32 currTime = millis() ;

    if( (recordingDevice == playing || recordingDevice == finishing)
    && (currTime - prevTime) >= event.time2nextEvent )
    {
        if( event.time2nextEvent == 0 )                                         
        {
            if( newSensor == event.data2 ) event.time2nextEvent = 1 ;
            return ;
        }
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