#include "event.h"
#include "src/io.h"

const int nEvents   = 10 ;
const int nMaxFlash =  5 ;

uint8  blinkCounter ;
uint8  nBlinks ;
uint8  leds ;
uint8  event ;
uint16 blinkSpeed ;

uint8 mode ;

uint32 prevTime ;
const int green   = 0b001 ;
const int yellow  = 0b010 ;
const int red     = 0b100 ;

void flash( uint8 _leds, uint8 blinks, uint16 _blinkSpeed )
{
    digitalWrite(  greenLed, LOW ) ;
    digitalWrite( yellowLed, LOW ) ;
    digitalWrite(    redLed, LOW ) ;

    blinkCounter = 0 ;
    blinkSpeed = _blinkSpeed ;
    nBlinks = blinks * 2 ;
    leds = _leds ;
    prevTime = millis() ;
}

void eventHandler()
{
    if( event != 0 && (millis() - prevTime > blinkSpeed ) )                             // If event is set, and time is expired, blink led
    {
        prevTime = millis() ;

        if( blinkCounter % 2 == 0 )
        {
            if( leds &  green ) digitalWrite(  greenLed, HIGH ) ;
            if( leds & yellow ) digitalWrite( yellowLed, HIGH ) ;
            if( leds &    red ) digitalWrite(    redLed, HIGH ) ;
        }
        else
        {
            digitalWrite(  greenLed, LOW ) ;
            digitalWrite( yellowLed, LOW ) ;
            digitalWrite(    redLed, LOW ) ;
        }
        if( ++ blinkCounter == nBlinks ) { event = 0 ; }                        // if blinked this many times, turn off blinking
    }

    if( event == 0 )
    {
        if( leds &  green )  { digitalWrite(  greenLed, HIGH ) ; } 
        else {                 digitalWrite(  greenLed,  LOW ) ; }
        if( leds & yellow )  { digitalWrite( yellowLed, HIGH ) ; } 
        else {                 digitalWrite( yellowLed,  LOW ) ; }
        if( leds &    red )  { digitalWrite(    redLed, HIGH ) ; } 
        else {                 digitalWrite(    redLed,  LOW ) ; }
    }
}

void setMode( uint8 _mode )
{
    leds = 0 ;
    switch( _mode )
    {
        case idling :           flash( red            , 0,   ON ) ; break ;
        case teachin :          flash( yellow         , 0,   ON ) ; break ;
        case settingPoints :    flash( green          , 0,   ON ) ; break ;
    }
}

void setEvent( uint8 _event )
{
    event = _event ;
    switch( event )
    {
        case pointSet:          flash( green | yellow  , 2, FAST ) ; break ;
        case enteringTeachin:   flash( yellow          , 3, SLOW ) ; break ;
        case indexReceived:     flash( yellow          , 1, SLOW ) ; break ; 
        case leavingTeachin:    flash( yellow          , 3, SLOW ) ; break ;
        case pointAdded:        flash( yellow          , 3, FAST ) ; break ;
    }
}
