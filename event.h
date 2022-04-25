#include <Arduino.h>
#include "src/macros.h"

enum defaultEvents
{
    FEEDBACK,
    START,
    STOP,
} ;

enum eepromTypes
{
    INTERNAL_EEPROM,
    I2C_EEPROM, 
} ;

typedef struct 				// 8 bytes per event
{
	uint8 	data1 ;
	uint16 	data2 ;
	uint8	data3 ;
	uint32  time2nextEvent ;
} Event ;


class EventHandler
{
public:
    EventHandler( uint32, uint8 ); // enter EEPROM ADDRESS AND STUFF
    
    void    startRecording() ;      // need begin function? to init I2c bus??
    void    stopRecording() ;
    void    startPlaying() ;
    void    stopPlaying() ;
    void    resetProgram() ;
    void    sendFeedbackEvent( uint16 ) ;
    void    update() ;
    void    storeEvent( uint8, uint16, uint8 ) ;

private:
    Event   event ;
    Event   getEvent() ;


    uint16  I2Caddress ;
    uint16  eeAddress ;
    uint32  prevTime ;
    uint16  newSensor ;
    uint8   recordingDevice ;
    uint8   eepromType ;


};


// callback function
extern void notifyEvent( uint8, uint16, uint8 ) __attribute__ (( weak )) ;
