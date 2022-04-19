// #include "event.h"
// #include "src/io.h"

// const int nEvents   = 10 ;
// const int nMaxFlash =  5 ;

// uint8  blinkCounter ;
// uint8  nBlinks ;
// uint8  leds ;
// uint8  event ;
// uint16 blinkSpeed ;

// uint8 mode ;

// uint32 prevTime ;
// const int green   = 0b001 ;
// const int yellow  = 0b010 ;
// const int red     = 0b100 ;

// void flash( uint8 _leds, uint8 blinks, uint16 _blinkSpeed )
// {
//     blinkCounter = 0 ;
//     blinkSpeed = _blinkSpeed ;
//     nBlinks = blinks * 2 ;
//     leds = _leds ;
//     prevTime = millis() ;
// }

// void lightHandler()
// {
//     if( event != 0 && (millis() - prevTime > blinkSpeed ) )                             // If event is set, and time is expired, blink led
//     {
//         prevTime = millis() ;

//         if( blinkCounter % 2 == 0 )
//         {
//             if( leds &  green ) digitalWrite(  greenLed, HIGH ) ;
//             if( leds & yellow ) digitalWrite( yellowLed, HIGH ) ;
//             if( leds &    red ) digitalWrite(    redLed, HIGH ) ;
//         }
//         else
//         {
//             digitalWrite(  greenLed, LOW ) ;
//             digitalWrite( yellowLed, LOW ) ;
//             digitalWrite(    redLed, LOW ) ;
//         }
//         if( ++ blinkCounter == nBlinks ) { event = 0 ; }                        // if blinked this many times, turn off blinking
//     }

//     if( event == 0 )
//     {
//         if( mode == running )
//         {
//             digitalWrite(   greenLed, HIGH ) ;
//             digitalWrite(  yellowLed,  LOW ) ;
//             digitalWrite(     redLed,  LOW ) ;
//         }
//         if( mode == teachin )
//         {
//             REPEAT_MS( SLOW )
//             {
//                 digitalWrite(   greenLed, LOW ) ;
//                 digitalWrite(  yellowLed, LOW ) ;
//                 digitalWrite(     redLed, !digitalRead( redLed ) ) ;
//             } END_REPEAT
//         }
//         if( mode == OFF )
//         {
//             digitalWrite(   greenLed,  LOW ) ;
//             digitalWrite(  yellowLed,  LOW ) ;
//             digitalWrite(     redLed,  LOW ) ;
//         }
//     }
// }


// void setLights( uint8 event )
// {
//     switch( event )
//     {
//         // case runModeEnabled:    flash( green           ,   5, FAST ) ; break ;
//         // case teachingEnabled:   flash( yellow          ,   5, FAST ) ; break ;
//         // case turnedOff:         flash( red             ,   5, FAST ) ; break ; 
//         // case detectorMade:      flash( yellow          ,   3, FAST ) ; break ;
//         // case trainBraking:      flash( green | red     ,   1, SLOW ) ; break ;
//         // case trainArrived:      flash( green | red     ,   2, SLOW ) ; break ;
//         // case trainDeparting:    flash( green | yellow  ,   1, SLOW ) ; break ;
//         // case trainDeparted:     flash( green | yellow  ,   2, SLOW ) ; break ;
//         // case trainNotDeparted:  flash( red             ,   5, FAST ) ; break ;
//         // case slotAdded:         flash( green|yellow|red,   4, FAST ) ; break ;
//         // case slotNotFound:      flash( red             ,   3, FAST ) ; break ;
//         // case slotLoaded:        flash( green  | yellow ,   3, FAST ) ; break ;
//         // case waiting4address:   flash( green  | yellow ,   1, FAST ) ; break ;
//         // case addressReceived:   flash( green  | yellow ,   2, FAST ) ; break ;
//     }
// }

// /*  runModeEnabled,
//     teachingEnabled,  
//     detectorMade,           // 3 flashes orange
//     trainBreaking,          // 1 flashes green
//     trainArrived,
//     trainDeparting,         // 2 flashes green
//     trainNotDeparted,       // 5 flashes red
//     trainDeparted,          //
//     slotAdded,              // 5 flashes yellow
//     slotNotFound,           // 5 flashed red
//     slotLoaded              // 5 flashes green*/