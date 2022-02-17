#include "src/version.h"
#include "src/macros.h"
#include "src/XpressNetMaster.h"

#define RS485DIR 2
#define  F1_F4  0x00
#define  F5_F8  0x40
#define F9_F10  0x80

XpressNetMasterClass Xnet ;

uint8   group ;

uint8_t lookUpSpeed( uint8_t speed )
{
    switch( speed )
    {
    case 0b00000 : return  0 ; //  0
    case 0b00010 : return  1 ; //  2
    case 0b10010 : return  2 ; // 18
    case 0b00011 : return  3 ; //  3
    case 0b10011 : return  4 ; // 19
    case 0b00100 : return  5 ; //  4
    case 0b10100 : return  6 ; // 20
    case 0b00101 : return  7 ; //  5
    case 0b10101 : return  8 ; // 21
    case 0b00110 : return  9 ; //  6
    case 0b10110 : return 10 ; // 22 
    case 0b00111 : return 11 ; //  7
    case 0b10111 : return 12 ; // 23
    case 0b01000 : return 13 ; //  8
    case 0b11000 : return 14 ; // 24
    case 0b01001 : return 15 ; //  9
    case 0b11001 : return 16 ; // 25
    case 0b01010 : return 17 ; // 10
    case 0b11010 : return 18 ; // 26
    case 0b01011 : return 19 ; // 11
    case 0b11011 : return 20 ; // 27
    case 0b01100 : return 21 ; // 12
    case 0b11100 : return 22 ; // 28
    case 0b01101 : return 23 ; // 13
    case 0b11101 : return 24 ; // 29
    case 0b01110 : return 25 ; // 14
    case 0b11110 : return 26 ; // 30
    case 0b01111 : return 27 ; // 15
    case 0b11111 : return 28 ; // 31
    }
}
void notifyXNetLocoDrive28( uint16_t Address, uint8_t Speed )                   
{
    group = lookUpSpeed( Speed & 0b00011111 ) ;
    group = map( group, 0, 28, 0, 4 ) ;                                         // map 28 speedsteps to 5 regions TEST ME
}

void setPoint( uint8_t Address, uint8_t _functions )
{
    static uint16 functions ;

    if(      _functions & F9_F10 ) { }
    else if( _functions & F5_F8  ) { }
    else /*               F1_F4 */ { }

    if( Address != 1) return ;                                                  // only loco adress 1 is used


    // for( int bitMask = 0b0001 ; bitMask <= 0b100 ; bitMask <<= 1 )
    // {
    //     if( (functions & bitMask) != (prevStates[ Address ] & bitMask) )        // check all 4 bits for F1 - F4, if atleast 1 bit has changed
    //     {
    //         prevStates[ Address ] = functions & 0x0F ;

    //         uint8_t pointNumber = ((Address - 1) * 10) ;
    //         uint8_t state ;

    //         if( functions & bitMask ) state = 1 ;    // on
    //         else                      state = 0 ;    // off        

    //         /*
            
    //         Xnet.SetTrntPos( pointNumber, state, 1 ) ;
    //         delay(20) ;                                 // needed?
    //         Xnet.SetTrntPos( pointNumber, state, 0 ) ;  // needed?
    //         */
    //         return ;
    //     }

    // }
}


void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { setPoint( Address, Func1 | F1_F4  ) ; } // Gruppe1 0 0 0 F0    F4  F3  F2  F1
void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { setPoint( Address, Func2 | F5_F8  ) ; } // Gruppe2     0000    F8  F7  F6  F5
void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func3 ) { setPoint( Address, Func3 | F9_F10 ) ; } // Gruppe3     0000   F12 F11 F10  F9


void setup()
{
    Xnet.setup( Loco28, RS485DIR ) ;
}

void loop()
{
    Xnet.update() ;
}