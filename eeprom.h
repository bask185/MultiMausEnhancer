#include <Arduino.h>
#include "src/macros.h"

const int nPointsPerStreet = 16 ;
const int nStreets = 16 ;


extern void clearStreet( uint8 streetIndex ) ;
extern void addPoint( uint16 point ) ;
