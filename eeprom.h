#include <Arduino.h>
#include "src/macros.h"

const int nPointsPerStreet = 10 ;
const int nStreets = 20 ;


extern void clear( uint8 streetIndex ) ;
extern void addPoint( uint16 point ) ;
