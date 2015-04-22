/*
  Config file for ImplementPlough
Copyright (C) 2011-2015 J.A. Woltjer.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ConfigImplementPlough_h
#define ConfigImplementPlough_h

//#define VOORSERIE
#define GPS
#define ROTATION
//#define DEBUG

#ifndef VOORSERIE
// Digital inputs 12V -> 5V conversion
#define PLOUGHSIDE_PIN      A2    // for hall position sensor (input 1 connector)
//#define xxx               A3    // (input 2 connector)

// Digital outputs
#define OUTPUT_LED          17

// FET outputs (PWM)
#define OUTPUT_NARROW       9
#define OUTPUT_WIDE         10
#define OUTPUT_BYPASS       11
//#define xxx               12

// Analog input
#define POSITION_SENS_PIN   A0    // for potmeter input (input 1 connector)
#define ROTATION_SENS_PIN   A1    // (input 2 connector)

#else

#define PLOUGHSIDE_PIN      10    // for hall position sensor (input 1 connector)
//#define xxx               A3    // (input 2 connector)

// Digital outputs
#define OUTPUT_LED          13

// FET outputs (PWM)
#define OUTPUT_NARROW       A2
#define OUTPUT_WIDE         A1
#define OUTPUT_BYPASS       A0
//#define xxx               12

// Analog input
#define POSITION_SENS_PIN   A3    // for potmeter input (input 1 connector)
#define ROTATION_SENS_PIN   A4    // (input 2 connector)

#endif

#define SHUTOFF             3000 // 3 seconds

#endif
