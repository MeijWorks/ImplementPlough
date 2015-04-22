/*
  ImplementPlanter - a libary for a planter
 Copyright (C) 2011-2015 J.A. Woltjer.
 All rights reserved.
 
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

#include "ImplementPlough.h"

//------------
// Constructor
//------------
ImplementPlough::ImplementPlough(VehicleGps * _gps){
  // Pin configuration
  // Inputs
  pinMode(PLOUGHSIDE_PIN, INPUT);
  
  // Outputs
  pinMode(OUTPUT_WIDE, OUTPUT);
  pinMode(OUTPUT_NARROW, OUTPUT);
  pinMode(OUTPUT_BYPASS, OUTPUT);
  pinMode(OUTPUT_LED, OUTPUT);

  // Analog IO
  analogReference(DEFAULT);
  pinMode(POSITION_SENS_PIN, INPUT);
  digitalWrite(POSITION_SENS_PIN, LOW);
  pinMode(ROTATION_SENS_PIN, INPUT);
  digitalWrite(ROTATION_SENS_PIN, LOW);

  // Get calibration data from EEPROM otherwise use defaults
  if (!readCalibrationData()){
    // Default offset calibration set 100, 101
    position_calibration_data[0] = 200;
    position_calibration_data[1] = 450;
    position_calibration_data[2] = 700;

    // Default xte calibration set 110, 111
    rotation_calibration_data[0] = 200;
    rotation_calibration_data[1] = 450;
    rotation_calibration_data[2] = 700;

    // PID constants 120, 121, 130, 131, 140, 141
    KP = 100;

    // pwm values for manual (150) and auto (160)
#ifdef PWM_MAN
    man_pwm = 90;
#else
    man_pwm = 255;
#endif

#ifdef PWM_AUTO
    auto_pwm = 70;
#else
    auto_pwm = 255;
#endif

    // error margin 170
    error = 2;
    
    // sideswap 190
    swap = false;
    
    // amount of shares 200
    shares = 4;
    
    // maximum correction 210
    max_correction = 50;
    
#ifdef DEBUG
    Serial.println("No calibration data found");
#endif
  }

  // Get latest offset from EEPROM
  readOffset(); // 180

  // Calibration points for position and xte
  position_calibration_points[0] = 34;
  position_calibration_points[1] = 42;
  position_calibration_points[2] = 50;
  position = 0;
  last_position = 0;

  rotation_calibration_points[0] = -90;
  rotation_calibration_points[1] =  0;
  rotation_calibration_points[2] =  90;
  rotation = 0;

  // Setpoint of ajust loop
  setpoint = 0;
  
  // Speed
  speed = 0;

  // PID variables
  P = 0;

  // End shutoff timers
  shutoff_time = SHUTOFF;
  shutoff_narrow = false;
  shutoff_wide = false;
  shutoff_timer = millis();

  // Update timer
  update_age = millis();
  update_flag = false;
    
  // Print calibration data
#ifdef DEBUG
  printCalibrationData();
#endif
  gps = _gps;
}

// --------------------------------------------
// Method for updating implement data using gps
// --------------------------------------------
void ImplementPlough::update(byte _mode, int _buttons){
  mode = _mode;
  // Update offset, xte, rotation and setpoint
  if (millis() - update_age >= 200){
    if (mode != 1){
      // Use buttons to set offset while not in manual mode
      setOffset(_buttons);
    }

    // Update xte, rotation and setpoint
    xte = gps->getXte();
    speed = gps->getSpeedMs();
    
    if (update_flag){
      update_flag = false;
      
      position = getActualPosition();
#ifdef ROTATION
      getActualRotation();
#endif
    }
    else {
      update_flag = true;
      
#ifdef ROTATION
      rotation = getActualRotation();
#endif
      getActualPosition();
    }
    
    setSetpoint();
    update_age = millis();    
  }
}

// ------------------------------
// Method for adjusting implement
// ------------------------------
void ImplementPlough::adjust(int _direction){
  int _actual_position;
  byte _pwm;

  if (mode < 2){
    _actual_position = getActualPosition();
    _pwm = auto_pwm;
  }
  else {
    _actual_position = setpoint - (_direction * (error + 1));
    _pwm = man_pwm;
  }
  
  // ---------------------------
  // Adjust tree including error
  // ---------------------------
  if (_actual_position < setpoint - error && !shutoff_narrow){
    // Setpoint < actual position
    digitalWrite(OUTPUT_WIDE, LOW);
    digitalWrite(OUTPUT_NARROW, HIGH);
    analogWrite(OUTPUT_BYPASS, _pwm);
    digitalWrite(OUTPUT_LED, HIGH);
    
    // End shutoff
    if (_actual_position != last_position){
      shutoff_timer = millis();
    }
    
    if (shutoff_wide){
      shutoff_narrow = false;
      shutoff_wide = false;
    }
    
    if (millis() - shutoff_timer > shutoff_time){
      shutoff_narrow = true;
      shutoff_wide = false;
    }
  }
  else if (_actual_position > setpoint + error && !shutoff_wide){
    // Setpoint < actual position
    digitalWrite(OUTPUT_WIDE, HIGH);
    digitalWrite(OUTPUT_NARROW, LOW);
    analogWrite(OUTPUT_BYPASS, _pwm);
    digitalWrite(OUTPUT_LED, HIGH);

    // End shutoff
    if (_actual_position != last_position){
      shutoff_timer = millis();
    }
    
    if (shutoff_narrow){
      shutoff_wide = false;
      shutoff_narrow = false;
    }
    
    if (millis() - shutoff_timer > shutoff_time){
      shutoff_narrow = false;
      shutoff_wide = true;
    }
  }
  else {
    // Setpoint reached
    digitalWrite(OUTPUT_WIDE, LOW);
    digitalWrite(OUTPUT_NARROW, LOW);
    digitalWrite(OUTPUT_BYPASS, LOW);
    digitalWrite(OUTPUT_LED, LOW);
    
    // Reset shutoff timer
    shutoff_timer = millis();
  }
  last_position = _actual_position;
}

// ------------------------------
// Method for stopping implement
// ------------------------------
void ImplementPlough::stop(){
  digitalWrite(OUTPUT_WIDE, LOW);
  digitalWrite(OUTPUT_NARROW, LOW);
  digitalWrite(OUTPUT_BYPASS, LOW);
  digitalWrite(OUTPUT_LED, LOW);
}

// --------------------------------------------
// Method for measuring actual implement offset
// --------------------------------------------
int ImplementPlough::getActualPosition(){
  // Read analog input
  int _read_raw = analogRead(POSITION_SENS_PIN);
  float _actual_position;
  int i = 0;

  // Loop through calibrationdata
  while (_read_raw > position_calibration_data[i] && i < 2){
    i++;
  }

  if (i == 0){
    i++;
  }

  // Interpolate calibrationdata
  float a = _read_raw - position_calibration_data[i-1];
  float b = position_calibration_data[i] - position_calibration_data[i-1];
  float c = position_calibration_points[i] - position_calibration_points[i-1];
  float d = position_calibration_points[i-1];

  // Calculate actual implement offset
  _actual_position = (((a * c) / b) + d);

  return _actual_position * shares;
}

//------------------------------------------
// Method for measuring actual implement XTE
//------------------------------------------
int ImplementPlough::getActualRotation(){
  // Read analog input
  int _read_raw = analogRead(ROTATION_SENS_PIN);
  int _actual_rotation;
  int i = 0;

  // Loop through calibrationdata
  while (_read_raw > rotation_calibration_data[i] && i < 1){
    i++;
  }

  if (i == 0){
    i++;
  }

  // Interpolate calibrationdata
  float a = _read_raw - rotation_calibration_data[i-1];
  float b = rotation_calibration_data[i] - rotation_calibration_data[i-1];
  float c = rotation_calibration_points[i] - rotation_calibration_points[i-1];
  float d = rotation_calibration_points[i-1];

  // Calculate actual implement offset
  _actual_rotation = (((a * c) / b) + d);

  return _actual_rotation;
}

// ---------------------------
// Method for setting setpoint
// ---------------------------
void ImplementPlough::setSetpoint(){
  // calculate proportional gain (KP should be 1)
  P = xte * (float(KP) / 100); 

  // maximise correction to set maximum
  if (P <= - max_correction){
    P = - max_correction;
  }
  else if (P >= max_correction){
    P = max_correction;
  }
  
  // calculate setpoint
  setpoint = offset + ((getSide() * 2) - 1) * P;  
}

// ---------------------------------------------------------
// Method for setting implement offset and writing to EEPROM
// ---------------------------------------------------------
void ImplementPlough::setOffset(int _correction){
  if (_correction){
    offset += _correction;
    if (offset > shares * 60 || offset < shares * 20){
      offset = shares * 40;
    }

    // Write each byte separately to the memory
    EEPROM.write(180, highByte(offset));
    EEPROM.write(181, lowByte(offset));
  }
}

// -----------------------------------------------
// Method for reading implement offset from EEPROM
// -----------------------------------------------
void ImplementPlough::readOffset(){
  if (EEPROM.read(180) < 255){
    // Read offset (2 bytes)
    offset = word(EEPROM.read(180), EEPROM.read(181));
    if (offset > shares * 60 || offset < shares * 20){
      offset = shares * 40;
    }
  }
  else{
    offset = shares * 40;
  }
}

// ----------------------------------------------
// Method for reading calibrationdata from EEPROM
// ----------------------------------------------
boolean ImplementPlough::readCalibrationData(){
  // Read amount of startups and add 1
  EEPROM.write(0, EEPROM.read(0));
  
  // Read offset and XTE calibration data
  if (EEPROM.read(100) != 255 || EEPROM.read(110) != 255 ||
    EEPROM.read(150) != 255 || EEPROM.read(151) != 255 ||
    EEPROM.read(160) != 255 || EEPROM.read(161) != 255 ||
    EEPROM.read(162) != 255 || EEPROM.read(163) != 255 ||
    EEPROM.read(170) != 255) {

    // Read from eeprom highbyte, then lowbyte, and combine into words
    for(int i = 0; i < 3; i++){
      int k = 2 * i;
      // 100 - 101 and 110 - 111
      position_calibration_data[i] = word(EEPROM.read(k+100), EEPROM.read(k+101));
      rotation_calibration_data[i] = word(EEPROM.read(k+110), EEPROM.read(k+111));
    }
    
#ifdef PID_KP
    KP = EEPROM.read(170);
#else
    KP = 100;
#endif

#ifdef PWM_MAN
    man_pwm = EEPROM.read(150);
#else
    man_pwm = 254;
#endif

#ifdef PWM_AUTO
    auto_pwm = EEPROM.read(151);
#else
    auto_pwm = 254;
#endif

    //Read error max == 10 cm
    if(EEPROM.read(160) < 10) {
      // Read error
      error = EEPROM.read(160);
    }
    else {
      error = 2;  //default to 2
    }

    //Read swap 1 or 0
    if(EEPROM.read(161) < 2) {
      // Read swap
      swap = EEPROM.read(161);
    }
    else {
      swap = false;  //default to 0
    }
    
    //Read number of shares
    if(EEPROM.read(162) < 10) {
      // Read number of shares
      shares = EEPROM.read(162);
    }
    else {
      shares = 4;  //default to 4
    }
    
    //Read maximum correction
    if(EEPROM.read(163) < 10) {
      // Read maximum correction
      max_correction = EEPROM.read(163);
    }
    else {
      max_correction = 50;  //default to 4
    }
  }
  else{
    return false;
  }
  return true;
}

//---------------------------------------------------
//Method for printing calibration data to serial port
//---------------------------------------------------
void ImplementPlough::printCalibrationData(){
  // Print amount of times started
  Serial.println("Times started");
  Serial.println(EEPROM.read(0));
  Serial.println("--------------------------");
  
  // Printing calibration data to serial port
  Serial.println("Using following data:");
  Serial.println("--------------------------");
  Serial.println("Offset calibration data");
  for (int i = 0; i < 3; i++){
    Serial.print(position_calibration_data[i]);
    Serial.print(", ");
    Serial.println(position_calibration_points[i]);
  }
  Serial.println("--------------------------");

  Serial.println("Rotation calibration data");
  for (int i = 0; i < 3; i++){
    Serial.print(rotation_calibration_data[i]);
    Serial.print(", ");
    Serial.println(rotation_calibration_points[i]);
  }
  Serial.println("--------------------------");

  Serial.println("Number of shares");
  Serial.println(shares);
  Serial.println("--------------------------");

#ifdef PID_KP  
  Serial.println("KP");
  Serial.println(KP);
  Serial.println("--------------------------");
#endif

#ifdef PWM_AUTO
  Serial.println("PWM auto");
  Serial.println(auto_pwm);
  Serial.println("--------------------------");
#endif

#ifdef PWM_MAN
  Serial.println("PWM manual");
  Serial.println(man_pwm);
  Serial.println("--------------------------");
#endif

  Serial.println("Error margin");
  Serial.println(error);
  Serial.println("--------------------------");
  
  Serial.println("Swap ploughside");
  Serial.println(swap);
  Serial.println("--------------------------");
  
  Serial.println("Maximum correction");
  Serial.println(max_correction);
  Serial.println("--------------------------");
}

// --------------------------------------------
// Method for writing calibrationdata to EEPROM
// --------------------------------------------
void ImplementPlough::writeCalibrationData(){
  // Write each byte separately to the memory first the data then the points
  for(int i = 0; i < 3; i++){
    int k = 2 * i;
    EEPROM.write(k+100, highByte(position_calibration_data[i])); // 100, 102, 104
    EEPROM.write(k+101, lowByte(position_calibration_data[i]));  // 101, 103, 105
    EEPROM.write(k+110, highByte(rotation_calibration_data[i])); // 110, 112, 114
    EEPROM.write(k+111, lowByte(rotation_calibration_data[i]));  // 111, 113, 115
  }

  EEPROM.write(150, man_pwm);  //150
  EEPROM.write(151, auto_pwm); //151

  EEPROM.write(160, error);    //160
  EEPROM.write(161, swap);     //161
  EEPROM.write(162, shares);   //162
  EEPROM.write(163, max_correction);   //163

  EEPROM.write(170, KP); // 120
#ifdef DEBUG
  Serial.println("Calibration data written");
#endif  
}

// ---------------------------------------------
// Method for wiping calibrationdata from EEPROM
// ---------------------------------------------
void ImplementPlough::wipeCalibrationData(){
  // Wipe calibration data

    // Write 255 into all memory registers
  for  (int i = 1; i < 255; i++){
    EEPROM.write(i, 255);
  }

#ifdef DEBUG
  Serial.println("Calibration data wiped");
#endif
}
