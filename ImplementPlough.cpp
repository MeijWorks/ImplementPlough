/*
  ImplementPlanter - a libary for a planter
 Copyright (C) 2011-2014 J.A. Woltjer.
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

#include <ImplementPlanter.h>

//------------
// Constructor
//------------
ImplementPlough::ImplementPlough(){
  // Pin configuration
  // Inputs
  pinMode(PLANTINGELEMENT_PIN, INPUT);
  
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
    maxcor = 50;
    
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

  // PID variables
  P = 0;

  // End shutoff timers
  shutoff_time = SHUTOFF;
  shutoff_dir = 0;
  shutoff_timer = millis();

  // Update timer
  update_age = millis();
  
  // Swap
  swap = false;
  
  // Print calibration data
#ifdef DEBUG
  printCalibrationData();
#endif
}

// --------------------------------------------
// Method for updating implement data using gps
// --------------------------------------------
void ImplementPlough::update(int _correction, boolean _reset, int _xte){
  // Update offset, xte, rotation and setpoint
  if (millis() - update_age >= 200){
    // Write EEPROM if nescessary
    if (_correction != 0){
      setOffset(_correction);
    }
    // Update xte, rotation and setpoint
    xte = _xte;
    position = getActualPosition();
    rotation = getActualRotation();

    setSetpoint();
    update_age = millis();    
  }
}

// ------------------------------
// Method for adjusting implement
// ------------------------------
void ImplementPlough::adjust(boolean _auto, int _direction){
  int _actual_position;
  byte _pwm;

  if (_auto){
    _actual_position = getActualPosition() + offset;
    _pwm = auto_pwm;
  }
  else {
    _actual_position = setpoint - (_direction * (error + 1) * 2);
    _pwm = man_pwm;
  }

  // Adjust tree including error
  if (_actual_position < setpoint - error){
    if (shutoff_dir > 0 || millis() - shutoff_timer < shutoff_time){
      digitalWrite(OUTPUT_WIDE, HIGH);
      digitalWrite(OUTPUT_NARROW, LOW);
      analogWrite(OUTPUT_BYPASS, _pwm);
      digitalWrite(OUTPUT_LED, HIGH);
      if (last_position != _actual_position){
        shutoff_timer = millis();
        shutoff_dir = 1;
      }
    }
    else {
      digitalWrite(OUTPUT_WIDE, LOW);
      digitalWrite(OUTPUT_NARROW, LOW);
      analogWrite(OUTPUT_BYPASS, 0);
      digitalWrite(OUTPUT_LED, LOW);      
    }
  }
  else if (_actual_position > setpoint + error){
    if (shutoff_dir < 0 || millis() - shutoff_timer < shutoff_time){
      digitalWrite(OUTPUT_WIDE, LOW);
      digitalWrite(OUTPUT_NARROW, HIGH);
      analogWrite(OUTPUT_BYPASS, _pwm);
      digitalWrite(OUTPUT_LED, HIGH);
      if (last_position != _actual_position){
        shutoff_timer = millis();
        shutoff_dir = -1;
      }
    }
    else{
      digitalWrite(OUTPUT_WIDE, LOW);
      digitalWrite(OUTPUT_NARROW, LOW);
      analogWrite(OUTPUT_BYPASS, 0);
      digitalWrite(OUTPUT_LED, LOW);
    }
  }
  else {
    digitalWrite(OUTPUT_WIDE, LOW);
    digitalWrite(OUTPUT_NARROW, LOW);
    analogWrite(OUTPUT_BYPASS, 0);
    digitalWrite(OUTPUT_LED, LOW);
    shutoff_timer = millis();
  }
  last_position = _actual_position;
}

// --------------------------------------------
// Method for measuring actual implement offset
// --------------------------------------------
int ImplementPlough::getActualPosition(){
  // Read analog input
  int _read_raw = analogRead(POSITION_SENS_PIN);
  int _actual_position;
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

  return _actual_position;
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
  while (_read_raw > rotation_calibration_data[i] && i < 2){
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
void ImplementPlough::setSetpoint(boolean _reset){
  // calculate proportional gain (KP should be 1)
  P = float(xte * KP) / 100; 

  // maximise correction to set maximum
  if (P <= - max_correction){
    P = - max_correction;
  }
  else if (P >= max_correction){
    P = max_correction;
  }
  
  // calculate setpoint
  setpoint = offset + P;   
}

// ---------------------------------------------------------
// Method for setting implement offset and writing to EEPROM
// ---------------------------------------------------------
void ImplementPlough::setOffset(int _correction){
  offset += _correction;
  if (offset > shares * 60 || offset < shares * 20){
    offset = shares * 40;
  }

  // Write each byte separately to the memory
  EEPROM.write(180, highByte(offset));
  EEPROM.write(181, lowByte(offset));
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
#ifdef PID_KP
    EEPROM.read(120) != 255 ||
#endif
#ifdef PWM_MAN
    EEPROM.read(150) != 255 ||
#endif
#ifdef PWM_AUTO
    EEPROM.read(160) != 255 ||
#endif
    EEPROM.read(170) != 255 || EEPROM.read(190) != 255 ||
    EEPROM.read(200) != 255 || EEPROM.read(210) != 255){

    // Read from eeprom highbyte, then lowbyte, and combine into words
    for(int i = 0; i < 3; i++){
      int k = 2 * i;
      // 100 - 101 and 110 - 111
      position_calibration_data[i] = word(EEPROM.read(k+100), EEPROM.read(k+101));
      rotation_calibration_data[i] = word(EEPROM.read(k+110), EEPROM.read(k+111));
    }
    
#ifdef PID_KP
    KP = float(word(EEPROM.read(120), EEPROM.read(121)) / 100);
#else
    KP = 100;
#endif

#ifdef PWM_MAN
    man_pwm = EEPROM.read(150);
#else
    man_pwm = 255;
#endif

#ifdef PWM_AUTO
    auto_pwm = EEPROM.read(160);
#else
    auto_pwm = 255;
#endif

    //Read error max == 10 cm
    if(EEPROM.read(170) < 10) {
      // Read error
      error = EEPROM.read(170);
    }
    else {
      error = 2;  //default to 2
    }

    //Read swap 1 or 0
    if(EEPROM.read(190) < 2) {
      // Read swap
      swap = EEPROM.read(190);
    }
    else {
      swap = false;  //default to 0
    }
    
    //Read number of shares
    if(EEPROM.read(200) < 10) {
      // Read number of shares
      shares = EEPROM.read(200);
    }
    else {
      shares = 4;  //default to 4
    }
    
    //Read maximum correction
    if(EEPROM.read(210) < 10) {
      // Read maximum correction
      maxcor = EEPROM.read(210);
    }
    else {
      maxcor = 50;  //default to 4
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
  Serial.println(maxcor);
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

#ifdef PID_KP
  EEPROM.write(120, highByte(int(KP*100))); // 120
  EEPROM.write(121, lowByte(int(KP*100)));  // 121
#endif

#ifdef PWM_MAN
  EEPROM.write(150, man_pwm);  //150
#endif
#ifdef PWM_MAN
  EEPROM.write(160, auto_pwm); //160
#endif
  EEPROM.write(170, error);    //170

  EEPROM.write(190, swap);     //190
  EEPROM.write(200, shares);   //200
  EEPROM.write(210, maxcor);   //210
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
