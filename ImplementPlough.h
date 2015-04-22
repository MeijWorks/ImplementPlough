/*
  ImplementPlanter - a library for a planter
 Copyright (C) 2011-2015 J.A. Woltjer
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

#ifndef ImplementPlough_h
#define ImplementPlough_h

#include <Arduino.h>
#include <EEPROM.h>
#include "ConfigImplementPlough.h"
#include "VehicleGps.h"

// software version of this library
#define PLOUGH_VERSION 0.11

class ImplementPlough {
private:
  //-------------
  // data members
  //-------------
  
  // Default offset calibration set
  int position_calibration_data[3];
  int position_calibration_points[3];
  int position;
  int last_position;

  // Default rotation calibration set
  // enigineered for lemken ploughs with rotation sensor
  int rotation_calibration_data[3];
  int rotation_calibration_points[3];
  int rotation;

  // Update timer
  unsigned long update_age;
  boolean update_flag;

  // Variables concerning adjust loop
  byte mode;
  int setpoint;
  int offset;
  byte man_pwm;
  byte auto_pwm;

  // XTE, error and maximum correction
  int xte;
  byte error;
  byte max_correction;


  // Speed indication for shutoff
  int speed;

  // PID variables
  float P;
  
  byte KP;
  
  // Timers for end shutoff
  int shutoff_time;
  bool shutoff_wide;
  bool shutoff_narrow;
  unsigned long shutoff_timer;
  
  // Variables for adjusting ploughside and amount of shares
  boolean swap;
  byte shares;

  // Objects
  VehicleGps * gps;
  
  //------------------------------------------------------------
  // private member functions implemented in ImplementPlough.cpp
  //------------------------------------------------------------
  int getActualRotation();
  int getActualPosition();

  void setSetpoint();
  void setOffset(int _correction);

  void readOffset();
  
  boolean readCalibrationData();
  void printCalibrationData();
  void writeCalibrationData();
  void wipeCalibrationData();

public:
  // ----------------------------------------------------------
  // public member functions implemented in ImplementPlough.cpp
  // ----------------------------------------------------------

  // Constructor
  ImplementPlough(VehicleGps * _gps);

  void update(byte _mode, int _buttons);
  void stop();
  void adjust(int _direction);
  void calibrate();

  // ----------------------------------------------------------------
  // public inline member functions implemented in ImplementPlanter.h
  // ----------------------------------------------------------------
  inline boolean resetCalibration(){
    return readCalibrationData();
  }
  
  inline void commitCalibration(){
    wipeCalibrationData();
    writeCalibrationData();
  }
  
  // -------
  // Getters
  // -------
  inline bool getSide(){
    return digitalRead(PLOUGHSIDE_PIN) ^ swap;
  }
  
  inline int getPosition(){
    return position;
  }
  
  inline int getRotation(){
    return rotation;
  }
  
  inline int getSetpoint(){
    return setpoint;
  }
  
  inline int getOffset(){
    return offset;
  }
  
  inline int getPositionCalibrationPoint(int _i){
    return position_calibration_points[_i];
  }

  inline int getRotationCalibrationPoint(int _i){
    return rotation_calibration_points[_i];
  }

  inline byte getShares(){
    return shares;
  }
  
  inline int getMaxCorrection(){
    return max_correction;
  }

#ifdef PID_KP  
  inline int getKP(){
    return KP;
  }
#endif

#ifdef PWM_MAN
  inline byte getPwmMan(){
    return man_pwm;
  }
#endif

#ifdef PWM_MAN
  inline byte getPwmAuto(){
    return auto_pwm;
  }
#endif
  
  inline byte getError(){
    return error;
  }

  // -------
  // Setters
  // -------  
  inline void setPositionCalibrationData(int _i){
    position_calibration_data[_i] = analogRead(POSITION_SENS_PIN);
  }

  inline void setRotationCalibrationData(int _i){
    rotation_calibration_data[_i] = analogRead(ROTATION_SENS_PIN);
  }

  inline void setShares(byte _value){
    shares = _value;
  }

#ifdef PID_KP  
  inline void setKP(int _value){
    KP = _value;
  }
#endif

#ifdef PWM_MAN
  inline void setPwmMan(byte _value){
    man_pwm = _value;
  }
#endif

#ifdef PWM_AUTO
  inline void setPwmAuto(byte _value){
    auto_pwm = _value;
  }
#endif

  inline void setMaxCorrection(int _value){
    max_correction = _value;
  }

  inline void setError(byte _value){
    error = _value;
  }
  
  inline void setSwap(boolean _value){
    swap = _value;
  }
};
#endif


