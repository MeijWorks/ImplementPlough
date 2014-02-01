/*
  ImplementPlanter - a library for a planter
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

#ifndef ImplementPlough_h
#define ImplementPlough_h

#include <Arduino.h>
#include <EEPROM.h>
#include <ConfigImplement.h>

// software version of this library
#define PLOUGH_VERSION 0.1

class ImplementPlanter {
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

  // Variables concerning adjust loop
  int setpoint;
  int offset;
  byte error;
#ifdef PWM_MAN
  byte man_pwm;
#endif  
#ifdef PWM_AUTO
  byte auto_pwm;
#endif

  // PID variables
  float P;
#ifdef PID_KP
  int KP;
#endif
  
  // Timers for end shutoff
  int shutoff_time;
  int shutoff_dir;
  unsigned long shutoff_timer;
  
  // Variables for adjusting ploughside
  boolean swap;
  byte shares;
  byte maxcor;

  //-------------------------------------------------------------
  // private member functions implemented in ImplementPlanter.cpp
  //-------------------------------------------------------------
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
  // -----------------------------------------------------------
  // public member functions implemented in ImplementPlanter.cpp
  // -----------------------------------------------------------

  // Constructor
  ImplementPlough();

  void update(int _correction, boolean _reset, int _xte);
  void adjust(boolean _auto, int _direction);
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
  inline boolean getSide(){
    return digitalRead(PLOUGHSIDE_PIN);
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
  
  inline int GetMaxCorrection(){
    return maxcor;
  }
  
  inline int getKP(){
    return KP;
  }

  inline byte getPwmMan(){
    return man_pwm;
  }

  inline byte getPwmAuto(){
    return auto_pwm;
  }
  
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
  
  inline void setKP(int _value){
    KP = _value;
  }

  inline void setPwmMan(byte _value){
    man_pwm = _value;
  }

  inline void setPwmAuto(byte _value){
    auto_pwm = _value;
  }

  inline void SetMaxCorrection(int _value){
    maxcor = _value;
  }

  inline void setError(byte _value){
    error = _value;
  }
  
  inline void setSwap(boolean _value){
    swap = _value;
  }
};
#endif


