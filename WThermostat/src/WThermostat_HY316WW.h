#ifndef THERMOSTAT_HY316WW_H
#define	THERMOSTAT_HY316WW_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "WThermostat.h"
#include "WThermostat_BAC_002_ALW.h"
#include "WThermostat_ME102H.h"

class WThermostat_HY316WW : public WThermostat {
public :
  WThermostat_HY316WW(WNetwork* network, WProperty* thermostatModel, WClock* wClock)
    : WThermostat(network, thermostatModel, wClock) {
    network->debug(F("WThermostat_HY316WW created"));
  }

  virtual void configureCommandBytes() {
    this->byteDeviceOn = 0x01;
    this->byteTemperatureActual = 0x03;
    this->byteTemperatureTarget = 0x02;
    this->byteTemperatureFloor = NOT_SUPPORTED;
    this->temperatureFactor = 10.0f;
    this->byteSchedulesMode = 0x04;
    this->byteLocked = 0x06;
    this->byteSchedules = 0x77;
    this->byteSchedulingPosHour = 0;
    this->byteSchedulingPosMinute = 1;
    this->byteSchedulingDays = 12;
    //custom parameters
    this->byteStatusMode = 0x66;
    this->byteSensorSelection = 0x74;
  }

  virtual void initializeProperties() {
    WThermostat::initializeProperties();
    //schedulesMode
    this->schedulesMode->clearEnums();
    this->schedulesMode->addEnumString(SCHEDULES_MODE_OFF);
    this->schedulesMode->addEnumString(SCHEDULES_MODE_AUTO);
    this->schedulesMode->addEnumString(SCHEDULES_MODE_HOLD);
    //statusMode
    this->statusMode = new WProperty("statusMode", "Status", STRING, TYPE_HEATING_COOLING_PROPERTY);
    this->statusMode->addEnumString(STATE_OFF);
    this->statusMode->addEnumString(STATE_HEATING);
    this->statusMode->setReadOnly(true);
    this->statusMode->setVisibility(MQTT);
    this->addProperty(statusMode);
    //sensorSelection
    this->sensorSelection = new WProperty("sensorSelection", "Sensor Selection", STRING, TYPE_THERMOSTAT_MODE_PROPERTY);
    this->sensorSelection->addEnumString(SENSOR_SELECTION_INTERNAL);
    this->sensorSelection->addEnumString(SENSOR_SELECTION_FLOOR);
    this->sensorSelection->addEnumString(SENSOR_SELECTION_BOTH);
    this->sensorSelection->setVisibility(MQTT);
    this->sensorSelection->setOnChange(std::bind(&WThermostat_HY316WW::sensorSelectionToMcu, this, std::placeholders::_1));
    this->addProperty(this->sensorSelection);
  }

protected :

  virtual bool processStatusCommand(byte cByte, byte commandLength) {
		//Status report from MCU
		bool changed = false;
		bool knownCommand = WThermostat::processStatusCommand(cByte, commandLength);

		if (!knownCommand) {
      const char* newS;
      if (cByte == byteStatusMode) {
				if (commandLength == 0x05) {
				  //status
				  newS = statusMode->getEnumString(receivedCommand[10]);
				  if (newS != nullptr) {
					  changed = ((changed) || (statusMode->setString(newS)));
					  knownCommand = true;
				  }
				}
			} else if (cByte == this->byteSensorSelection) {
        if (commandLength == 0x05) {
          //sensor selection -
          //internal: 55 aa 03 07 00 05 2b 04 00 01 00
          //floor:    55 aa 03 07 00 05 2b 04 00 01 01
          //both:     55 aa 03 07 00 05 2b 04 00 01 02
          newS = this->sensorSelection->getEnumString(receivedCommand[10]);
          if (newS != nullptr) {
            changed = ((changed) || (this->sensorSelection->setString(newS)));
            knownCommand = true;
          }
        }
      } else {
      //consume some unsupported commands
        switch (cByte) {
          case 0x6d :
            //A1 Measure Temperature Calibration
            knownCommand = true;
            break;
          case 0x6e :
            //A2 Temperature control return diference
            knownCommand = true;
            break;
          case 0x6f :
            //A3 External sensors limit temperature control return diference
            knownCommand = true;
            break;
          case 0x6a :
            //A6 Limit value of high temperature On/off
            knownCommand = true;
            break;
          case 0x70 :
            //A6 Value
            knownCommand = true;
            break;
          case 0x6b :
            //A7 Limit value of low temperature On/Off
            knownCommand = true;
            break;
          case 0x71 :
            //A7 Value
            knownCommand = true;
            break;
          case 0x72 :
            //A9 Setting temperature highest limit
            knownCommand = true;
            break;
          case 0x73 :
            //A8 Setting temperature lowest limit
            knownCommand = true;
            break;
          case 0x74 :
            //A4 Options of sensor control
            //0 = N1 Built-in sensor
            //1 = N2 External sensor
            //2 = N3 Built-in sensor control temperature
            knownCommand = true;
            break;
          case 0x75 :
            //AB Power with memory function
            //0 = Power with memory function
            //1 = Shutdown power after power off
            //2 = Shutdown power after power on
            knownCommand = true;
            break;
          case 0x76 :
            //AC Weekly programming selection
            //0 = 5+2
            //1 = 6+1
            //2 = 7
            knownCommand = true;
            break;
        }
      }
    }
		if (changed) {
			notifyState();
		}
	  return knownCommand;
  }

  virtual bool processStatusSchedules(byte commandLength) {
    bool result = (commandLength == 0x0D);
    if (result) {
      bool changed = false;
      //schedules for model HY316WW
      int res = 1;
      int ii = 0;
      for (int i = 0; i < 32; i++) {
      byte newByte = receivedCommand[i + 10];
      if (i != 2) {
        if (i > 2)
          res = (i+2) % 4;
          if (res != 0) {
            changed = ((changed) || (newByte != schedules[ii]));
            schedules[ii] = newByte;
            ii++;
          }
        }
      }
      if (changed) {
        notifySchedules();
      }
    }
    return result;
  }

  virtual void schedulesToMcu() {
    if (receivedSchedules()) {
			int daysToSend = this->byteSchedulingDays;
			int functionLengthInt = (daysToSend * 3);
			char functionL = 0x09;
			char dataL = 0x0D;
			unsigned char scheduleCommand[functionLengthInt+10];
			scheduleCommand[0] = 0x55;
			scheduleCommand[1] = 0xaa;
			scheduleCommand[2] = 0x03;
			scheduleCommand[3] = 0x07;
			scheduleCommand[4] = 0x00;
			scheduleCommand[5] = dataL; //0x3a; // dataLength
			scheduleCommand[6] = byteSchedules;
			scheduleCommand[7] = 0x00;
			scheduleCommand[8] = 0x00;
			scheduleCommand[9] = functionL;

			int res = 1;
			functionLengthInt = functionLengthInt + 8;
			int ii = 0;
			for (int i = 0; i <functionLengthInt; i++) {
				if (i == 2) {
					scheduleCommand[i + 10] = 0x00;
				} else if (i > 2) {
					res = (i+2) % 4;
					if (res != 0) {
						scheduleCommand[i + 10] = schedules[ii];
						ii++;
					} else {
						scheduleCommand[i + 10] = 0x00;
					}
				} else {
					scheduleCommand[i + 10] = schedules[ii];
					ii++;
        }
    	}

			commandCharsToSerial(functionLengthInt+10, scheduleCommand);
			//notify change
			this->notifySchedules();
		}
  }

  void sensorSelectionToMcu(WProperty* property) {
    if (!isReceivingDataFromMcu()) {
      byte sm = property->getEnumIndex();
      if (sm != 0xFF) {
        //send to device
        //internal: 55 aa 03 07 00 05 2d 05 00 01 00
        //floor:    55 aa 03 07 00 05 2d 05 00 01 01
        //both:     55 aa 03 07 00 05 2d 05 00 01 02
        unsigned char cm[] = { 0x55, 0xAA, 0x00, 0x06, 0x00, 0x05,
                               this->byteSensorSelection, 0x05, 0x00, 0x01, sm};
        commandCharsToSerial(11, cm);
      }
    }
  }

private :
  WProperty* statusMode;
  byte byteStatusMode;
  WProperty* sensorSelection;
  byte byteSensorSelection;

};

#endif