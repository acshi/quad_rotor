#include <Metro.h>
#include <Streaming.h>
#include <Wire.h>

enum MSG_CODES {
    SET_DUTY_MSG = 1,
    READ_DUTY_MSG,
    READ_MEASURED_HZ_MSG,
    SET_CURRENT_LIMIT_MSG,
    READ_CURRENT_LIMIT_MSG,
    SET_START_DUTY_MSG,
    READ_START_DUTY_MSG,
    READ_MEASURED_CURRENT_MSG,
    READ_MEASURED_VOLTAGE_MSG,
    READ_TEMPERATURE_MSG,
    READ_ERROR_MSG,
    DIAGNOSTIC_MSG,
    ADDRESS_MSG,
    PHASE_STATE_MSG,
};

#define MSG_END_BYTE 0xed
#define VAL_END_BYTE 0xec

#define MSG_MARK_BYTE 0xfe

uint8_t selectedAddress = 255; // none selected yet

bool availableAddrs[128];

int motor_on = 0;

bool flushTwi(int address) {
  //Serial << "flush" << endl;
  
  bool hasDiscarded = false;
  // Clear out buffers on both ends from any nonsense...
  while (true) {
    while (Wire.available()) {
      int b = Wire.read();
      if (b != 0xff) {
        Serial << address << ":1: " << b << endl;
        hasDiscarded = true;
      }
    }
    Wire.requestFrom(address, 3);
    for (uint8_t i = 0; i < 3; i++) {
      if (!Wire.available()) {
        //Serial << "flush out 1\n";
        return hasDiscarded;
      }
      int b = Wire.read();
      if (b == 0xff) {
        //Serial << "flush out 2\n";
        return hasDiscarded;
      }
      Serial << address << ":" << (i + 2) << ": " << b << endl;
      hasDiscarded = true;
    }
  }
}

uint16_t sendReadMessage(uint8_t command, int address) {
  if (address == 255) {
    return 0xffff;
  }

  flushTwi(address);

  //Serial << "read transmission: " << command << endl;
  
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(command);
  Wire.write(MSG_END_BYTE);
  // not sure why we need false, but without it the receiver doesn't
  // seem to ever get a read-start condition from the requestFrom below
  Wire.endTransmission(false);

  // Look for the message mark to know the message has been received.
  Wire.requestFrom(address, 3);
  
  int16_t b = Wire.read();
  if (b == MSG_MARK_BYTE) {
    return (uint16_t)(Wire.read() << 8) | Wire.read();
  } else if (b != -1) {
    Serial << "Got " << b << " instead\n";
  }

  return 0xffff;
}

void singleSendMessage(uint8_t command, int16_t value, int address) {  
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(command);
  Wire.write(MSG_END_BYTE);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff);
  Wire.write(VAL_END_BYTE);
  Wire.endTransmission();
}

void sendMessageValue(uint8_t command, int16_t value, int address) {
  if (address == 255) {
    Serial << "Cannot send a message without first selecting a TWI device\n";
    return;
  }

  // send several times because noise may interfere
  singleSendMessage(command, value, address);
  singleSendMessage(command, value, address);
  singleSendMessage(command, value, address);
}

void scanAddrs() {
  bool foundAny = false;
  for (uint8_t i = 8; i < 24; i++) {
    uint16_t reply = sendReadMessage(DIAGNOSTIC_MSG, i);
    bool foundAddr = reply == 'HI';
    availableAddrs[i] = foundAddr;
    if (foundAddr) {
      foundAny = true;
    } else if (reply != 65535) {
      Serial << "Wrong reply to diagnostic: " << reply << endl;
    }
  }
  if (foundAny) {
    Serial << "Available TWI devices:\n";
    for (uint8_t i = 8; i < 24; i++) {
      if (availableAddrs[i]) {
        Serial << '\t' << i;
        if (selectedAddress == 255) {
          selectedAddress = i;
        }
        if (selectedAddress == i) {
          Serial << " (selected)";
        }
        Serial << endl;
      }
    }
    Serial << endl;
  } else {
    Serial << "No TWI devices found. Check your wiring and scan again with 'sa'\n";
  }
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  // slow clock from 100kHz to 40kHz
  // hopefully help with noise
  Wire.setClock(40000); 
  Serial.begin(115200);

  Serial << "Ready!\n";
  scanAddrs();
  Serial << "Done scanning\n";
}

double measureVoltage(int address) {
  uint16_t value = sendReadMessage(READ_MEASURED_VOLTAGE_MSG, address);
  // return value * 6 * 2.23 / 8192 - 0.01; // old version for ATSAMD
  // voltage is divided by 6, put into * 8.0 fixed point then read as 10-bit out of 5 volts
  // also a correction factor at the end
  return value * 6.0 / 8.0 * 5.0 / 1023 * 0.956;
}

double measureCurrent(int address) {
  uint16_t value = sendReadMessage(READ_MEASURED_CURRENT_MSG, address);
  //double current = value / 395.0 / 2.0; // old version for ATSAMD
  // 0.1V/A, *16.0 fixed point then read as 10-bit out of 5 volts
  // also a correction factor at the end
  double current = value * 5.0 / 1023 / 0.1 / 16.0 * 0.856;
//  if (current < 0.018) {
//    current = 0.0;
//  }
  return current;
}

bool performAction(int16_t action, int address) {
    uint16_t value;
    switch (action) {
      case 'hl':
      case 'hp':
        Serial.print(F("Commands include...\n"
          "hl, hp, to show this help\n"
          "sd, gd, (set/get) duty cycle target\n"
          "mh, measured hz target\n"
          "sc, gc, mc, (set/get/measured) current target\n"
          "mv, measured motor voltage\n"
//          "mt, measured temperature\n"
          "er, check for error conditions\n"
          "tw, return TWI address of device\n"
          "sa, scan addresses for connected TWI devices\n"
          "se, set device to communicate with from those scanned\n"
          "fl, flush communcations buffers with a specific device\n"
        ));
        break;
      case 'sd':
        value = (uint16_t)Serial.parseInt();
        Serial << F("Setting duty cycle target to: ") << (int16_t)value << endl;
        sendMessageValue(SET_DUTY_MSG, value, address);
        break;
      case 'gd':
        value = sendReadMessage(READ_DUTY_MSG, address);
        Serial << F("Duty cycle target: ") << (int16_t)value << endl;
        break;
      case 'mh':
        value = sendReadMessage(READ_MEASURED_HZ_MSG, address);
        Serial << F("Hz measurement: ") << value / 6.0 / 7.0 * 2.0 << endl;
        break;
      case 'sc':
        value = (int16_t)Serial.parseInt();
        Serial << F("Setting current limit to: ") << value << " (" << (value / 395.0) << "A)\n";
        sendMessageValue(SET_CURRENT_LIMIT_MSG, value, address);
        break;
      case 'gc':
        value = sendReadMessage(READ_CURRENT_LIMIT_MSG, address);
        Serial << F("Current limit: ") << value << " (" << (value / 395.0) << "A)\n";
        break;
      case 'ss':
        value = (int16_t)Serial.parseInt();
        Serial << F("Setting start duty cycle to: ") << value << "\n";
        sendMessageValue(SET_START_DUTY_MSG, value, address);
        break;
      case 'gs':
        value = sendReadMessage(READ_START_DUTY_MSG, address);
        Serial << "Start duty cycle: " << value << "\n";
        break;
      case 'mc':
        Serial << F("Current measurement: ") << measureCurrent(address) << "A\n";
        break;
      case 'mv':
        Serial << F("Motor voltage: ") << measureVoltage(address) << "V\n";
        break;
//      case 'mt':
//        value = sendReadMessage(READ_TEMPERATURE_MSG, address);
//        Serial << F("Temperature (celsius): ") << *(int16_t*)(&value) / 10.0 << endl;
//        break;
      case 'er':
        value = sendReadMessage(READ_ERROR_MSG, address);
        Serial << F("Error: ");
        switch (value) {
          case 0:
            Serial << F("No error");
            break;
          case 1:
            Serial << F("Overcurrent shut-off");
            break;
          case 2:
            Serial << F("VDS fault");
            break;
          default:
            Serial << F("invalid error/communication problem");
            break;
        }
        Serial << endl;
        break;
      case 'tw':
        value = sendReadMessage(ADDRESS_MSG, address);
        Serial << F("TWI Address: ") << value << endl;
        break;
      case 'sa':
        scanAddrs();
        break;
      case 'se':
        value = Serial.parseInt();
        if (value < 8 || value > 127) {
          Serial << F("Address ") << value << F(" is invalid. TWI addresses may range from 8 to 127.\n");
          break;
        }
        if (true || availableAddrs[value]) {
          selectedAddress = value;
          Serial << F("Selected address ") << selectedAddress << endl;
        } else {
          Serial << F("Address ") << value << F(" is not available. Check your wiring and then rescan with 'sa'\n");
        }
        break;
      case 'fl':
        value = Serial.parseInt();
        if (value < 8 || value > 127) {
          Serial << F("Address ") << value << F(" is invalid. TWI addresses may range from 0 to 127.\n");
          break;
        }
        Serial << F("Flushing TWI interface at ") << value << endl;
        flushTwi(value);
        break;
      default:
        //Serial << "Command not recognized: " << (char)(action >> 8) << ',' << (char)action << endl;
        return false;
    }
    return true;
}

void loop() {
  static Metro phase_print(100);
  if (true && phase_print.check()) {
//    Serial << "Phase: " << sendReadMessage(PHASE_STATE_MSG, selectedAddress) << endl;
//    Serial << "Hz: " << sendReadMessage(READ_MEASURED_HZ_MSG, selectedAddress) / 6.0 / 7.0 << endl;
    Serial << "Duty Cycle: " << sendReadMessage(READ_DUTY_MSG, selectedAddress);
    Serial << " Current Limit: " << sendReadMessage(READ_CURRENT_LIMIT_MSG, selectedAddress) / 395.0;
    //Serial << " Amps: " << (int16_t)sendReadMessage(READ_MEASURED_CURRENT_MSG, selectedAddress); //measureCurrent(selectedAddress);
    Serial << " Amps: " << measureCurrent(selectedAddress);
    //Serial << " Volts: " << (int16_t)sendReadMessage(READ_MEASURED_VOLTAGE_MSG, selectedAddress); //measureVoltage(selectedAddress);
    Serial << " Volts: " << measureVoltage(selectedAddress);
    Serial << " Hz: " << sendReadMessage(READ_MEASURED_HZ_MSG, selectedAddress) / 6.0 / 7.0 * 2.0;
    Serial << " Ticks: " << (int16_t)sendReadMessage(PHASE_STATE_MSG, selectedAddress) << endl;
  }

//  static Metro reset_metro(50);
//  if (selectedAddress != 255 && reset_metro.check()) {
//    uint16_t test_duty = 420;
//    for (uint16_t i = 0; i < 100; i++) {
//      sendMessageValue(SET_START_DUTY_MSG, test_duty, selectedAddress);
//      int sanity_value = sendReadMessage(READ_START_DUTY_MSG, selectedAddress);
//      if (sanity_value == test_duty) {
//        break;
//      }
//    }
//    sendMessageValue(SET_DUTY_MSG, 100, selectedAddress);
//    delay(30);
//    sendMessageValue(SET_DUTY_MSG, 0, selectedAddress);
//    int check_value = sendReadMessage(READ_START_DUTY_MSG, selectedAddress);
//    Serial << "Got " << check_value << " Had reset? " << ((check_value != test_duty) ? "yes" : "NO!") << "\n";
//  }
  
  //static int32_t lastSentMicros;
  static byte byte0 = 0;
  static byte byte1 = 0;
  if (Serial.available()) {
    byte b = Serial.read();

    byte0 = byte1;
    byte1 = b;

    if (byte0 != 0 && byte1 != 0) {
      if (performAction((byte0 << 8) | byte1, selectedAddress)) {
        byte0 = 0;
        byte1 = 0;
        //lastSentMicros = micros();
      }
    }
  }
  /*if (flushTwi(selectedAddress)) {
    Serial << "message received " << (micros() - lastSentMicros) << "us after message sent\n";
  }*/
}
