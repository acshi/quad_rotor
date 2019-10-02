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
        return hasDiscarded;
      }
      int b = Wire.read();
      if (b == 0xff) {
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
  
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(command);
  Wire.write(MSG_END_BYTE);
  Wire.endTransmission();

  // Look for the message mark to know the message has been received.
  Wire.requestFrom(address, 3);
  
  int16_t b = Wire.read();
  if (b == MSG_MARK_BYTE) {
    return (uint16_t)(Wire.read() << 8) | Wire.read();
  } else if (b != -1) {
    Serial << "Got " << b << " instead\n";
  }
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
  return value * 6 * 2.23 / 8192 - 0.01;
}

double measureCurrent(int address) {
  uint16_t value = sendReadMessage(READ_MEASURED_CURRENT_MSG, address);
  double current = value / 395.0 / 2.0;
  if (current < 0.018) {
    current = 0.0;
  }
  return current;
}

bool performAction(int16_t action, int address) {
    uint16_t value;
    switch (action) {
      case 'hl':
      case 'hp':
        Serial << "Commands include...\n";
        Serial << "hl, hp, to show this help\n";
        Serial << "sd, gd, (set/get) duty cycle target\n";
        Serial << "mh, measured hz target\n";
        Serial << "sc, gc, mc, (set/get/measured) current target\n";
        Serial << "mv, measured motor voltage\n";
        Serial << "mt, measured temperature\n";
        Serial << "er, check for error conditions\n";
        Serial << "tw, return TWI address of device\n";
        Serial << "sa, scan addresses for connected TWI devices\n";
        Serial << "se, set device to communicate with from those scanned\n";
        Serial << "fl, flush communcations buffers with a specific device\n";
        break;
      case 'sd':
        value = (uint16_t)Serial.parseInt();
        Serial << "Setting duty cycle target to: " << (int16_t)value << endl;
        sendMessageValue(SET_DUTY_MSG, value, address);
        break;
      case 'gd':
        value = sendReadMessage(READ_DUTY_MSG, address);
        Serial << "Duty cycle target: " << (int16_t)value << endl;
        break;
      case 'mh':
        value = sendReadMessage(READ_MEASURED_HZ_MSG, address);
        Serial << "Hz measurement: " << (int16_t)value / 7.0 / 6.0 << endl;
        break;
      case 'sc':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting current limit to: " << value << " (" << (value / 395.0) << "A)\n";
        sendMessageValue(SET_CURRENT_LIMIT_MSG, value, address);
        break;
      case 'gc':
        value = sendReadMessage(READ_CURRENT_LIMIT_MSG, address);
        Serial << "Current limit: " << value << " (" << (value / 395.0) << "A)\n";
        break;
      case 'ss':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting start duty cycle to: " << value << "\n";
        sendMessageValue(SET_START_DUTY_MSG, value, address);
        break;
      case 'gs':
        value = sendReadMessage(READ_START_DUTY_MSG, address);
        Serial << "Start duty cycle: " << value << "\n";
        break;
      case 'mc':
        Serial << "Current measurement: " << measureCurrent(address) << "A\n";
        break;
      case 'mv':
        Serial << "Motor voltage: " << measureVoltage(address) << "V\n";
        break;
      case 'mt':
        value = sendReadMessage(READ_TEMPERATURE_MSG, address);
        Serial << "Temperature (celsius): " << *(int16_t*)(&value) / 10.0 << endl;
        break;
      case 'er':
        value = sendReadMessage(READ_ERROR_MSG, address);
        Serial << "Error: ";
        switch (value) {
          case 0:
            Serial << "No error";
            break;
          case 1:
            Serial << "Overcurrent shut-off";
            break;
          case 2:
            Serial << "VDS fault";
            break;
          default:
            Serial << "invalid error/communication problem";
            break;
        }
        Serial << endl;
        break;
      case 'tw':
        value = sendReadMessage(ADDRESS_MSG, address);
        Serial << "TWI Address: " << value << endl;
        break;
      case 'sa':
        scanAddrs();
        break;
      case 'se':
        value = Serial.parseInt();
        if (value < 8 || value > 127) {
          Serial << "Address " << value << " is invalid. TWI addresses may range from 8 to 127.\n";
          break;
        }
        if (true || availableAddrs[value]) {
          selectedAddress = value;
          Serial << "Selected address " << selectedAddress << endl;
        } else {
          Serial << "Address " << value << " is not available. Check your wiring and then rescan with 'sa'\n";
        }
        break;
      case 'fl':
        value = Serial.parseInt();
        if (value < 8 || value > 127) {
          Serial << "Address " << value << " is invalid. TWI addresses may range from 0 to 127.\n";
          break;
        }
        Serial << "Flushing TWI interface at " << value << endl;
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
  if (false && phase_print.check()) {
//    Serial << "Phase: " << sendReadMessage(PHASE_STATE_MSG, selectedAddress) << endl;
//    Serial << "Hz: " << sendReadMessage(READ_MEASURED_HZ_MSG, selectedAddress) / 6.0 / 7.0 << endl;
    Serial << "Duty Cycle: " << sendReadMessage(READ_DUTY_MSG, selectedAddress);
    Serial << " Current Limit: " << sendReadMessage(READ_CURRENT_LIMIT_MSG, selectedAddress) / 395.0;
    Serial << " Amps: " << measureCurrent(selectedAddress);
    Serial << " Volts: " << measureVoltage(selectedAddress);
    Serial << " Hz: " << sendReadMessage(READ_MEASURED_HZ_MSG, selectedAddress) / 6.0 / 7.0;
    Serial << " Ticks: " << (int16_t)sendReadMessage(PHASE_STATE_MSG, selectedAddress) << endl;
  }

  static Metro reset_metro(500);
  if (false && reset_metro.check()) {
    sendMessageValue(SET_START_DUTY_MSG, 1000, selectedAddress);
    sendMessageValue(SET_DUTY_MSG, 100, selectedAddress);
    sendMessageValue(SET_DUTY_MSG, 0, selectedAddress);
    int check_value = sendReadMessage(READ_START_DUTY_MSG, selectedAddress);
    Serial << "Had reset? " << ((check_value == 0xffff) ? "yes" : "NO!") << "\n";
  }
  
  static int32_t lastSentMicros;
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
        lastSentMicros = micros();
      }
    }
  }
  /*if (flushTwi(selectedAddress)) {
    Serial << "message received " << (micros() - lastSentMicros) << "us after message sent\n";
  }*/
}
