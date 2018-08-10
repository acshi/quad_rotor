#include <Streaming.h>

#include <Wire.h>

#define MOTOR1_MASK (1 << 6) // indicates to target motor1/second motor instead of motor0.
#define SET_DUTY_MSG 1
#define READ_DUTY_MSG 2
#define READ_MEASURED_HZ_MSG 3
#define SET_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_LIMIT_MSG 5
#define READ_MEASURED_CURRENT_MSG 6
#define READ_TEMPERATURE_MSG 7
#define READ_ERROR_MSG 8
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
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

  int motor_mask = (motor_on == 1) ? MOTOR1_MASK : 0;
  
  Wire.beginTransmission(address);
  Wire.write(command | motor_mask);
  Wire.write(command | motor_mask);
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
  int motor_mask = (motor_on == 1) ? MOTOR1_MASK : 0;
  
  Wire.beginTransmission(address);
  Wire.write(command | motor_mask);
  Wire.write(command | motor_mask);
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
        Serial << "tm, temperature\n";
        Serial << "er, check for error conditions\n";
        Serial << "tw, return TWI address of device\n";
        Serial << "sa, scan addresses for connected TWI devices\n";
        Serial << "se, set device to communicate with from those scanned\n";
        Serial << "fl, flush communcations buffers with a specific device\n";
        break;
      case 'sm':
        value = (uint16_t)Serial.parseInt();
        if (value > 0) {
          value = 1;
        }
        motor_on = value;
        Serial << "Setting active motor to: " << motor_on << endl;
        break;
      case 'gm':
        Serial << "Active motor is: " << motor_on << endl;
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
        Serial << "Hz measurement: " << (int16_t)value / 7.0 << endl;
        break;
      case 'sc':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting current limit to: " << value << " (" << (value * 22.3 / 4096) << "A)\n";
        sendMessageValue(SET_CURRENT_LIMIT_MSG, value, address);
        break;
      case 'gc':
        value = sendReadMessage(READ_CURRENT_LIMIT_MSG, address);
        Serial << "Current limit: " << value << " (" << (value * 22.3 / 4096) << "A)\n";
        break;
      case 'mc':
        value = sendReadMessage(READ_MEASURED_CURRENT_MSG, address);
        Serial << "Current measurement: " << value << " (" << (value * 22.3 / 4096) << "A)\n";
        break;
      case 'tm':
        value = sendReadMessage(READ_TEMPERATURE_MSG, address);
        Serial << "Temperature (celsius): " << value << endl;
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
