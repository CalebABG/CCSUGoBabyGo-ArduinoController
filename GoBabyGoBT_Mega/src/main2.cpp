#include <Arduino.h>
#include <AltSoftSerial.h>
#include <TimerOne.h>
#include <CRC32.h>

/* Clear FTDI-TTL-232R Cable
  Pin 1 GND (Black)
  Pin 2 CTS (Brown)
  Pin 3 VCC (Red)
  Pin 4 TXD (Orange)
  Pin 5 RXD (Yellow)
  Pin 6 RTS (Green)
*/

/*
Mega Interrupt pins:
Mega, Mega2560, MegaADK - 2, 3, 18, 19, 20, 21

Wires for Arduino:
- Black stripe = Motor B Output
*/

/*
Preprocessor Defines
*/

#define Bluetooth Serial2
#define MotorController Serial1

#define p_startByte 0x01
#define p_endByte 0x04

#define p_sensorDataId 0xD7       // sd = 215
#define p_stopMotorsId 0xE0       // sm = 224
#define p_parentalOverrideId 0xDF // po = 223

#define p_MAXDataLength 8

// Sabertooth Commands
// Limits for each motor
#define SBT_MOTOR1_FULL_FORWARD 127
#define SBT_MOTOR1_FULL_REVERSE 1

#define SBT_MOTOR2_FULL_FORWARD 255
#define SBT_MOTOR2_FULL_REVERSE 128

// Full stop commands for Motor1 and Motor2
#define SBT_MOTOR1_STOP 64
#define SBT_MOTOR2_STOP 192

// Shut down both motors
#define SBT_ALL_STOP 0

/*
Variables
*/

/* AltSoftSerial Serial/Bluetooth stuff
Board          TX  RX
Arduino Uno    9   8 
Arduino Mega   46  48
*/

// Set to correct/matching baud on MotorController
uint16_t motorControllerBaud = 9600;

// Set to correct/matching baud for Bluetooth (query BT using command reference)
uint16_t bluetoothBaud = 115200;
// uint16_t bluetoothBaud = 57600;

uint16_t serialBaud = 9600;

// the possible states of the state-machine
typedef enum
{
  BT_Idle,
  BT_SensorData,
  BT_StopMotors,
  BT_ParentOverride
} BluetoothStates;

// current state-machine state
BluetoothStates bluetooth_state = BT_StopMotors;

// Packet timer
uint16_t period = 1000; // in milliseconds
unsigned long time_now = 0;
unsigned long time_now2 = 0;

// Motor stop safety
// set initially to false until
// we receive a go signal from the motors
volatile bool receivedAck = false;
volatile bool main_acked = false;

// Variables for MotorController and processing of incoming
// Bluetooth data
byte last_ud = 0;
byte last_lr = 0;

/*
Structs/Classes
*/

typedef struct GoPacket_t
{
  byte start_byte = p_startByte;
  byte id;
  uint32_t crc32_chksum;
  bool ack = false;
  byte data_size_in_bytes;
  byte data[p_MAXDataLength] = {0};
  byte end_byte = p_endByte;

} GoPacket;


/*
Helper functions
*/

void PrintHex83(uint8_t* data, uint8_t length) // prints 8-bit data in hex
{
  char tmp[length * 2 + 1];
  byte first;
  int j = 0;
  for (uint8_t i = 0; i < length; i++)
  {
    first = (data[i] >> 4) | 48;
    if (first > 57)
      tmp[j] = first + (byte)39;
    else
      tmp[j] = first;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57)
      tmp[j] = first + (byte)39;
    else
      tmp[j] = first;
    j++;
  }

  tmp[length * 2] = 0;
  Serial.println(tmp);
}

void printHex(byte x)
{
  if (x < 16)
    Serial.print("0");
  Serial.print(x, HEX);
}

void printByteArray(byte bytearr[], size_t length)
{
  for (size_t i = 0; i < length; i++)
  {
    printHex(bytearr[i]);
  }
  Serial.println();
}

// Code complexity should be
// MINIMAL in interrupt!
// Long running code in interrupt can cause timing issues
// If we haven't received a packet with an ACK = 1
// then stop the motors
void BT_ACK_Interrupt()
{
  noInterrupts();
  main_acked = receivedAck;
  interrupts();
}

// ask about how to properly handle isr
// problem of knowing that we've not gotten
// a packet/bluetooth has disconnected from Arduino perspective
uint16_t safetyTimer = 350; // in milliseconds
bool stopMotors = false;
void safetyIsrMotors()
{
  if (millis() > time_now2 + safetyTimer)
  {
    time_now2 = millis();

    if (main_acked == true)
    {
      stopMotors = false;
    }
    else if (main_acked == false)
    {
      // If we haven't stopped the motors, STOP them now
      if (stopMotors == false)
      {
        Serial.println("Safety: Stopping Motors");
        // send stop command to motor controller
        MotorController.write(0x0);
        stopMotors = true;
      }
    }

    // Set the main ACK to false for safety stop of Motors
    main_acked = false;
  }
}


void packetTest2()
{
  uint32_t packet_checksum = 0xffffffff;
  byte packet_data[p_MAXDataLength] = {7, 7, 7, 7};

  GoPacket packet;

  /* Comment when ACTUALLY USING; outgoing: ack should ALWAYS be false*/
  packet.ack = false;
  packet.id = 3;

  memcpy(packet.data, packet_data, sizeof(packet.data));
  packet.data_size_in_bytes = sizeof(packet_data);

  byte* packet_bytes = (byte *)&packet;
  // packet_checksum = CRC32::calculate(byteArray, sizeof(packet));
  packet.crc32_chksum = packet_checksum;

  // Serial.print(time_now / 1000); Serial.print(" - ");
  // Serial.println(sizeof(packet));
  // printByteArray(packet_bytes, sizeof(packet));

  // Example Packet
  // 01 03 FF FF FF FF 00 08 07 07 07 07 00 00 00 00 04
  Bluetooth.write(packet_bytes, sizeof(packet));
}


void sendArduinoPacket()
{
  if (millis() > time_now + period)
  {
    time_now = millis();
    packetTest2();
  }
}


// Todo:
void sendHardwareStatus()
{
}


void readSerial()
{
  if (Serial.available())
  {
    // String s;
    while (Serial.available() > 0)
    {
      Bluetooth.print(Serial.read());
      //   s += (char)Serial.read();
    }
    Bluetooth.println();
    // bluetooth.print(s);
  }

  if (Bluetooth.available())
  {
    // String s;
    while (Bluetooth.available() > 0)
    {
      Serial.print(Bluetooth.read());
      // s += (char)bluetooth.read();
    }
    Serial.println();

    // Serial.print(s);
  }
}

float m_drive(byte v)
{
  return (-.376 * v) + 48;
}

float m_angular_offset(byte v)
{
  return (v * .117) - 15;
}

/*
bluetooth State-Machine
*/

void processStopMotors()
{
  Serial.println("StopMotors State!");
  Serial.println("Stopping Motors: Sending 0x0");

  // send 0x00 to motor controller on board
  MotorController.write(0x0);
}

uint32_t composeUInt32(byte byte1, byte byte2, byte byte3, byte byte4)
{
  return ((long)byte4 << 24) + ((long)byte3 << 16) + ((long)byte2 << 8) + ((long)byte1);
}

void processSensorData(byte data[])
{
  /*
    Ex. Packet - indexes 2-5 are for checksum; indexes 7-15 are for data
    Index values are in HEX (ex. 0xff = 255) and (ex. 0x0f = 15)
    Index:    0     1     2     3     4     5     6     7     8     9     10     11     12     13     14     15     16
           {  01    03    ff    ff    ff    ff    01    08    01    02    03     04     05     06     07     08     04  }
  */
  // Serial.println("SensorData State!");

  //Serial.print("AccelXY: "); PrintHex83(data, sizeof(data));

  last_ud = data[0];
  last_lr = data[1];

  last_ud = constrain(last_ud, 0, 255);
  last_lr = constrain(last_lr, 0, 255);

  byte m_ud1 = (byte)floor((m_drive(last_ud) + SBT_MOTOR1_STOP));
  byte m_ud2 = (byte)floor(m_drive(last_ud) + SBT_MOTOR2_STOP);

  byte angular_offset = m_angular_offset(last_lr);

  byte motor1_speed = m_ud1 + angular_offset;
  byte motor2_speed = m_ud2 - angular_offset;

  MotorController.write(motor1_speed);
  MotorController.write(motor2_speed);
}

void processParentalOverride()
{
  Serial.println("ParentalOverride State!");
}

uint16_t state = 0;
byte incomingByte;

// Bluetooth Packet data-structure
byte packet_id;
uint32_t packet_checksum;
byte packet_data_length;
byte sensor_accel_x;
byte sensor_accel_y;

void bluetoothStateMachine()
{
  if (Bluetooth.available() > 0)
  {
    incomingByte = Bluetooth.read();
    Serial.println(incomingByte);

    switch (state)
    {
    case 0: // 1st Byte (0x01)
      if (incomingByte == p_startByte)
      {
        state++;
      }
      break;

    case 1:
      if (incomingByte == p_stopMotorsId ||
          incomingByte == p_sensorDataId ||
          incomingByte == p_parentalOverrideId)
      {
        packet_id = incomingByte;
        state++;
      }
      else
      {
        state = 0;
      }
      break;

    case 2:
      packet_checksum = (long)incomingByte << 24;
      state++;
      break;

    case 3:
      packet_checksum += (long)incomingByte << 16;
      state++;
      break;

    case 4:
      packet_checksum += (long)incomingByte << 8;
      state++;
      break;

    case 5:
      packet_checksum += (long)incomingByte << 0;
      state++;
      break;

    case 6:
      if (incomingByte == 1 || incomingByte == 0)
      {
        main_acked = incomingByte;
        // Serial.println(main_acked);
        state++;
      }
      else
      {
        state = 0;
      }
      break;

    case 7:
      packet_data_length = incomingByte;
      state++;
      break;

    case 8:
      sensor_accel_x = incomingByte;
      state++;
      break;

    case 9:
      sensor_accel_y = incomingByte;
      state++;
      break;

    case 16:     // 17th Byte (End of packet: 0x04)
      state = 0; // Go back to state 0 to look for a new packet

      // handle state based on ID
      switch (packet_id)
      {
      case p_stopMotorsId:
        processStopMotors();
        break;

      case p_sensorDataId:
        byte data[2] = {sensor_accel_x, sensor_accel_y};
        processSensorData(data);
        break;

      case p_parentalOverrideId:
        processParentalOverride();
        break;
      } // end of switch

      // reset packet_checksum
      packet_checksum = 0;
      break;

    default: // defalt case is to increase the state
      state++;
      break;
    }
  }
}

/*
Arduino Setup and Loop 
*/

void setup()
{
  // Wait for newly restarted system to stabilize
  // bluetooth needs 500ms to be ready to ender command mode
  delay(500);

  // setup onboard LED
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize timer1, and set a 1/2 second period
  // attaches callback() as a timer overflow interrupt
  // Timer1.initialize(500000);
  // Timer1.attachInterrupt(BT_ACK_Interrupt);

  // wait for Arduino Serial Monitor to open
  while (!Serial)
  {
  };

  // Setup Serial
  Serial.begin(serialBaud);
  Serial.println("Arduino Up!");

  // setup MotorController
  MotorController.begin(motorControllerBaud);
  delay(1200);

  // Setup bluetooth
  // Bluetooth.begin(bluetoothBaud);
  // Bluetooth.print("$$$");
  Bluetooth.begin(bluetoothBaud);

  uint16_t delay_ = 1050;
  Bluetooth.write("AT"); // send at command
  delay(delay_);

  Bluetooth.write("AT+NAMEGoBabyGoBT"); // Set BT Name
  delay(delay_);

  Bluetooth.write("AT+BAUD8"); // Set BT Baud
  delay(delay_);

  // Serial.println("AT+PIN"); // Set PIN
  // delay(delay_);

  //Wait for response from bluetooth: 'CMD'
  delay(300);

  // fastForward();

  // delay(1000);
}

void loop()
{
  // bluetooth state-machine read data/messages
  bluetoothStateMachine();

  // handle isr: safety measures for stopping car if not acked
  safetyIsrMotors();

  // send status of hardware to phone
  sendHardwareStatus();

  // send example packet to android
  // sendArduinoPacket();

  // read incoming Serial data/messages
  // readSerial();
}