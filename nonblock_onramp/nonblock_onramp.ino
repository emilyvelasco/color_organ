/* Experiments building onramp to use AMS AS7341 sensor via twi_nonblock I2C.
 * Adafruit's sample code for AS7341 sensor uses Arduino's Wire library for
 * I2C communication.
 * https://github.com/adafruit/Adafruit_AS7341
 * 
 * Reading data via this library is a blocking operation which is
 * incompatible with timing-sensitive tasks like generating sound
 * via Mozzi library.
 * https://github.com/sensorium/Mozzi
 *
 * To resolve this issue, Mozzi includes a non-blocking I2C library under the
 * name twi_nonblock. This sketch explores using twi_nonblock to talk to
 * AMS AS7341 using Adafruit's library as a guide and also for declaration of
 * AS7341 I2C commands.
 *
 * AMS AS7341 datasheet references are against version v3-00 dated 2020-JUN-25
 *
 * Hardware: Mozzi twi_nonblock only supports AVR microcontrollers including
 * the ATMega328P used in Arduino Nano. This will not run on non-AVR chips
 * like the ESP32.
 */

#include <MozziGuts.h>   // at the top of your sketch
#include <mozzi_midi.h>
#include <Oscil.h>  // a template for an oscillator
#include <tables/sin2048_int8.h>  // a wavetable holding a sine wave
#include <twi_nonblock.h>

/////////////////////////////////////////////////////////////////////////////
// Declarations supporting Mozzi audio generation

// I expected these to be in Mozzi library somewhere but I failed to find it.
const int note_C4 = mtof(60);
const int note_D4 = mtof(62);
const int note_E4 = mtof(64);
const int note_F4 = mtof(65);
const int note_G4 = mtof(67);
const int note_A4 = mtof(69);
const int note_B4 = mtof(71);
const int note_C5 = mtof(72);

int scale4[] = {note_C4, note_D4, note_E4, note_F4, note_G4, note_A4, note_B4, note_C5};

// Mozzi setup
Oscil <2048, AUDIO_RATE> aSin(SIN2048_DATA);

// Tracking what note we are playing and when to update
unsigned long nextUpdate;
uint8_t currentNote;

/////////////////////////////////////////////////////////////////////////////
// Declarations supporting utility functions calling into Mozzi twi_nonblock

static volatile uint8_t async_status;
typedef enum {
  ASYNC_IDLE,
  ASYNC_READING,
  ASYNC_WRITING,
  ASYNC_COMPLETE,
  ASYNC_ERROR
} async_states_enum;

/////////////////////////////////////////////////////////////////////////////
//
// AS7341 constants copied from Adafruit_AS7341.h

#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI
#define AS7341_REVID 0x91           ///< Chip revision register
#define AS7341_WHOAMI 0x92          ///< Chip ID register
#define AS7341_ENABLE                                                          \
  0x80 ///< Main enable register. Controls SMUX, Flicker Detection, Spectral
       ///< Measurements and Power
#define AS7341_ATIME 0x81        ///< Sets ADC integration step count
#define AS7341_ASTEP_L 0xCA      ///< Integration step size low byte
#define AS7341_ASTEP_H 0xCB      ///< Integration step size high byte
#define AS7341_CFG1 0xAA         ///< Controls ADC Gain
#define AS7341_CFG6 0xAF         ///< Used to configure Smux
#define AS7341_STATUS2 0xA3      ///< Measurement status flags; saturation, validity
#define AS7341_CH0_DATA_L 0x95   ///< ADC Channel Data
#define AS7341_CFG8 0xB1         ///< AGC enable and FIFO threshold

/**
 * @brief Allowable gain multipliers for `setGain`
 *
 */
typedef enum {
  AS7341_GAIN_0_5X,
  AS7341_GAIN_1X,
  AS7341_GAIN_2X,
  AS7341_GAIN_4X,
  AS7341_GAIN_8X,
  AS7341_GAIN_16X,
  AS7341_GAIN_32X,
  AS7341_GAIN_64X,
  AS7341_GAIN_128X,
  AS7341_GAIN_256X,
  AS7341_GAIN_512X,
} as7341_gain_t;

/**
 * @brief Available SMUX configuration commands
 *
 */
typedef enum {
  AS7341_SMUX_CMD_ROM_RESET, ///< ROM code initialization of SMUX
  AS7341_SMUX_CMD_READ,      ///< Read SMUX configuration to RAM from SMUX chain
  AS7341_SMUX_CMD_WRITE, ///< Write SMUX configuration from RAM to SMUX chain
} as7341_smux_cmd_t;

/////////////////////////////////////////////////////////////////////////////
// Additional support data modified from Adafruit AS7341 library

/**
 * @brief Spectral Channel specifiers for configuration and reading
 *
 */
typedef enum {
  AS7341_CHANNEL_415nm_F1,
  AS7341_CHANNEL_445nm_F2,
  AS7341_CHANNEL_480nm_F3,
  AS7341_CHANNEL_515nm_F4,
  AS7341_CHANNEL_555nm_F5,
  AS7341_CHANNEL_590nm_F6,
  AS7341_CHANNEL_630nm_F7,
  AS7341_CHANNEL_680nm_F8,
  AS7341_CHANNEL_CLEAR,
  AS7341_CHANNEL_NIR,
} as7341_color_channel_t;

const uint8_t SMUX_config_size = 21;

// SMUX configuration copied from Adafruit_AS7341::setup_F1F4_Clear_NIR()
uint8_t SMUX_F1F4_Clear_NIR[SMUX_config_size] = {
  0x00, // Write starts at register zero
  // SMUX Config for F1,F2,F3,F4,NIR,Clear
  0x30, // F3 left set to ADC2
  0x01, // F1 left set to ADC0
  0x00, // Reserved or disabled
  0x00, // F8 left disabled
  0x00, // F6 left disabled
  0x42, // F4 left connected to ADC3/f2 left connected to ADC1
  0x00, // F5 left disbled
  0x00, // F7 left disbled
  0x50, // CLEAR connected to ADC4
  0x00, // F5 right disabled
  0x00, // F7 right disabled
  0x00, // Reserved or disabled
  0x20, // F2 right connected to ADC1
  0x04, // F4 right connected to ADC3
  0x00, // F6/F8 right disabled
  0x30, // F3 right connected to AD2
  0x01, // F1 right connected to AD0
  0x50, // CLEAR right connected to AD4
  0x00, // Reserved or disabled
  0x06  // NIR connected to ADC5
};

// SMUX configuration copied from Adafruit_AS7341::setup_F5F8_Clear_NIR()
uint8_t SMUX_F5F8_Clear_NIR[SMUX_config_size] = {
  0x00, // Write starts at register zero
  // SMUX Config for F5,F6,F7,F8,NIR,Clear
  0x00, // F3 left disable
  0x00, // F1 left disable
  0x00, // reserved/disable
  0x40, // F8 left connected to ADC3
  0x02, // F6 left connected to ADC1
  0x00, // F4/ F2 disabled
  0x10, // F5 left connected to ADC0
  0x03, // F7 left connected to ADC2
  0x50, // CLEAR Connected to ADC4
  0x10, // F5 right connected to ADC0
  0x03, // F7 right connected to ADC2
  0x00, // Reserved or disabled
  0x00, // F2 right disabled
  0x00, // F4 right disabled
  0x24, // F8 right connected to ADC2/ F6 right connected to ADC1
  0x00, // F3 right disabled
  0x00, // F1 right disabled
  0x50, // CLEAR right connected to AD4
  0x00, // Reserved or disabled
  0x06  // NIR connected to ADC5
};

/////////////////////////////////////////////////////////////////////////////
// Declarations supporting AS7341-specific code

// Total integration time will be (ATIME + 1) * (ASTEP + 1) * 2.78µS
// Datasheet: "typical integration time" is listed as 50ms (ATIME=29 ASTEP=599 50.04ms)
// ATIME is an 8-bit unsigned value. All values 0-255 are valid
// ASTEP is a 16-bit unsigned value. 0-65534 are valid, 65535 is reserved.
// (ATIME + 1) * (ASTEP + 1) must not exceed 65535.
const uint8_t ATIME=100;
const uint16_t ASTEP=999;

// State machine tracking progress through asynchrnous read operations
static volatile uint8_t as7341_read_status;
typedef enum {
  AS7341_READ_IDLE,
  AS7341_READ_SMUX,
  AS7341_READ_READY,
  AS7341_READ_DATA,
  AS7341_READ_COMPLETE,
  AS7341_READ_ERROR
} as7341_read_steps;

// Tracking whether we're reading sensors F1-F4 or F5-F8.
static volatile uint8_t as7341_sensor_group;
typedef enum {
  AS7341_SENSORS_IDLE,
  AS7341_SENSORS_F1_F4,
  AS7341_SENSORS_F5_F8,
  AS7341_SENSORS_COMPLETE
} as7341_sensor_groupings;

// Mozzi twi_nonblock has its own internal memory for I2C data (twi_masterBuffer)
// It has also allocated txBuffer and rxBuffer that it sometimes uses
// internally and sometimes used externally by sample code. When is it OK for
// external code to use txBuffer/rxBuffer? I'm not sure so I'm allocating
// my own async_buffer.
uint8_t async_buffer[32];

// Register AS7341_ENABLE (0x80) has multiple flags that are set/cleared
// for different operations. But reading and writing has to be done all
// eight bits of the byte at once, so we have to track what's going on.
uint8_t as7341EnableRegister;

// The most recent set of AS7341 sensor values: F1 to F8 + Clear + NIR
// Can index into this array via as7341_color_channel_t
uint16_t as7341Readings[10];

/////////////////////////////////////////////////////////////////////////////
// Utility functions for calling into Mozzi twi_nonblock

// Perform a blocking read from register_id of device at i2c_address.
//
// Yes, a blocking read defeats the point of using twi_nonblock, but
// 1. Sometimes we don't need nonblock and this is convenient.
// 2. It serves to illustrate the sequence of events for a twi_nonblock
//    read operation.
uint8_t blocking_read(uint8_t i2c_address, uint8_t register_id, uint8_t length) {
  // Send address of register where we want to start reading
  async_buffer[0] = register_id;
  twi_initiateWriteTo(i2c_address, async_buffer, 1);

  // Wait for register address write to complete.
  async_status = ASYNC_WRITING;
  while (ASYNC_WRITING == async_status) {
    if ( TWI_MTX != twi_state ){
      // Register address write completed, start read operation
      uint8_t retVal = twi_initiateReadFrom(i2c_address, length);

      if (retVal != 0) {
        Serial.print("ERROR: twi_initiateReadFrom failed with ");
        Serial.println(retVal);
        async_status = ASYNC_ERROR;
      } else {
        async_status = ASYNC_READING;
      }
    }
  }

  // Experimentally determined a tiny bit of delay is required between
  // twi_initiateReadFrom() and checking twi_state. In normal nonblocking
  // usage we would have handed control off to other code thus an explicit
  // delay is not normally needed.
  delay(1);

  // Wait for read operation to complete
  while (ASYNC_READING == async_status) {
    if (TWI_MRX != twi_state) {
      // Read operation completed, copy data
      uint8_t available = twi_readMasterBuffer(async_buffer, length);

      if (available != length) {
        Serial.print("ERROR: Wanted ");
        Serial.print(length);
        Serial.print(" bytes but only read ");
        Serial.print(available);
        async_status = ASYNC_ERROR;
      }
      async_status = ASYNC_IDLE;
    }
  }

  return async_status;
}

// Check for timeout during waits for asynchronous operation
const unsigned long async_timeout = 1000; // 1 second
unsigned long async_read_start;
bool asyncTimeOutCheck() {
  if (millis() < async_read_start) {
    Serial.println("Timeout counter overflow, resetting");
    async_read_start = millis();
    return false;
  }

  if (millis() > async_read_start + async_timeout) {
    Serial.println("Async operation timed out.");
    async_status = ASYNC_ERROR;
    return true;
  }

  return false;
}

// State machine to handle a single asynchronous read. Supports only a single
// read operation at any given time. Current state is tracked in global variable
// async_status. Caller should call this frequently and check for the following
// two end states in async_status:
//   * ASYNC_COMPLETE = data has successfully been read into async_buffer.
//   * ASYNC_ERROR = read operation failed for some reason.
// Once in either end state, this function would do nothing until the caller
// wants a new operation. To do so, set async_status to ASYNC_IDLE and call
// this method again to start a new read.
uint8_t async_read(uint8_t i2c_address, uint8_t register_id, uint8_t length) {
  switch (async_status) {
    case ASYNC_IDLE:
      // Send address of register where we want to start reading
      async_buffer[0] = register_id;
      twi_initiateWriteTo(i2c_address, async_buffer, 1);
      async_read_start = millis();
      async_status = ASYNC_WRITING;
      break;
    case ASYNC_WRITING:
      if ( TWI_MTX != twi_state ){
        // Register address write completed, start read operation
        uint8_t retVal = twi_initiateReadFrom(i2c_address, length);

        if (retVal != 0) {
          Serial.print("ERROR: twi_initiateReadFrom failed with ");
          Serial.println(retVal);
          async_status = ASYNC_ERROR;
        } else {
          async_status = ASYNC_READING;
        }
      }
      asyncTimeOutCheck();
      break;
    case ASYNC_READING:
      if (TWI_MRX != twi_state) {
        // Read operation completed, copy data
        uint8_t available = twi_readMasterBuffer(async_buffer, length);

        if (available != length) {
          Serial.print("ERROR: Wanted ");
          Serial.print(length);
          Serial.print(" bytes but only read ");
          Serial.print(available);
          async_status = ASYNC_ERROR;
        }
        async_status = ASYNC_COMPLETE;
      }
      asyncTimeOutCheck();
      break;
    case ASYNC_COMPLETE:
    case ASYNC_ERROR:
      // Do nothing until caller handles the situation and sets ASYNC_IDLE
      break;
    default:
      Serial.print("ERROR: Unexpected state encountered for async_read: ");
      Serial.println(async_status);
      break;
  }

  return async_status;
}

// Write a byte to specified register
// Reviewing twi_nonblock code, looks like this is technically a blocking
// operation but a single write is pretty fast and hasn't caused problems. (yet?)
uint8_t register_write_byte(uint8_t i2c_address, uint8_t register_id, uint8_t register_value) {
  twowire_beginTransmission(i2c_address);
  twowire_send(register_id);
  twowire_send(register_value);
  return twowire_endTransmission();
}

/////////////////////////////////////////////////////////////////////////////
// AS7341-specific code

// AS7341 initial setup: queries chip for its product number and wake it up
// from default SLEEP mode to IDLE. Then proceed to set additional
// configuration registers. Uses blocking I2C communication.
void as7341Setup() {
  uint8_t retVal;
  uint8_t revision;
  uint8_t part_number;
  uint8_t register_cfg8;

  // Read two bytes: start at AS7341_REVID and AS7341_WHOAMI immediately after
  retVal = blocking_read(AS7341_I2CADDR_DEFAULT, AS7341_REVID, 2);

  if (ASYNC_ERROR == retVal) {
    Serial.println("ERROR: Failed to retrieve AS7341 revision and chip ID.");
    return;
  }

  // Datasheet: REV_ID are bits 2:0 in register AS7341_REVID (0x91)
  revision = async_buffer[0] & 0x03;

  // Datasheet: Part number is AS7341_CHIP_ID 001001 (0x09) stored in
  // bits 7:2 of register AS7341_WHOAMI (0x92) = 0b 0010 01xx
  part_number = async_buffer[1] >> 2;

  if (AS7341_CHIP_ID != part_number) {
    Serial.print("ERROR: Expected to find part number AS7341_CHIP_ID (0x09) but found ");
    Serial.println(part_number);
    return;
  }
  Serial.print("Successfully connected to AS7341 revision ");
  Serial.println(revision);

  // AS7341 chip powers up to SLEEP, we have to bring it to IDLE
  as7341EnableRegister = 0x01;
  as7341UpdateEnableRegister();

  // Configure integration time.
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ATIME, ATIME);
  if (0 != retVal) {
    Serial.print("ERROR: Failed to set AS7341 sensor integration ATIME. ");
    Serial.println(retVal);
  }
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ASTEP_L, (uint8_t)(ASTEP & 0xFF));
  if (0 != retVal) {
    Serial.print("ERROR: Failed to set AS7341 sensor integration ASTEP low byte. ");
    Serial.println(retVal);
  }
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ASTEP_H, (uint8_t)((ASTEP>>8) & 0xFF));
  if (0 != retVal) {
    Serial.print("ERROR: Failed to set AS7341 sensor integration ASTEP high byte. ");
    Serial.println(retVal);
  }

  // Activate spectral auto gain control
  retVal = blocking_read(AS7341_I2CADDR_DEFAULT, AS7341_CFG8, 1);
  if (ASYNC_ERROR == retVal) {
    Serial.println("ERROR: Failed to retrieve CFG8.");
    return;
  } else {
    register_cfg8 = async_buffer[0];

    if (register_cfg8 & 0b00000100) {
      Serial.println("Spectral AGC already enabled.");
    } else {
      // Enable spectral AGC
      register_cfg8 |= 0b00000100;
      retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_CFG8, register_cfg8);
      if (0 != retVal) {
        Serial.print("ERROR: Failed to enable auto gain control ");
        Serial.println(retVal);
      }
    }
  }

  // Configuration complete, we are ready to start reading sensor
  as7341_read_status = AS7341_READ_IDLE;
  as7341_sensor_group = AS7341_SENSORS_F1_F4;
}

// Update AS7341 ENABLE register with our tracking value.
void as7341UpdateEnableRegister() {
  uint8_t retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ENABLE, as7341EnableRegister);
  if (0 != retVal) {
    Serial.print("ERROR: Failed to update AS7341 enable register to ");
    Serial.print(as7341EnableRegister);
    Serial.print(" error ");
    Serial.println(retVal);
  }
}

// AS7341 spectral measurement needs to be disabled for SMUX configuration
// and re-enabled for reading sensor.
void as7341EnableSpectralMeasurement(bool enable_measurement) {
  if (enable_measurement) {
    as7341EnableRegister |= 0b00000010;
  } else {
    as7341EnableRegister &= 0b11111101;
  }
  as7341UpdateEnableRegister();
}

// Tell AS7341 that SMUX configuration data is coming.
void as7341SMUXWrite() {
  uint8_t retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_SMUX_CMD_WRITE, (AS7341_SMUX_CMD_WRITE << 3));
  if (0 != retVal) {
    Serial.print("ERROR: Failed to configure AS7341 SMUX for writing ");
    Serial.println(retVal);
  }
}

// After SMUX configuration data has been sent, tell AS7341 to enable
// new configuration.
void as7341SMUXEnable() {
  as7341EnableRegister |= 0b00010000;
  as7341UpdateEnableRegister();
}

// Check ENABLE register value to see if SMUX Enable bit is set.
bool as7341SMUXEnableBitSet(uint8_t enableRegister) {
  if (enableRegister & 0b00010000) {
    return true;
  } else {
    return false;
  }
}

// Check STATUS2 register value to see if Spectral Valid bit is set.
bool as7341SpectralValid(uint8_t status2register) {
  if (status2register & 0b01000000) {
    return true;
  } else {
    return false;
  }
}

// Send AS7341 SMUX configuration
void as7341SendSMUXConfig(uint8_t* SMUX_config) {
  uint8_t retVal = twi_writeToBlocking(AS7341_I2CADDR_DEFAULT, SMUX_config, SMUX_config_size, true /* wait for completion */);
  if (0 != retVal) {
    Serial.print("ERROR: Failed to configure SMUX ");
    Serial.println(retVal);
  }
}

// Reads all channels of AS7341. Analogous to Adafruit_AS7341::readAllChannels
void as7341ConfigureSMUXAndRead(uint8_t* SMUX_config) {
  switch(as7341_read_status) {
    case AS7341_READ_IDLE:
      // Disable spectral measurement then send SMUX configuration
      as7341EnableSpectralMeasurement(false);
      as7341SMUXWrite();
      as7341SendSMUXConfig(SMUX_config);
      as7341SMUXEnable();
      as7341_read_status = AS7341_READ_SMUX;
      break;
    case AS7341_READ_SMUX:
      // Read ENABLE register to see SMUX status
      async_read(AS7341_I2CADDR_DEFAULT, AS7341_ENABLE, 1);
      if (ASYNC_COMPLETE == async_status) {
        if (!as7341SMUXEnableBitSet(async_buffer[0])) {
          // SMUX ready, update register value and start a read!
          as7341EnableRegister = async_buffer[0];
          as7341EnableSpectralMeasurement(true);
          as7341_read_status = AS7341_READ_READY;
        }
        async_status = ASYNC_IDLE;
      }
      break;
    case AS7341_READ_READY:
      // Read STATUS2 register to see if data is ready
      async_read(AS7341_I2CADDR_DEFAULT, AS7341_STATUS2, 1);
      if (ASYNC_COMPLETE == async_status) {
        if (as7341SpectralValid(async_buffer[0])) {
          // Spectrum is valid, move on to retrieve that data.
          as7341_read_status = AS7341_READ_DATA;
        }
        async_status = ASYNC_IDLE;
      }
      break;
    case AS7341_READ_DATA:
      // Read 12 bytes (6 16-bit spectral sensor values) from AS7341_CH0_DATA_L
      async_read(AS7341_I2CADDR_DEFAULT, AS7341_CH0_DATA_L, 12);
      if (ASYNC_COMPLETE == async_status) {
        async_status = ASYNC_IDLE;
        as7341_read_status = AS7341_READ_COMPLETE;
      }
      break;
    case AS7341_READ_COMPLETE:
      // Sensor read and waiting for reset to AS7341_READ_IDLE
      break;
    case AS7341_READ_ERROR:
      // Do nothing, hold in error state waiting for reset to AS7341_READ_IDLE
      break;
    default:
      Serial.print("Unexpected value for as7341_read_status ");
      Serial.println(as7341_read_status);
      break;
  }
}

// Reads all channels of AS7341. Analogous to Adafruit_AS7341::readAllChannels
void as7341ReadAllChannels() {
  switch(as7341_sensor_group) {
    case AS7341_SENSORS_IDLE:
      as7341_sensor_group = AS7341_SENSORS_F1_F4;
      // Fall through to case AS7341_SENSORS_F1_F4:
    case AS7341_SENSORS_F1_F4:
      as7341ConfigureSMUXAndRead(SMUX_F1F4_Clear_NIR);
      if (AS7341_READ_COMPLETE == as7341_read_status) {
        // Last two are Clear and NIR which are duplicated
        for(int i = 0; i < 4; i++) {
          as7341Readings[i] = async_buffer[(i*2)] + (async_buffer[(i*2)+1]<<8);
        }
        as7341_read_status = AS7341_READ_IDLE;
        as7341_sensor_group = AS7341_SENSORS_F5_F8;
      }
      break;
    case AS7341_SENSORS_F5_F8:
      as7341ConfigureSMUXAndRead(SMUX_F5F8_Clear_NIR);
      if (AS7341_READ_COMPLETE == as7341_read_status) {
        for(int i = 0; i < 6; i++) {
          as7341Readings[i+4] = async_buffer[(i*2)] + (async_buffer[(i*2)+1]<<8);
        }
        as7341_read_status = AS7341_READ_IDLE;
        as7341_sensor_group = AS7341_SENSORS_COMPLETE;
      }
      break;
    case AS7341_SENSORS_COMPLETE:
      // Do nothing until caller resets to AS7341_SENSORS_IDLE
      break;
    default:
      Serial.println("ERROR: Unexpected state in as7341_sensor_group");
      break;
  }
}

// Print full set of spectral sensor readings on a compact single line
void as7341PrintReadings() {
  for(int i = 0; i < 10; i++) {
    Serial.print("\t");
    Serial.print(as7341Readings[i]);
  }
  Serial.println();
}

/////////////////////////////////////////////////////////////////////////////
// Mozzi callback functions

// Update audio control (standard Mozzi callback)
// This is called very frequently (*) but not as frequently as updateAudio()
// This is where we can perform logic that usually lives in loop() of an
// Arduino sketch. We don't have to finish as quickly as updateAudio(), but
// we still can't dawdle too much and we still can't use blocking waits like
// delay().
//
// (*) Mozzi running STANDARD audio mode on an Arduino Nano (ATmega328P)
// will call updateControl() roughly 64 times per second.
void updateControl(){
  if (millis() > nextUpdate) {
    aSin.setFreq(scale4[currentNote++]);
    currentNote = currentNote % 8;
    nextUpdate = millis() + 500;
  }
  as7341ReadAllChannels();
  if (AS7341_SENSORS_COMPLETE == as7341_sensor_group) {
    as7341PrintReadings();
    // Reset to IDLE so as7341ReadAllChannels() will start a new reading
    // the next time it is called.
    as7341_sensor_group = AS7341_SENSORS_IDLE;
  }
}

// Update audio signal (standard Mozzi callback)
// This is called EXTREMELY frequently (*) to update the currently generated
// signal. The code needs to be very short and fast. Delay will cause
// audible glitches like clicks or pops. Do only the most critical work
// here, everything else can go in updateControl().
//
// (*) Mozzi's STANDARD audio mode runs at 16384Hz, so updateAudio() is called
// 16384 times per second.
int updateAudio(){
  return aSin.next();
}

/////////////////////////////////////////////////////////////////////////////
// Arduino callback functions

// One-time initial setup (standard Arduino boilerplate)
void setup() {
  Serial.begin(115200);
  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }
  Serial.println("Serial online");

  aSin.setFreq(440);
  startMozzi(CONTROL_RATE);
  nextUpdate = millis() + 500;
  currentNote = 0;
  Serial.println("Mozzi online");

  async_status = ASYNC_IDLE;
  initialize_twi_nonblock();
  Serial.println("twi_nonblock initialized");

  as7341Setup();

  Serial.println("Setup complete");
}

// Endless loop (standard Arduino boilerplate)
// In most Arduino sketches, this is where most of the logic lives.
// However, in a Mozzi sketch, we give audioHook() immediate control so it
// gets first dibs on processor computing time. Once critical tasks are
// taken care of, we can do our work in updateControl().
void loop() {
  audioHook();
}
