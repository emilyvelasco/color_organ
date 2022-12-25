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
//
// AS7341 constants copied from Adafruit_AS7341.h
//
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

//
// End of excerpt directly copied from Adafruit_AS7341.h
//
/////////////////////////////////////////////////////////////////////////////

// Utility functions calling into Mozzi twi_nonblock
static volatile uint8_t async_status;
#define ASYNC_IDLE 0
#define ASYNC_READING 1
#define ASYNC_WRITING 2
#define ASYNC_ERROR 255

// Mozzi twi_nonblock has its own internal memory for I2C data (twi_masterBuffer)
//
// It has also allocated txBuffer and rxBuffer that it sometimes uses
// internally and sometimes used externally by sample code. When is it OK for
// external code to use txBuffer/rxBuffer? I'm not sure so I'm allocating
// my own asyncBuffer.
uint8_t asyncBuffer[32];

// Perform a blocking read from register_id of device at i2c_address.
//
// Yes, a blocking read defeats the point of using twi_nonblock, but
// 1. Sometimes we don't need nonblock and this is convenient.
// 2. It serves to illustrate the sequence of events for a twi_nonblock
//    read operation.
uint8_t blocking_read(uint8_t i2c_address, uint8_t register_id, uint8_t length) {
  // Send address of register where we want to start reading
  asyncBuffer[0] = register_id;
  twi_initiateWriteTo(i2c_address, asyncBuffer, 1);

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
  // delay is not normally be needed.
  delay(1);

  // Wait for read operation to complete
  while (ASYNC_READING == async_status) {
    if (TWI_MRX != twi_state) {
      // Read operation completed, copy data
      uint8_t available = twi_readMasterBuffer(asyncBuffer, length);

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

// Write a byte to specified register
// Reviewing twi_nonblock code, looks like this is technically a blocking
// operation but a single write is pretty fast and hasn't caused problems. (yet?)
uint8_t register_write_byte(uint8_t i2c_address, uint8_t register_id, uint8_t register_value) {
  twowire_beginTransmission(i2c_address);
  twowire_send(register_id);
  twowire_send(register_value);
  return twowire_endTransmission();
}

// AS7341 initial setup: queries chip for its product number and wake it up
// from default SLEEP mode to IDLE. Uses blocking I2C communication.
void as7341_setup() {
  uint8_t retVal;
  uint8_t revision;
  uint8_t part_number;

  // Read two bytes: start at AS7341_REVID and AS7341_WHOAMI immediately after
  retVal = blocking_read(AS7341_I2CADDR_DEFAULT, AS7341_REVID, 2);

  if (ASYNC_ERROR == retVal) {
    Serial.println("ERROR: Failed to retrieve AS7341 revision and chip ID.");
    return;
  }

  // Datasheet: REF_ID are bits 2:0 in register AS7341_REVID (0x91)
  revision = asyncBuffer[0] & 0x03;

  // Datasheet: Part number is AS7341_CHIP_ID 001001 (0x09) stored in
  // bits 7:2 of register AS7341_WHOAMI (0x92) = 0b 0010 01xx
  part_number = asyncBuffer[1] >> 2;

  if (AS7341_CHIP_ID != part_number) {
    Serial.print("ERROR: Expected to find part number AS7341_CHIP_ID (0x09) but found ");
    Serial.println(part_number);
    return;
  }
  Serial.print("Successfully connected to AS7341 revision ");
  Serial.println(revision);

  // AS7341 chip powers up to SLEEP, we have to bring it to IDLE
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ENABLE, 0x01);
  if (0 != retVal) {
    Serial.print("ERROR: Failed to send AS7341 enable command to bring it from SLEEP to IDLE. ");
    Serial.println(retVal);
  }

  // Configure integration time.
  // Total integration time will be (ATIME + 1) * (ASTEP + 1) * 2.78µS
  // Datasheet: "typical integration time" is listed as 50ms (ATIME=29 ASTEP=599 50.04ms)
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ATIME, 29);
  if (0 != retVal) {
    Serial.print("ERROR: Failed to set AS7341 sensor integration ATIME. ");
    Serial.println(retVal);
  }
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ASTEP_L, (uint8_t)(599 & 0xFF));
  if (0 != retVal) {
    Serial.print("ERROR: Failed to set AS7341 sensor integration ASTEP low byte. ");
    Serial.println(retVal);
  }
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_ASTEP_H, (uint8_t)((599>>8) & 0xFF));
  if (0 != retVal) {
    Serial.print("ERROR: Failed to set AS7341 sensor integration ASTEP high byte. ");
    Serial.println(retVal);
  }

  // Configure sensor gain.
  // TODO: Figure out how auto-gain control works on this sensor
  retVal = register_write_byte(AS7341_I2CADDR_DEFAULT, AS7341_CFG1, AS7341_GAIN_256X);
  if (0 != retVal) {
    Serial.print("ERROR: Failed to configure AS7341 sensor gain. ");
    Serial.println(retVal);
  }
}

// Update audio control (standard Mozzi callback)
// This is called very frequently but not as frequently as updateAudio()
// This is where we can perform logic that usually lives in loop() of an
// Arduino sketch. We don't have to finish as quickly as updateAudio(), but
// we still can't dawdle too much and we still can't use blocking waits like
// delay().
void updateControl(){
  if (millis() > nextUpdate) {
    aSin.setFreq(scale4[currentNote++]);
    currentNote = currentNote % 8;
    nextUpdate = millis() + 500;
  }
}

// Update audio signal (standard Mozzi callback)
// This is called EXTREMELY frequently to update the currently generated
// signal. The code needs to be very short and fast. Delay will cause
// audible glitches like clicks or pops. Do only the most critical work
// here, everything else can go in updateControl().
int updateAudio(){
  return aSin.next();
}

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
  nextUpdate = millis();
  currentNote = 0;
  Serial.println("Mozzi online");

  async_status = ASYNC_IDLE;
  initialize_twi_nonblock();
  Serial.println("twi_nonblock initialized");

  as7341_setup();

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
