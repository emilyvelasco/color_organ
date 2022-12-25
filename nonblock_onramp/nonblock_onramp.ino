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

// AS7341 constants copied from Adafruit_AS7341.h
#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI
#define AS7341_REVID 0x91           ///< Chip revision register
#define AS7341_WHOAMI 0x92          ///< Chip ID register

// Utility functions calling into Mozzi twi_nonblock
#define TWI_IDLE 0
#define TWI_READING 1
#define TWI_WRITING 2

void twi_readStart(uint8_t i2c_address, uint8_t register_id, uint8_t length) {
  uint8_t retVal = 0;

  txBuffer[0] = register_id;
  txBufferIndex = 1;
  txBufferLength = 1;
  twi_initiateWriteTo(i2c_address, txBuffer, txBufferLength);

  Serial.println("Write I2C register address");
  twi_state = TWI_WRITING;
  while (TWI_WRITING == twi_state) {
    if ( TWI_MTX != twi_state ){
      txBufferIndex = 0;
      txBufferLength = 0;

      retVal = twi_initiateReadFrom(i2c_address, length);

      if (retVal != 0) {
        Serial.print("twi_initiateReadFrom failed with ");
        Serial.println(retVal);
        // Reset to idle in case of failure
        twi_state = TWI_IDLE;
      } else {
        twi_state = TWI_READING;
      }
    }
  }
  Serial.println("Receive data");
  while (TWI_READING == twi_state) {
    if (TWI_MRX != twi_state) {
      uint8_t readBuffer[32];
      uint8_t available = twi_readMasterBuffer(readBuffer, length);

      Serial.print("Available bytes: ");
      Serial.println(available);

      for (int i = 0; i < available; i++) {
        Serial.println(readBuffer[i]);
      }

      Serial.println("Read operation complete");
      twi_state = TWI_IDLE;
    }
  }
}

// AS7341 code
void as7341_setup() {
  uint8_t chip_id;
  uint8_t bytesRead = 0;

  Serial.print("twi_state = ");
  Serial.println(twi_state);
  Serial.print("twi_error = ");
  Serial.println(twi_error);

  twi_readStart(AS7341_I2CADDR_DEFAULT, AS7341_REVID, 2);

  if (1 != bytesRead) {
    Serial.print("Expected to read single byte for ID but read ");
    Serial.println(bytesRead);
  }
}

// Arduino boilerplate onetime setup code
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

  initialize_twi_nonblock();
  Serial.println("twi_nonblock initialized");

  as7341_setup();
  Serial.println("AS7341 initialized");

  Serial.println("Setup complete");
}

void updateControl(){
  if (millis() > nextUpdate) {
    aSin.setFreq(scale4[currentNote++]);
    currentNote = currentNote % 8;
    nextUpdate = millis() + 500;
  }
}

int updateAudio(){
	return aSin.next();
}

void loop() {
	audioHook();
}
