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

#define CONTROL_RATE 128
Oscil <2048, AUDIO_RATE> aSin(SIN2048_DATA);

unsigned long nextUpdate;
uint8_t currentNote;

void setup() {
  Serial.begin(115200);
  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }

	aSin.setFreq(440);
	startMozzi(CONTROL_RATE);
  nextUpdate = millis();
  currentNote = 0;
  Serial.println("setup() complete");
}

void updateControl(){
  if (millis() > nextUpdate) {
    Serial.print("Changing note to ");
    Serial.println(currentNote);

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
