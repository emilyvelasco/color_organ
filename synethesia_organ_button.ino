/*  Example playing a sinewave at a set frequency,
    using Mozzi sonification library.

    Demonstrates the use of Oscil to play a wavetable.

    Circuit: Audio output on digital pin 9 on a Uno or similar, or
    DAC/A14 on Teensy 3.1, or
    check the README or http://sensorium.github.io/Mozzi/

    Mozzi documentation/API
    https://sensorium.github.io/Mozzi/doc/html/index.html

    Mozzi help/discussion/announcements:
    https://groups.google.com/forum/#!forum/mozzi-users

    Tim Barrass 2012, CC by-nc-sa.
*/

#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <Adafruit_AS7341.h>

#define BUTTON_PIN 18 // GIOP21 pin connected to button


Adafruit_AS7341 as7341;

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> redSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> orangeSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> yellowSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> greenSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> cyanSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> blueSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> indigoSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> violetSin(SIN2048_DATA);



byte redGain;
byte orangeGain;
byte yellowGain;
byte greenGain;
byte cyanGain;
byte blueGain;
byte indigoGain;
byte violetGain;

int violetValue;
int indigoValue;
int blueValue;
int cyanValue;
int greenValue;
int yellowValue;
int orangeValue;
int redValue;

int sensorTimeLast;
int sensorTimeNow;

// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // Hz, powers of 2 are most reliable

int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin


void setup(){
  startMozzi(CONTROL_RATE); // :)
  Serial.begin(9600);
  redSin.setFreq(523); // set the frequency
  orangeSin.setFreq(587); // set the frequency
  yellowSin.setFreq(659); // set the frequency
  greenSin.setFreq(698); // set the frequency
  cyanSin.setFreq(784); // set the frequency
  blueSin.setFreq(880); // set the frequency
  indigoSin.setFreq(988); // set the frequency
  violetSin.setFreq(1046); // set the frequency

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  as7341.setLEDCurrent(50); // 4mA
  as7341.enableLED(true);
  sensorTimeLast = millis();
  sensorTimeNow = millis();

    while (!Serial) {
    delay(1);
  }
  
  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }

  as7341.setATIME(100);
  as7341.setASTEP(10);
  as7341.setGain(AS7341_GAIN_256X);
}




void updateControl(){
currentState = digitalRead(BUTTON_PIN);
  
  //sensorTimeNow = millis();
if (lastState == HIGH && currentState == LOW){
  readSensor();
  //sensorTimeLast = millis();
//  Serial.print("volume = ");Serial.println(volume);
  //Serial.print("redValue = ");Serial.println(redValue);
  //Serial.print("greenValue = ");Serial.println(greenValue);
  Serial.println(currentState);
  
}

  else if (lastState == LOW && currentState == HIGH)
    Serial.println("The button is released");

  // save the the last state
  lastState = currentState;

}


AudioOutput_t updateAudio(){
  long asig = (long)
    redSin.next() * redGain +
      orangeSin.next() * orangeGain +
      yellowSin.next() * yellowGain +
      greenSin.next() * greenGain +
      cyanSin.next() * cyanGain +
      blueSin.next() * blueGain +
      indigoSin.next() * indigoGain +
      violetSin.next() * violetGain;
  return MonoOutput::fromAlmostNBit(18, asig);

  
//return MonoOutput::from8Bit(((int))+((int))+((int))+((int))+((int))+((int))+((int))+((int)));
//((int)cyanSin.next() * cyanGain)++((int)indigoSin.next() * indigoGain)+((int)violetSin.next() * violetGain) // return an int signal centred around 0
}


void readSensor(){
    as7341.setLEDCurrent(50); // 4mA
    as7341.enableLED(true);
    if (!as7341.readAllChannels()){
    Serial.println("Error reading all channels!");
    return;
    
  
  
  // put changing controls in here


  violetValue = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
  indigoValue = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
  blueValue = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
  cyanValue = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
  greenValue = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
  yellowValue = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
  orangeValue = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
  redValue =as7341.getChannel(AS7341_CHANNEL_680nm_F8);
  }
  else{}
}

void loop(){
  audioHook(); // required here
 

  redGain =  map(redValue, 0, 1500, 0, 255);
  orangeGain =  map(orangeValue, 0, 1500, 0, 255);
  yellowGain =  map(yellowValue, 0, 1500, 0, 255);
  greenGain =  map(greenValue, 0, 1500, 0, 255);
  cyanGain =  map(cyanValue, 0, 1500, 0, 255);
  blueGain =  map(blueValue, 0, 1500, 0, 255);
  indigoGain =  map(indigoValue, 0, 1500, 0, 255);
  violetGain =  map(violetValue, 0, 1500, 0, 255);
}
