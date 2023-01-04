/*  An organ that plays notes according to input
    from an AS7341 color sensor using Mozzi 
    sonification library.
    Tim Barrass 2012, CC by-nc-sa.
    Emily Velasco 2022, CC by-nc-sa.
*/

#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
/*sine table for oscillators. choose others from mozzi 
if triangle wave, square wave, etc, desired*/
#include <tables/sin2048_int8.h> 
#include <tables/cos2048_int8.h>

#include <Adafruit_AS7341.h>
#include <mozzi_midi.h>
int sineOutput;
int j;

Adafruit_AS7341 as7341;

//an array for holding color values from the sensor
int colorValues[8];

int red = 0;
int orange = 1;
int yellow = 2;
int green = 3;
int cyan = 4;
int blue = 5;
int indigo = 6;
int violet = 7;

//variables used for determining dominant color
int theMax, current, maxI;


float depth = 0.1;
float red_centre_freq = 523.25;
float orange_centre_freq = 587.33;
float yellow_centre_freq = 659.26;
float green_centre_freq = 698.46;
float cyan_centre_freq = 783.99;
float blue_centre_freq = 880.00;
float indigo_centre_freq = 987.77;
float violet_centre_freq = 1046.50;

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> redSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> orangeSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> yellowSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> greenSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> cyanSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> blueSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> indigoSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> violetSin(SIN2048_DATA);

Oscil <2048, CONTROL_RATE> kVib(SIN2048_DATA);
// a line to interpolate control tremolo at audio rate


/*gain variables for volume level of notes. Mozzi uses
0-255 to get volume levels, thus byte is used*/
byte redGain;
byte orangeGain;
byte yellowGain;
byte greenGain;
byte cyanGain;
byte blueGain;
byte indigoGain;
byte violetGain;

// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // Hz, powers of 2 are most reliable


void setup() {
  startMozzi(CONTROL_RATE); // :)
    Serial.begin(9600);
  
  redSin.setFreq(mtof(72)); // set the frequency
  orangeSin.setFreq(mtof(74)); // set the frequency
  yellowSin.setFreq(mtof(76)); // set the frequency
  greenSin.setFreq(mtof(77)); // set the frequency
  cyanSin.setFreq(mtof(79)); // set the frequency
  blueSin.setFreq(mtof(81)); // set the frequency
  indigoSin.setFreq(mtof(83)); // set the frequency
  violetSin.setFreq(mtof(84)); // set the frequency
  
  kVib.setFreq(12.5f);
  
  while (!Serial) {
    delay(1);
  }

  if (!as7341.begin()) {
    //   Serial.println("Could not find AS7341");
    while (1) {
      delay(10);
    }
  }
 /*the smaller these values are, the quicker the 
 sensor returns values, but the bigger they are, the
 better the readings are. striking a balance between
 sensitivity and speed here*/
  as7341.setATIME(100);
  as7341.setASTEP(10);
  as7341.setGain(AS7341_GAIN_256X);

  // Start sensor integration.
  as7341.startReading();
}




void updateControl() {

  colorGains();

  float vibrato = depth * kVib.next();


  redSin.setFreq(red_centre_freq+vibrato); // set the frequency
  orangeSin.setFreq(orange_centre_freq+vibrato); // set the frequency
  yellowSin.setFreq(yellow_centre_freq+vibrato); // set the frequency
  greenSin.setFreq(green_centre_freq+vibrato); // set the frequency
  cyanSin.setFreq(cyan_centre_freq+vibrato); // set the frequency
  blueSin.setFreq(blue_centre_freq+vibrato); // set the frequency
  indigoSin.setFreq(indigo_centre_freq+vibrato); // set the frequency
  violetSin.setFreq(violet_centre_freq+vibrato); // set the frequency
  
  if (as7341.checkReadingProgress()) {
    // Sensor integration complete, read and process sensor values.
    readSensor();

    // Start another round of sensor integration.
    as7341.startReading();
  }

}


AudioOutput_t updateAudio() {
  //sums all oscillators into a single audio output





  
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
//Serial.println(redGain*aGain.next());

 
}


void readSensor() {
  uint16_t readings[12];

  as7341.getAllChannels(readings);







//each time it loops, reset these for clean slate to compare from
      theMax = 0;
      current = 0;
/*read the color channels from the sensor and write them into 
the colorValues array for later use*/
    colorValues[violet] = readings[AS7341_CHANNEL_415nm_F1];
    colorValues[indigo] = readings[AS7341_CHANNEL_445nm_F2];
    colorValues[blue] = readings[AS7341_CHANNEL_480nm_F3];
    colorValues[cyan] = readings[AS7341_CHANNEL_515nm_F4];
    colorValues[green] = readings[AS7341_CHANNEL_555nm_F5];
    colorValues[yellow] = readings[AS7341_CHANNEL_590nm_F6];
    colorValues[red] = readings[AS7341_CHANNEL_630nm_F7];
    colorValues[orange] = readings[AS7341_CHANNEL_680nm_F8];

  for (byte i = 0; i < 8; i = i + 1) {

    //compare all the color values to find which is greatest and log its index
    current = colorValues[i];
    if (current > theMax) {
      theMax = current;
      maxI = i;
    }


  }
}
//turns volume up for note associate with dominant color; mutes all others
void colorGains() {
    redGain =  0;
    orangeGain =  0;
    yellowGain =  0;
    greenGain =  0;
    cyanGain =  0;
    blueGain =  0;
    indigoGain =  0;
    violetGain =  0;
    
  if (maxI == red) {
    redGain =  200;
  }
  
  else if (maxI == orange) {
    orangeGain =  200;
  }
  
  else if (maxI == yellow) {
    yellowGain =  200;
  }
  
  else if (maxI == green) {
    greenGain =  200;
  }
  
  else if (maxI == cyan) {
    cyanGain =  200;
  }
  else if (maxI == blue) {
    blueGain =  200;
  }
  else if (maxI == indigo) {
    indigoGain =  200;
  }
  
  else if (maxI == violet) {
    violetGain =  200;
  }
}
void loop() {
  audioHook(); // required here

}
