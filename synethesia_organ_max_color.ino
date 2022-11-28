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
#include <Adafruit_AS7341.h>
#include <mozzi_midi.h>


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

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> redSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> orangeSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> yellowSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> greenSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> cyanSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> blueSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> indigoSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> violetSin(SIN2048_DATA);


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

//timing variables. mozzi doesn't like delay()
int sensorTimeLast;
int sensorTimeNow;

// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // Hz, powers of 2 are most reliable


void setup() {
  startMozzi(CONTROL_RATE); // :)
  //  Serial.begin(9600);
  
  redSin.setFreq(mtof(72)); // set the frequency
  orangeSin.setFreq(mtof(74)); // set the frequency
  yellowSin.setFreq(mtof(76)); // set the frequency
  greenSin.setFreq(mtof(77)); // set the frequency
  cyanSin.setFreq(mtof(79)); // set the frequency
  blueSin.setFreq(mtof(81)); // set the frequency
  indigoSin.setFreq(mtof(83)); // set the frequency
  violetSin.setFreq(mtof(84)); // set the frequency

  sensorTimeLast = millis();
  sensorTimeNow = millis();

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
}




void updateControl() {
  sensorTimeNow = millis();
  colorGains();
  /*communicates with the sensor ten times a second
  because I2C comms interrupts audio synthesis, this
  is another balancing act. each communication over I2C
  creates an audio pop. too often sounds like buzzing.
  too infrequent means code is slow to take sensor readings*/
  if (sensorTimeNow >= (sensorTimeLast + 100)) {
    readSensor();
    sensorTimeLast = millis();

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


 
}


void readSensor() {
  if (!as7341.readAllChannels()) {
    //   Serial.println("Error reading all channels!");
    return;
  }

  // put changing controls in here

/*read the color channels from the sensor and write them into 
the colorValues array for later use*/
  for (byte i = 0; i < 8; i = i + 1) {
    colorValues[violet] = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
    colorValues[indigo] = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
    colorValues[blue] = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
    colorValues[cyan] = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
    colorValues[green] = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
    colorValues[yellow] = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
    colorValues[red] = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
    colorValues[orange] = as7341.getChannel(AS7341_CHANNEL_680nm_F8);

//compare all the color values to find which is greatest and log its index
    current = colorValues[i];
    if (current > theMax) {
      theMax = current;
      maxI = i;
    }

//after each time it loops, reset these for clean slate to compare from
    if (i >= 7) {
      theMax = 0;
      current = 0;
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
    
  if (maxI == 0) {
    redGain =  200;
  }
  
  else if (maxI == 1) {
    orangeGain =  200;
  }
  
  else if (maxI == 2) {
    yellowGain =  200;
  }
  
  else if (maxI == 3) {
    greenGain =  200;
  }
  
  else if (maxI == 4) {
    cyanGain =  200;
  }
  else if (maxI == 5) {
    blueGain =  200;
  }
  else if (maxI == 6) {
    indigoGain =  200;
  }
  
  else if (maxI == 7) {
    violetGain =  200;
  }
}
void loop() {
  audioHook(); // required here

}

