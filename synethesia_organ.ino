#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <Adafruit_AS7341.h> //sensor library


Adafruit_AS7341 as7341;

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
//sine wave table for each color channel
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> redSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> orangeSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> yellowSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> greenSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> cyanSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> blueSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> indigoSin(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> violetSin(SIN2048_DATA);


//these are the gain (volume) for each channel
byte redGain;
byte orangeGain;
byte yellowGain;
byte greenGain;
byte cyanGain;
byte blueGain;
byte indigoGain;
byte violetGain;

//these store the raw value for each channel straight from the sensor
int violetValue;
int indigoValue;
int blueValue;
int cyanValue;
int greenValue;
int yellowValue;
int orangeValue;
int redValue;

//for non-blocking timer (can't use delays with mozzi)
int sensorTimeLast;
int sensorTimeNow;

// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // Hz, powers of 2 are most reliable


void setup(){
  startMozzi(CONTROL_RATE); // :)
  Serial.begin(9600);
  //these frequencies correspond to actual notes
  redSin.setFreq(523); // set the frequency
  orangeSin.setFreq(587); // set the frequency
  yellowSin.setFreq(659); // set the frequency
  greenSin.setFreq(698); // set the frequency
  cyanSin.setFreq(784); // set the frequency
  blueSin.setFreq(880); // set the frequency
  indigoSin.setFreq(988); // set the frequency
  violetSin.setFreq(1046); // set the frequency
  
  /*
  as7341.setLEDCurrent(50); // 4mA
  as7341.enableLED(true);
  */
  
  sensorTimeLast = millis();
  sensorTimeNow = millis();

    while (!Serial) {
    delay(1);
  }
  
  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }

  //keep these values on the low side to minimize time spent waiting for sensor to read
  as7341.setATIME(100);
  as7341.setASTEP(10);
  as7341.setGain(AS7341_GAIN_256X);
}




void updateControl(){
  sensorTimeNow = millis();
  //every 100 milliseconds, read the values from the sensor
if (sensorTimeNow>=(sensorTimeLast+100)){
  readSensor();
  sensorTimeLast = millis();
//  Serial.print("volume = ");Serial.println(volume);
  //Serial.print("redValue = ");Serial.println(redValue);
  //Serial.print("greenValue = ");Serial.println(greenValue);
  
}

}


AudioOutput_t updateAudio(){

//adds all the sound waves together to create one sound output
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
    if (!as7341.readAllChannels()){
    Serial.println("Error reading all channels!");
    return;
  }
  
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

void loop(){
  audioHook(); // required here
 
/*maps the potentially large values from the sensor
to a range of 0-255, which is what mozzi uses for
setting the volume of an individual tone*/

  redGain =  map(redValue, 0, 1500, 0, 255);
  orangeGain =  map(orangeValue, 0, 1500, 0, 255);
  yellowGain =  map(yellowValue, 0, 1500, 0, 255);
  greenGain =  map(greenValue, 0, 1500, 0, 255);
  cyanGain =  map(cyanValue, 0, 1500, 0, 255);
  blueGain =  map(blueValue, 0, 1500, 0, 255);
  indigoGain =  map(indigoValue, 0, 1500, 0, 255);
  violetGain =  map(violetValue, 0, 1500, 0, 255);
}
