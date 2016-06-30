/*
 BeatBag - A Speed Bag Counter
 Nathan Seidle
 SparkFun Electronics
 2/23/2013
 Modified by Todd Berg
 6/19/2016, 6/26/2016

 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 BeatBag is a speed bag counter that uses an accelerometer to counts the number hits. 
 It's easily installed ontop of speed bag platform only needing an accelerometer attached to the top of platform. 
 You don't have to alter the hitting surface or change out the swivel.

x -> abs -\
y -> abs --> sum -> hpf -> abs -> lpf -> envelope -> slope -> slope thresh -> one shot
z -> abs -/

Higher accuracy with less CPU loading

Read signed raw counts of all three axes from accelerometer
  No need to convert to g's

Absolute value of raw counts of all three axes, compute sum
  No need to calculate true magnitude (square root of sum of squares)
  Turns "ringing" of signal into a positive response (full wave rectification)
  Sum after absolute value ensures that combining axes does not interfere destructively

True high pass filter
  Removes DC component, allows specific low cutoff frequency
  Absolute value of high pass filter to convert ringing into positive envelope

True low pass filter
  Reduces high frequency noise to improve envelope detection (optional)

Envelope detector
  Fast attack, slow decay to respond to impacts and greatly reduce noise
  decay can be adjusted

Slope of envelope and comparison to threshold
  If slope is greater than threshold then a hit has been detected

One shot of detected hit
  Reduces false triggers by keeping the time between hits realistic

Hit counter
  Increments based on one shot event
  cleared manually by user

Variables:
  high pass cutoff frequency
  low pass cutoff frequency
  envelope decay
  hit detection threshold
  one shot time


 Hardware setup:
 5V from wall supply goes into barrel jack on Redboard. Trace cut to diode.
 RedBoard barel jack is wired to power switch then to Vin diode
 Display gets power from Vin and data from I2C pins
 Vcc/Gnd from RedBoard goes into Bread Board Power supply that supplies 3.3V to accelerometer. Future
 versions should get power from 3.3V rail on RedBoard. 

 MMA8452 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA(yellow) -------^^(330)^^------- A4
 SCL(blue) -------^^(330)^^------- A5
 GND ---------------------- GND
 The MMA8452 is 3.3V so we recommend using 330 or 1k resistors between a 5V Arduino and the MMA8452 breakout.
 The MMA8452 has built in pull-up resistors for I2C so you do not need additional pull-ups.

 3/2/2013 - Got data from Hugo and myself, 3 rounds, on 2g setting. Very noisy but mostly worked

 12/19/15 - Segment burned out. Power down display after 10 minutes of non-use.
 Use I2C, see if we can avoid the 'multiply by 10' display problem.

 1/23/16 - Accel not reliable. Because the display is now also on the I2C the pull-up resistors on the accel where
 not enough. Swapped out to new accel. Added 100 ohm inline resistors to accel and 4.7k resistors from SDA/SCL to 5V.
 Reinforced connection from accel to RedBoard.

 6/26/16 - TBB - Redesigned hit detection algorithm

 */

#include <avr/wdt.h> //We need watch dog for this program
#include <Wire.h> // Used for I2C

#define DISPLAY_ADDRESS 0x71 //I2C address of OpenSegment display

int hitCounter = 0; //Keeps track of the number of hits

const int resetButton = 6; //Button that resets the display and counter
const int LED = 13; //Status LED on D3

long lastPrint; //Used for printing updates every second

boolean displayOn; //Used to track if display is turned off or not

//Used in the new algorithm
float lastMagnitude = 0;
float lastFirstPass = 0;
float lastSecondPass = 0;
float lastThirdPass = 0;
long lastHitTime = 0;
int secondsCounter = 0;

//This is the number of miliseconds before we turn off the display
long TIME_TO_DISPLAY_OFF = 60L * 1000L * 5L; //5 minutes of no use

int DEFAULT_BRIGHTNESS = 50; //50% brightness to avoid burning out segments after 3 years of use

unsigned long currentTime; //Used for millis checking

/*
Todd Berg's algorithm:
x -> abs -\
y -> abs --> sum -> hpf -> abs -> lpf -> envelope -> slope -> slope thresh -> one shot
z -> abs -/
*/

#define hpftau 32             // high pass cutoff = 500 / 32 = 16 Hz
#define lpftau 1              // low pass cutoff = 500 / 1 = 500 Hz
#define envelope_decay 256    // higher number = slower decay
#define hit_threshold 100     // slope of attack which qualifies as a hit
#define oneshot_time 50		    // minimum time between hits = 0.1 seconds (50 / 500 samp/sec = 0.1)
#define hpfdelay hpftau / 2   // compensates for lag, must be half of hpftau

//unsigned int hpfdelay = hpftau / 2;  // compensates for lag
unsigned int sumxyz_delay_read_idx = 0;
unsigned int sumxyz_delay_write_idx = sumxyz_delay_read_idx + hpfdelay - 1;
unsigned int sumxyz_delay_idx_max = hpfdelay;
unsigned int oneshot_timer = 0;
int accelCount[3];
unsigned int absx;
unsigned int absy;
unsigned int absz;
unsigned int sumxyz;
unsigned int sumxyz_delayed[hpfdelay];
int hpf1;
int hpf;
unsigned int hpfabs;
int lpf;
unsigned int envelope;
unsigned int envelope_previous;
int slope;
unsigned int hit;
unsigned int oneshot;
unsigned int previous_oneshot;
//unsigned int oneshot_timer;
long LastOneshotTime = 0;
unsigned int sample_time;

void setup()
{
  wdt_reset(); //Pet the dog
  wdt_disable(); //We don't want the watchdog during init

  pinMode(resetButton, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  //By default .begin() will set I2C SCL to Standard Speed mode of 100kHz
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  Wire.begin(); //Join the bus as a master

  Serial.begin(115200);
  Serial.println("Speed Bag Counter");

  initDisplay();

  clearDisplay();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.print("Accl"); //Display an error until accel comes online
  Wire.endTransmission();

//  while(!initMMA8452()) //Test and intialize the MMA8452
//    ; //Do nothing

  clearDisplay();
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.print("0000");
  Wire.endTransmission();

  lastPrint = millis();
  lastHitTime = millis();

  sumxyz_delay_read_idx = 0;
  sumxyz_delay_write_idx = sumxyz_delay_read_idx + hpfdelay - 1;
  sumxyz_delay_idx_max = hpfdelay;
  oneshot_timer = 0;
  hitCounter = 0;

  wdt_enable(WDTO_250MS); //Unleash the beast
}

void loop()
{
  wdt_reset(); //Pet the dog

  currentTime = millis();
  if ((unsigned long)(currentTime - lastPrint) >= 1000)
  {
    if (digitalRead(LED) == LOW)
      digitalWrite(LED, HIGH);
    else
      digitalWrite(LED, LOW);

    lastPrint = millis();
  }

  //See if we should power down the display due to inactivity
  if (displayOn == true)
  {
    currentTime = millis();
    if ((unsigned long)(currentTime - lastHitTime) >= TIME_TO_DISPLAY_OFF)
    {
      Serial.println("Power save");

      hitCounter = 0; //Reset the count

      clearDisplay(); //Clear to save power
      displayOn = false;
    }
  }

  //Check the accelerometer
//  float currentMagnitude = getAccelData();
//	readAccelData(accelCount); // Read the x/y/z adc values
	
	//Print the readings for logging to an OpenLog
	Serial.print(millis());
	Serial.print(",");
	Serial.print(accelCount[0]);
	Serial.print(",");
	Serial.print(accelCount[1]);
	Serial.print(",");
	Serial.println(accelCount[2]);

// calculate poor man's magnitude
// sum of absolute values of x, y, and z axes

	absx = abs(accelCount[0]);
	absy = abs(accelCount[1]);
	absz = abs(accelCount[2]);

	sumxyz = absx + absy + absz;

// store sum in fifo - will delay values to compensate for filter lag

	sumxyz_delayed[sumxyz_delay_write_idx++] = sumxyz;
	if (sumxyz_delay_write_idx == sumxyz_delay_idx_max) sumxyz_delay_write_idx = 0;

// high pass filter

	hpf1 = hpf1 - hpf1 / hpftau + sumxyz / hpftau;  // low pass component of high pass filter
	hpf = hpf1 - sumxyz_delayed[sumxyz_delay_read_idx++]; // subtract delayed input
	if (sumxyz_delay_read_idx == sumxyz_delay_idx_max) sumxyz_delay_read_idx = 0;

// absolute value (full wave rectification)

	hpfabs = abs(hpf);

// low pass filter

  lpf = lpf - lpf / lpftau + hpfabs / lpftau;

// fast attack / slow decay envelope detector

	if (lpf > envelope)
	{
	  envelope = lpf; // fast attack
	}
	else
	{
	  envelope = envelope - envelope / envelope_decay; // slow decay
	}

// two sample slope of envelope

	slope = envelope - envelope_previous;
  envelope_previous = envelope;

// hit detector (slope threshold)

	if (slope > hit_threshold)
	{
	  hit = 1;
	}
	else
	{
	  hit = 0;
	}

// one shot

	previous_oneshot = oneshot;
	if ((oneshot_timer == 0) and (hit == 1))
	{
	  oneshot = 1;
	  oneshot_timer = oneshot_time; // start timer
	  hitCounter++;
	}
	else
	{
	  oneshot = 0;
	  if (oneshot_timer > 0)
	  {
      currentTime = millis();
      if ((unsigned long)(currentTime - LastOneshotTime) >= 2) // every 2 milliseconds
      {
        LastOneshotTime = currentTime;
        oneshot_timer = oneshot_timer - 1;
      }
	  }
	}

//	if (hitcount_reset == 1) hitcount = 0;

	if (oneshot)
	{
        //Serial.print("Hit: ");
        //Serial.println(hitCounter);

        if (displayOn == false) displayOn = true;

        printHits(); //Updates the display
	}

  //Check if we need to reset the counter and display
  if (digitalRead(resetButton) == LOW)
  {
    //This breaks the file up so we can see where we hit the reset button
    Serial.println();
    Serial.println();
    Serial.println("Reset!");
    Serial.println();
    Serial.println();

    hitCounter = 0;

    resetDisplay(); //Forces cursor to beginning of display
    printHits(); //Updates the display

    while (digitalRead(resetButton) == LOW) wdt_reset(); //Pet the dog while we wait for you to remove finger

    //Do nothing for 250ms after you press the button, a sort of debounce
    for (int x = 0 ; x < 25 ; x++)
    {
      wdt_reset(); //Pet the dog
      delay(10);
    }
  }
}

// SUBROUTINES

//This function makes sure the display is at 57600
void initDisplay()
{
  resetDisplay(); //Forces cursor to beginning of display

  printHits(); //Update display with current hit count

  displayOn = true;

  setBrightness(DEFAULT_BRIGHTNESS);
}

//Set brightness of display
void setBrightness(int brightness)
{
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(0x7A); // Brightness control command
  Wire.write(brightness); // Set brightness level: 0% to 100%
  Wire.endTransmission();
}

void resetDisplay()
{
  //Send the reset command to the display - this forces the cursor to return to the beginning of the display
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write('v');
  Wire.endTransmission();

  if (displayOn == false)
  {
    setBrightness(DEFAULT_BRIGHTNESS); //Power up display
    displayOn = true;
    lastHitTime = millis();
  }
}

//Push the current hit counter to the display
void printHits()
{
//  int tempCounter = hitCounter / 2; //Cut in half
  int tempCounter = hitCounter;

  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(0x79); //Move cursor
  Wire.write(4); //To right most position

  Wire.write(tempCounter / 1000); //Send the left most digit
  tempCounter %= 1000; //Now remove the left most digit from the number we want to display
  Wire.write(tempCounter / 100);
  tempCounter %= 100;
  Wire.write(tempCounter / 10);
  tempCounter %= 10;
  Wire.write(tempCounter); //Send the right most digit

  Wire.endTransmission(); //Stop I2C transmission
}

//Clear display to save power (a screen saver of sorts)
void clearDisplay()
{
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write(0x79); //Move cursor
  Wire.write(4); //To right most position

  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');
  Wire.write(' ');

  Wire.endTransmission(); //Stop I2C transmission
}
