//   File:  Fishtank_2_project
//
//  Alex Bekwinknoll, Shaho Meman, Jason Chen, Eric Ovcharenko, Robert Hestand, Connor Hutchinson
//
//   Make temperature readings and water qulaity readings with a TMP36 sensor and
//   a photoresistor to determine when the heater and filter should turn on or off
//   then display the readings on the OLED display along with the time in minutes
//   in which the automatic feeder should engage

// -- Libraries needed for the OLED display
#include <Wire.h>               //  Wire.h provides I2C support
#include <Adafruit_GFX.h>       //  Generic graphics library: fonts, lines, effects
#include <Adafruit_SSD1306.h>   //  Library for the micro OLED display
#include <Servo.h>              //  Libary for the servo

//LED Set-Up
int WaterHeaterLED = 5;
int WaterFilterLED = 6;
int ServoLED = 13;

Servo feederServo;  // create servo object called feederServo that controls the servo on the feeder
unsigned int feedCount = 0;

// -- Create an SSD1306 object called OLED that is connected by I2C
#define OLED_RESET      4  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH  128  // OLED display width in pixels
#define SCREEN_HEIGHT  32  // OLED display height in pixels
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//void setupOLED();
#define  AMAX  1023.0   //  Maximum analog reading, expressed as float
#define  VAMAX  3.6     //  Maximum voltage for ADC conversion of analog input

float tstart;           // set tstart as a global varible

const int TMP36pin = A0;                                    //  declare TMP36pin as a analog pin
const int Heaterpin = 11;                                   //  declare heater digital output
const int Filterpin = 12;                                   //  declare heater digital output
bool heater;    //keeps track if heater is on or off.

const int pinCapPr = A1;
const int pinSidePr = A3;       //pins of the photoresisters on the cap and side respectivly.
const int turbidityLedPin = 6;  //pin fot the LED in the turbidity sensor
// ------------------------------------------------------------------------------------
void setup() {
  
 digitalWrite(WaterHeaterLED,LOW);  //turn off all LEDs
 digitalWrite(WaterFilterLED,LOW);
 digitalWrite(ServoLED,LOW);

  Serial.begin(9600);                                   // setup serial monitor
  // while ( !Serial )  delay(10);

  setupOLED();                                          // setup the OLED display

  pinMode(Heaterpin, OUTPUT);                           // declare pin as an output
  pinMode(Filterpin, OUTPUT);                           // declare pin as an output

  //LED Code
  pinMode(WaterHeaterLED, OUTPUT);       // declare pin as an output
  pinMode(WaterFilterLED, OUTPUT);    // declare pin as an output
  pinMode(ServoLED, OUTPUT);     // declare pin as an output

  feederServo.attach(9);  // attaches the feederServo on pin 9 to the servo object

  pinMode(INPUT, pinCapPr);  //declares pins used in turbidity sensor
  pinMode(INPUT, pinSidePr);
  pinMode(OUTPUT, turbidityLedPin);

  analogWrite(turbidityLedPin, 255);  //turn on the turbidity sensor led

  tstart = millis();                                    // begin system clock in millis

  Serial.println("  Time    Temp   Water Quality  ");                  // Header for Serial Monitor or Plotte

}

// ------------------------------------------------------------------------------------
void loop() {

  ufloat timeSeconds, timeMinutes;

  timeSeconds = (millis() - tstart) / 1000.0;             // convert milliseconds to seconds
  timeMinutes = timeSeconds / 60.0;                       // convert seconds to Minutes


  int TMP36pin = A0;                                    //  declare TMP36pin as a analog pin
  float alf = 0.05;                                     //  Set Alpha to a value
  float T, TexpAve, turExpAve;                          //  declare tempature and averaged tempature
  //  readings as a variable

  TexpAve = readExpAveTMP36(TMP36pin, alf);             // Tempature readings from TMP36
  turExpAve = turbidity();                              //turbidity reading from sensor

  turnonHeater(TexpAve);                                // Turn on Heater
  turnonFilter(turExpAve);                              // Turn on Filter

  if( timeMinutes >= (15 + 720 * feedCount ) )
     {
      feedCount ++;
      dispenseFood();
     }

  //-- print time and tempature reading to the serial monitor
  Serial.print("  ");
  Serial.print(timeMinutes);
  Serial.print("    ");
  Serial.print(TexpAve);
  Serial.print("   ");
  Serial.println (turExpAve);
  delay(200);

  updateOLED(timeMinutes, TexpAve, turExpAve);                  // update OLED display

}

// ------------------------------------------------------------------------------------
//  Set the I2C address and internal voltage supply for the 128x32 OLED display.
//  If configuration is successful, display the splash screen and another message.
//  These steps are only needed once at the start of a sketch, and presume the
//  existence of a global Adafruit_SSD1306 object called OLED.
//
void setupOLED() {

  // -- Set up OLED display.  Use internal 3.3v supply, and Adafruit trick
  //      SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //    I2C address is 0x3C for the 128x32 display
  if ( !OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C) ) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true) ;   // Don't proceed, loop forever
  }

  // -- Show Adafruit splash screen stored in image buffer.
  Serial.println("Monochrome OLED is available");
  OLED.display();
  delay(1000);         //  Pause to allow user to read the display

  // -- Clear the display buffer show a blank screen
  OLED.clearDisplay();
  OLED.display();

  // -- Set default text mode and display a "ready" message.
  OLED.setTextSize(1);
  OLED.setTextColor(SSD1306_WHITE);
  OLED.print(F("OLED is ready"));
  OLED.display();
  delay(1000);                         //  Pause to allow user to read the display
}

// ------------------------------------------------------------------------------------
//  Display values of time in decimal minute and tempature readings on the micro OLED.
//  This function assumes that OLED is a global Adafruit_SSD103 object.
//
void updateOLED(float timeMinutes, float TexpAve, float turExpAve) {

  OLED.clearDisplay();              // Clear the buffer.

  OLED.setCursor(0, 0);             // (x,y) coords to start.
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Temp = "));         // display "Temp" as a constant
  OLED.print(TexpAve);              // print tempature readings
  OLED.print(F(" C"));              // display "C" as a constant

  OLED.setCursor(0, 8);            // (x,y) coords to start.
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Threshold = "));    // display "Threshold" as a constant
  OLED.print(F("16C"));              // display "9C" as a constant

  OLED.setCursor(0, 24);            // (x,y) coords to start.
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Water Quality = "));         // display "TBad" as a constant
  OLED.print(turExpAve);                 // display time when temp passes threshold
  OLED.print(F(" unit"));             // display "min" as a constant
  
  OLED.setCursor(0, 16);             // (x,y) coords to start.  (0,0) is upper left
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Heater: "));         // display "Heater: " as a constant
  if (heater == 1)
  {
    OLED.print(F("on"));      // show heater is 'on' on oled
  }
  else if(heater == 0)
  {
    OLED.print(F("off"));    // show heater is 'off' on oled
  }

  OLED.display();                   // Update the display
}

// ---------------------------------------------------------------
//  Exponential moving average of TMP36 sensor readings
//  Return temperature in C
//
float readExpAveTMP36(int sensorPin, float alfa)  {

  int reading;
  float TCave, TC, voltage;
  static float aveOld = 0.0;    // Static variable to remember last reading

  reading = analogRead(sensorPin);        // Sacrificial reading
  delay(10);                              // Let ADC settle
  reading = analogRead(sensorPin);        // Keep this reading

  voltage = (VAMAX / AMAX) * float(reading); // Scale 10-bit value to voltage
  TC = (voltage - 0.5) * 100.0;           // 10 mV/C with 500 mV offset

  TCave = aveOld + alfa * (TC - aveOld);  // Update the moving average
  aveOld = TCave;                         // Save for next reading
  return (TCave);
}

// ---------------------------------------------------------------
//  Turn on Heater depending on if threshold
//  has been reached
//
void turnonHeater(float TexpAve)  {

  float setpoint = 16.0;                        // Declare temperature setpoint
  float deadband = 0.5;                         // Declare deadband percentage


  if (TexpAve < setpoint - deadband)   // Test Level below lower deadpoint control limit
  {
    digitalWrite(Heaterpin, HIGH);              // Turn heater on
    digitalWrite(WaterHeaterLED, HIGH);         // Turn water heater LED on
    heater = 1;
  }

  else if (TexpAve > setpoint + deadband ) // Test Level above upper deadpoint control limit
  {
    digitalWrite(Heaterpin, LOW);                // Turn heater off
    digitalWrite(WaterHeaterLED, LOW);           // Turn water heater LED off
  heater = 0;
  }

}
// ---------------------------------------------------------------
//  Turn on filter depending on if threshold
//  has been reached
//
void turnonFilter(float turExpAve )  {

  float setpoint = 0.13;                        // Declare water quality setpoint
  float deadband = 0.1;                         // Declare deadband percentage


  if (turExpAve > setpoint + deadband)
  { // Test Level below lower deadpoint control limit
    digitalWrite(Filterpin, HIGH);                // Turn filter on
    digitalWrite(WaterFilterLED, HIGH);           // Turn water filter LED on
  }
  if (turExpAve < setpoint - deadband ) // Test Level above lower deadpoint control limit
  {
    digitalWrite(Filterpin, LOW);                // Turn filter off
    digitalWrite(WaterFilterLED, LOW);           // Turn water filter LED off
  }

}

//------------------------------------------------------------------------
// function to dispenses food to the fish when called upon

void dispenseFood()
{
  int pos1 = 5;    // position 1 is the position for the feeder to put food into the dispensing mechanism
  int pos2 = 175;    // position 2 drops the food from the dispensing mechanism into the tank

  for (int num1 = pos2; num1 >= pos1; num1 = num1 - 1) {      // tell servo to go to position in variable 'pos1'
    // in steps of 1 degree
    feederServo.write(num1);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  delay(1000);                         // waits 1 second when at position 1

  for (int num2 = pos1; num2 <= pos2; num2 ++) {   // tell servo to go to position in variable 'pos2'
    feederServo.write(num2);              // tell servo to go to position in variable 'num1'
    digitalWrite(ServoLED, HIGH);           // Turn "Fisher Feeder" LED on
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  digitalWrite(ServoLED, LOW);           // Turn "fish feeder" LED off
  
  // tell servo to go to position in variable 'pos2'
  delay(1500);                         // waits 1.5 seconds after reaching position 2


} 

//---------------------------------------------------------------------------------------------
// function that reports back an exponentaly averaged turbidity

float turbidity()
{

  float alfa = 0.2;
  float ratio;
  static float aveCapPr = 0, aveSidePr = 0;

  for (int num1 = 1; num1 <= 3; num1 ++)
  {
    analogRead(pinCapPr);  // waste reading to prevent noise in data
    aveCapPr = aveCapPr + alfa * (analogRead(pinCapPr) - aveCapPr); // expoental averaging

    analogRead(pinSidePr);  // waste reading to prevent noise in data
    aveSidePr = aveSidePr + alfa * (analogRead(pinSidePr) - aveSidePr);
  }

  ratio = (aveSidePr / aveCapPr);  //calculating the ratio between the side and endcap readings
  return (ratio);
}
