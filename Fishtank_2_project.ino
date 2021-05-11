//   File:  Fishtank_2_project
//
//  Test line
//
//   Make temperature readings with a TMP36 sensor and display
//   the readings on the OLED display along with
//   the time in minutes

// -- Libraries needed for the OLED display
#include <Wire.h>               //  Wire.h provides I2C support
#include <Adafruit_GFX.h>       //  Generic graphics library: fonts, lines, effects
#include <Adafruit_SSD1306.h>   //  Library for the micro OLED display

// -- Create an SSD1306 object called OLED that is connected by I2C
#define OLED_RESET      4  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH  128  // OLED display width in pixels
#define SCREEN_HEIGHT  32  // OLED display height in pixels
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//void setupOLED(); 
#define  AMAX  1023.0   //  Maximum analog reading, expressed as float
#define  VAMAX  3.6     //  Maximum voltage for ADC conversion of analog input

float tstart;           // set tstart as a global varible

  int TMP36pin = A0;                                    //  declare TMP36pin as a analog pin
  int Heaterpin = 11;
// ------------------------------------------------------------------------------------
void setup() {

  Serial.begin(9600);                                   // setup serial monitor
  while ( !Serial )  delay(10);

  setupOLED();                                          // setup the OLED display

  pinMode(Heaterpin, OUTPUT);

  tstart = millis();                                    // begin system clock in millis

  Serial.println("  Time    Temp   ");                  // Header for Serial Monitor or Plotte
  
}

// ------------------------------------------------------------------------------------
void loop() {

  float timeSeconds, timeMinutes;

  timeSeconds = (millis() - tstart) / 1000.0;             // convert milliseconds to seconds
  timeMinutes = timeSeconds / 60.0;                       // convert seconds to Minutes


  int TMP36pin = A0;                                    //  declare TMP36pin as a analog pin
  float alf = 0.05;                                      //  Set Alpha to a value
  float T, TexpAve;                                     //  declare tempature and averaged tempature
                                                        //  readings as a variable

  TexpAve = readExpAveTMP36(TMP36pin, alf);             // Tempature readings from TMP36


  turnonHeater(TexpAve);


  //-- print time and tempature reading to the serial monitor
  Serial.print("  ");
  Serial.print(timeMinutes);
  Serial.print("    ");
  Serial.println(TexpAve);
  //Serial.print("   ");
  //Serial.println (TBad);
  delay(200);

  updateOLED(timeMinutes, TexpAve);                  // update OLED display

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
void updateOLED(float timeMinutes, float TexpAve) {

  OLED.clearDisplay();              // Clear the buffer.

  OLED.setCursor(0, 0);             // (x,y) coords to start.  (0,0) is upper left
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Time = "));         // display "time =" as a constant
  OLED.print(timeMinutes);          // print time reading in minutes
  OLED.print(F(" min"));             // display "min" as a constant

  OLED.setCursor(0, 8);             // (x,y) coords to start.
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Temp = "));         // display "Temp" as a constant
  OLED.print(TexpAve);              // print tempature readings
  OLED.print(F(" C"));              // display "C" as a constant

  OLED.setCursor(0, 16);            // (x,y) coords to start.
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Threshold = "));    // display "Threshold" as a constant
  OLED.print(F("20C"));              // display "9C" as a constant

  OLED.setCursor(0, 24);            // (x,y) coords to start.
  OLED.setTextSize(1);              // Select font size
  OLED.print(F("Water Quality = "));         // display "TBad" as a constant
  //OLED.print(TBad);                 // display time when temp passes threshold
  OLED.print(F(" min"));             // display "min" as a constant

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

  float setpoint = 6;                        // Declare relative humidity setpoint 
  float deadband = 1.0;                         // Declare deadband percentage


  if (TexpAve < setpoint - deadband) {             // Test Level below lower deadpoint control limit
    digitalWrite(Heaterpin, LOW);                // Turn fan off 
    delay(1000);


  } else if (TexpAve > setpoint + deadband ) {      // Test Level above upper deadpoint control limit
    digitalWrite(Heaterpin, HIGH);                // Turn fan on 
    delay(1000);
  }
  
}
