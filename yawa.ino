#include "DHT.h"

#include 
Servo myservo; 
int pinDHT11 = 2;
SimpleDHT11 dht11; 
int redLed = 12;
int greenLed = 11;
int buzzer = 10;
int smokeA0 = A5;
int sensorThres = 400;
#include <Adafruit_MAX31865.h>          
#include <MCP41_Simple.h>               
#include <Adafruit_RGBLCDShield.h>      
#include <utility/Adafruit_MCP23017.h> 
#include <Wire.h>


MCP41_Simple digitalPotentiometer; 
const uint8_t digitalPotentiometer_CS = 10;


Adafruit_MAX31865 PT100amplifier = Adafruit_MAX31865(2, 3, 4, 5);
#define RREF      430.0
#define RNOMINAL  100.0


Adafruit_RGBLCDShield LCD_shield = Adafruit_RGBLCDShield();
 
void setup() { 
myservo.attach(5);
 Serial.begin(115200);
 pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);
  Serial.begin(9600);

  Serial.begin(115200);
  Serial.println("Starting operation...");
  PT100amplifier.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary, 4-wire RTD in this case

  // Initialise digital potentiometer
  digitalPotentiometer.begin(digitalPotentiometer_CS);
  // Set the wiper to an arbitrary point between 0 and 255
  digitalPotentiometer.setWiper( 200 );

  // Initialise LCD display shield
  // set up the LCD's number of columns and rows: 
  LCD_shield.begin(16, 2);
  // set up setpoint and measured T text on LCD with the correct spacings
  LCD_shield.print("Tsetpoint:   C");
  LCD_shield.setCursor(0, 1);
  LCD_shield.print("Tsample:     C");
}

// Initialise PID constants, temperature related variables and shield button value
int powerMode = 1; 
int operationMode = 1;  
float PT100ratio;   
uint8_t buttonsPressed = 0;
float kp = 500.0;  
float PID_p = 0.0;  
float Tmeasured = -1.0;
float Tsetpoint = 22.0;  
float PID_error = 5;
float PID_value = 0;

// Define print_Tsetpoint function to correctly print the temperature setpoint on the LCD screen
static char TsetpointString[3];
void print_Tsetpoint(int T) {
  // Print Tsetpoint in the correct place 
  LCD_shield.setCursor(10, 0);
  dtostrf(T, 3, 0, TsetpointString);
  LCD_shield.print(TsetpointString);
}

// Define print_Tmeasured function to correctly print the measured temperature on the LCD screen
static char TmeasuredString[4];
void print_Tmeasured(float T) {
  LCD_shield.setCursor(8, 1);
  dtostrf(T, 5, 1, TmeasuredString);
  LCD_shield.print(TmeasuredString);
}

// Define print_powerMode function to correctly print the power mode (B, battery; M, mains)
void print_powerMode() {
  LCD_shield.setCursor(15, 0);
  if (powerMode == 1) {
    LCD_shield.print("B");
  }
  else if (powerMode == -1) {
    LCD_shield.print("M");
  }
}

// Define print_operationMode function to correctly print the power mode (C, Peltier cooler; H, heating mat)
void print_operationMode() {
  LCD_shield.setCursor(15, 1);
  if (operationMode == 1) {
    LCD_shield.print("C");
  }
  else if (operationMode == -1) {
    LCD_shield.print("H");
  }
}
  
  }
  
  
 void loop() { 
  //Humidity sensor and servo motor
Serial.println("================================="); 
Serial.println("Sample DHT11..."); 
 byte humidity = 0;  
return;
Serial.print((int)humidity);
Serial.println(" %"); 
if (humidity > Y) {  //Y is the best amt of humidity will force the servo to close
for (pos = 0; pos <= 180; pos += 1) { 
myservo.write(pos);
delay(15);
} 
}
 else {
for (pos = 180; pos >= 0; pos -= 1) {

 //MQ2
myservo.write(pos);
delay(15);
 }
delay(1000);
 }
int analogSensor = analogRead(smokeA0);

  Serial.print("Pin A0: ");
  Serial.println(analogSensor);
  // Checks if it has reached the threshold value
  if (analogSensor > sensorThres)
  {
    digitalWrite(greenLed, HIGH);
    digitalWrite(redLed, LOW);
    noTone(buzzer);
  }
  else
  {
    digitalWrite(greenLed, LOW);
    digitalWrite(redLed, HIGH);
    noTone(buzzer);
    tone(buzzer, 1000, 200);
  }
  delay(100);
}
}
Serial.begin(115200);
  Serial.println("Mr ThermoParcel, starting operation...");
  PT100amplifier.begin(MAX31865_4WIRE); 

  // Initialise digital potentiometer
  digitalPotentiometer.begin(digitalPotentiometer_CS);
  digitalPotentiometer.setWiper( 200 );

  // Initialise LCD display shield
  // set up the LCD's number of columns and rows: 
  LCD_shield.begin(16, 2);
  LCD_shield.print("Tsetpoint:   C");
  LCD_shield.setCursor(0, 1);
  LCD_shield.print("Tsample:     C");
}

// Initialise PID constants, temperature related variables and shield button value
int powerMode = 1;  
int operationMode = 1;   
float PT100ratio;  
uint8_t buttonsPressed = 0;
float kp = 500.0;  
float PID_p = 0.0; 
float Tmeasured = -1.0;
float Tsetpoint = 22.0;   
float PID_error = 5;
float PID_value = 0;

// Define print_Tsetpoint function to correctly print the temperature setpoint on the LCD screen
static char TsetpointString[3];
void print_Tsetpoint(int T) {
  
  LCD_shield.setCursor(10, 0);
  dtostrf(T, 3, 0, TsetpointString);
  LCD_shield.print(TsetpointString);
}

// Define print_Tmeasured function to correctly print the measured temperature on the LCD screen
static char TmeasuredString[4];
void print_Tmeasured(float T) {
  // Print Tmeasured in the correct place
  LCD_shield.setCursor(8, 1);
  dtostrf(T, 5, 1, TmeasuredString);
  LCD_shield.print(TmeasuredString);
}

// Define print_powerMode function to correctly print the power mode (B, battery; M, mains)
void print_powerMode() {
  LCD_shield.setCursor(15, 0);
  if (powerMode == 1) {
    LCD_shield.print("B");
  }
  else if (powerMode == -1) {
    LCD_shield.print("M");
  }
}

// Define print_operationMode function to correctly print the power mode (C, Peltier cooler; H, heating mat)
void print_operationMode() {
  LCD_shield.setCursor(15, 1);
  if (operationMode == 1) {
    LCD_shield.print("C");
  }
  else if (operationMode == -1) {
    LCD_shield.print("H");
  }
}

 {
  // Read temperature
  uint16_t rtd = PT100amplifier.readRTD();
  PT100ratio = rtd;
  PT100ratio /= 32768;
  Tmeasured = PT100amplifier.temperature(RNOMINAL, RREF);
  Serial.print("Setpoint Temperature = "); Serial.println(Tsetpoint);
  Serial.print("Temperature = "); Serial.println(Tmeasured);

  // Print temperature values and modes
  print_Tsetpoint(Tsetpoint);
  print_Tmeasured(Tmeasured);
  print_powerMode();
  print_operationMode();

  // Calculate the error between setpoint and measured value
  PID_error = Tmeasured - Tsetpoint;
  //Calculate the P value
  PID_p = operationMode * kp * PID_error;

  // Calculate total PID value, if above maximum (255) keep at 255, if below minimum (0) keep at 0
  PID_value = (int) PID_p; //+ PID_i + PID_d;
  Serial.print("PID_p = "); Serial.println(PID_p);
  Serial.print("powerMode = "); Serial.println(powerMode);
  Serial.print("operationMode = "); Serial.println(operationMode);
  Serial.print("PID_error = "); Serial.println(PID_error);
  Serial.print("PID_value = "); Serial.println(PID_value);

  // If in battery mode (powerMode=1) limit output to avoid battery overload
  // If in mains mode (powerMode=-1) allow full power (255)
  if (powerMode == 1) {
    if (PID_value < 0)
      { PID_value = 0; }
    if (PID_value > 120)  
      { PID_value = 120; }
  }
  else if (powerMode == -1) {
    if (PID_value < 0)
      { PID_value = 0; }
    if (PID_value > 255)  
      { PID_value = 255; }
  }
  Serial.print("Adjusted PID_value = "); Serial.println(PID_value);

  // Set digital potentiometer resistance from PID value
  digitalPotentiometer.setWiper(255 - PID_value);

  // Detect any buttons pressed, change the setpoint value if needed, and display measured and setpoint T
  // delay() funcion calls ensure that enough time is given to press buttons and see the values change
  delay(1000);
  buttonsPressed = LCD_shield.readButtons();
  if (buttonsPressed & BUTTON_SELECT) {
    LCD_shield.setCursor(14, 0);
    LCD_shield.blink();
    delay(1000);
    buttonsPressed = 0;
    while (not (buttonsPressed & BUTTON_SELECT)) {
      buttonsPressed = LCD_shield.readButtons();
      if (buttonsPressed & BUTTON_UP) {
        Tsetpoint += 1;
      }
      if (buttonsPressed & BUTTON_DOWN) {
        Tsetpoint -= 1;
      }
      if (buttonsPressed & BUTTON_RIGHT) {
        powerMode *= -1;
        print_powerMode();
      }
      if (buttonsPressed & BUTTON_LEFT) {
        operationMode *= -1;
        print_operationMode();
      }
      print_Tsetpoint(Tsetpoint);
      LCD_shield.setCursor(14, 0);
      delay(500);
    }   
    LCD_shield.noBlink();
    buttonsPressed = 0;
  }
  Serial.println();
}
