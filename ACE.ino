/**
 * Humidity, Temperature and lightning controlled by Arduino Nano with parametrizable setting through LCD screen
 * Author: Alexandre Tchourbassoff 
 **/

// ************** Import Libraries **************
#include <SimpleDHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LCD.h>
#include <DS3231.h>
#include <EEPROM.h>

// ************** Pins Setup **************

// Rotary Encoder

#define clk 3
#define data 1
#define push 2

// Relays

#define relayLights  5
#define relayMist    4

// Temperature and Humidity sensor

#define pinDHT22     6

// Fans

#define pinFanMist   9
#define pinFanPeltier    10

// Temperature control

#define relayHot    8
#define relayCold   7


// ************** Variables **************

unsigned long unixTime   = 0;
unsigned long refreshTime = 0;
unsigned long testingTime = 0;


// Rotary Encoder
volatile float counter         = 1; //counts number of rotations
float lastCounter              = 1;
volatile bool buttonPushed     = 0;
volatile float increment       = 1;

volatile unsigned long lastRotaryUseTime = 0;


// Debounce for rotary encoder **************************************
static unsigned long lastInterruptTimeButton = 0;
unsigned long interruptTimeButton = 0;
static unsigned long lastInterruptTime = 0;
unsigned long interruptTime = 0;


// LCD **************************************
int subScreenLevel = 0;
int screenLevel = 0;

uint8_t arrow[8] = {0x00, 0x04 , 0x06, 0x1f, 0x06, 0x04, 0x00};     //Arrow character
uint8_t arrowBack[8] = {0x00, 0x04,  0x0C, 0x1F, 0x0C, 0x04, 0x00}; //Reverse Arrow character

int screenOffTime  = 30; //Time in s after lcd backlight off
int homeScreenTime = 10; //Time in s after lcd go home

unsigned long lastTimeScreenPrint = 0;

bool backlightFlag = true;

bool refreshFlag               = 0;
bool homeFlag                  = 0;

String settingText;
char *menuItems[][5][5] = {
  {{"Back", "Temperature", "Humidity", "Lights", "Resets"}},
  {{"Back", "Toggle Temp", "Set Temp"}, {"Back", "Toggle RH", "Set RH"}, {"Back", "Toggle Lights", "Set Time ON", "Set Time OFF"}, {"Back", "Reset Fogger", "Reset Filter"}},
  {{"OFF", "ON"}, {"NO", "YES"}},
};

int setting;


// PID controller for temperature fans **************************************
float tempErr = 0;
int PID_value = 0;

float kp = 85;   float ki = 10;
float PID_p = 0; float PID_i = 0;

float PID_error  = 0;
float preTempErr = 0;
int PID_interval = 5;         // Time interval after PID updated
int I_interval   = 20;        // Time interval after I updated

unsigned long timeTemp_PID = 0;
unsigned long timeTemp_I = 0;


// DHT Sensor (Humidity & Temperature) **************************************
float temperature_read = 0;    //temperature reading of the room
float humidity_read = 0;       //humidity reading of the room

unsigned long timeDHT = 0;


// Temperature *****************************
float temperature_set;                    //Set temperature of room
float setMaxTemp = 30;                    //Max temperature settable
float setMinTemp = 20;                    //Min temperature settable

float tempUndershoot = 0.2;
float tempOvershoot = 1;
float temperature_setLow;
float temperature_setHigh;

int minSpeedTempFan = 60;

bool tempColdFlag     = true;
bool tempHotFlag    = true;
bool tempReachedFlag = false;
bool temp_toggle;         // Set if temperature control is active or not

int temperature_address = 0;       //Address where value is saved if power out
int temp_toggle_address = 4;       //Address where value is saved if power out

unsigned long timeTemp;


// Humidity **************************************
int humidity_set;
float humidity_setHigh; // Mist maker shuts off when set humidity is reached

// max and min settable values for humidity
int setMaxHum = 99;
int setMinHum = 90;

int fanMistSpeedMin = 0;
int fanMistSpeedMax = 20;

bool humFlag  = true;
bool humidity_toggle;  // Set if humidity control is active or not

int humidity_address = 8;               //Address where value is saved if power out
int humidity_toggle_address = 12;       //Address where value is saved if power out
int totalMistTime_address = 28;         //Address where value is saved if power out
int totalFilterTime_address = 32;       //Address where value is saved if power out

unsigned long lastMistTime = 0;
int fanMistInterval = 3600;

unsigned long totalMistTime;  //Time since mist was clean
unsigned long lastMistCount;
unsigned long mistCleanInterval = 15UL * 24UL * 3600UL; //Time after mist maker needs to be cleaned

unsigned long lastFanMistTime;
unsigned long totalFilterTime;                           //Time since filter was changed
unsigned long filterInterval = 30UL * 24UL * 3600UL; //Time after filter needs to be changed

unsigned long tempMistValue;
unsigned long tempFilterValue;

bool mistFlag = false;


// Lights **************************************
int timeON;
int lights_ON_address = 16;       //Address where value is saved if power out
int timeOFF;
int lights_OFF_address = 20;       //Address where value is saved if power out
bool lights_toggle;
int lights_toggle_address = 24;       //Address where value is saved if power out
Time  now;


// LCD Alerts *************************************

unsigned long alertTime;
bool alertFlag = false;


// ************** Clock Module **************

DS3231  rtc(SDA, SCL);

// ************** LCD **************

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);


// ************** Temp & humidity sensors **************

SimpleDHT22 dht22(pinDHT22);


// *************************************** CODE ***************************************

void setup() {

  // Change pin 9 & 10 frequency to 31372.55 Hz for PWM Fans
  TCCR1B = TCCR1B & B11111000 | B00000001;
  Serial.begin(19200);
  // Data acquisition
  Serial.println("LABEL,Timer,Temperature_set,Temperature_read,Humidity_set,Humidity_read,PID_value,PID_i,tempColdFlag,tempHotFlag,totalMistTime,totalFilterTime");
  Serial.println("RESETTIMER");

  rtc.begin(); // initialize clock
  unixTime = rtc.getUnixTime(rtc.getTime());
  // Set up Time
  //  rtc.setDOW(SUNDAY);     // Set Day-of-Week to SUNDAY
  //  rtc.setTime(18, 06, 30);     // Set the time to 12:00:00 (24hr format)
  //  rtc.setDate(1, 1, 2000);   // Set the date

  lcd.begin(16, 2);
  lcd.clear();
  lcd.createChar(1, arrow);   //Create the arrow symbol
  lcd.createChar(2, arrowBack);   //Create the reverse arrow symbol

  // Relay pins
  pinMode(relayHot, OUTPUT);
  pinMode(relayCold, OUTPUT);
  pinMode(relayLights, OUTPUT);
  pinMode(relayMist, OUTPUT);

  digitalWrite(relayHot, HIGH);
  digitalWrite(relayCold, HIGH);
  digitalWrite(relayLights, HIGH);
  digitalWrite(relayMist, HIGH);

  // Fans pins
  pinMode(pinFanMist, OUTPUT);
  pinMode(pinFanPeltier, OUTPUT);

  analogWrite(pinFanMist, fanMistSpeedMin);
  analogWrite(pinFanPeltier, 0);

  // Sensor pin
  pinMode(pinDHT22, INPUT);

  // Rotary pins
  pinMode (clk, INPUT);
  pinMode (data, INPUT);
  pinMode (push, INPUT_PULLUP);
  
  //Interrupts for rotary encoder
  attachInterrupt(digitalPinToInterrupt(3), rotaryTurn, LOW); 
  attachInterrupt(digitalPinToInterrupt(2), rotaryPush, LOW);

  //Read sensor until data is returned
  while (temperature_read == 0) {
    readSensor();
  }

  // Recover value store on arduino EEPROM
  EEPROM.get(temperature_address, temperature_set);
  EEPROM.get(temp_toggle_address, temp_toggle);
  EEPROM.get(humidity_address, humidity_set);
  EEPROM.get(humidity_toggle_address, humidity_toggle);
  EEPROM.get(lights_toggle_address, lights_toggle);
  EEPROM.get(lights_ON_address, timeON);
  EEPROM.get(lights_OFF_address, timeOFF);
  EEPROM.get(totalMistTime_address, totalMistTime);
  EEPROM.get(totalFilterTime_address, totalFilterTime);

  // Initiate time and temp values
  lastTimeScreenPrint = unixTime;
  lastRotaryUseTime = unixTime;

  lastMistCount = unixTime;
  lastMistTime = unixTime;
  lastFanMistTime = unixTime;

  temperature_setHigh = temperature_set;
  temperature_setLow  = temperature_set;

}

void loop() {

  unixTime = rtc.getUnixTime(rtc.getTime());

  // Data Acquisition	
  if (unixTime > testingTime + 5) {
    testingTime = unixTime;
    Serial.print("DATA,TIMER,");
    Serial.print(temperature_set);
    Serial.print(",");
    Serial.print(temperature_read);
    Serial.print(",");
    Serial.print(humidity_set);
    Serial.print(",");
    Serial.print(humidity_read);
    Serial.print(",");
    Serial.print(PID_value);
    Serial.print(",");
    Serial.print(PID_i);
    Serial.print(",");
    Serial.print(tempColdFlag);
    Serial.print(",");
    Serial.print(tempHotFlag);
    Serial.print(",");
    Serial.print(totalMistTime);
    Serial.print(",");
    Serial.println(totalFilterTime);
  }

  // Read sensor value for temp and RH
  readSensor();

  // Manage lightning
  checkLights();

  // Adjust Temp and humidity
  humidityCheck();
  tempCheck();

  //Refresh LCD
  printScreen();

  // Save Running time of fogger and filter every 2 hours of operation
  saveRunningTimes();
}


// *************************************** Functions *******************************************


void saveRunningTimes() {

  // Save Running time of fogger and filter every 2 hours of operation

  EEPROM.get(totalMistTime_address, tempMistValue);
  EEPROM.get(totalFilterTime_address, tempFilterValue);

  // Check gap between stored and live value and store
  if (totalMistTime - tempMistValue > 7200) {
    EEPROM.put(totalMistTime_address, totalMistTime);
  }

  if (totalFilterTime - tempFilterValue > 7200) {
    EEPROM.put(totalFilterTime_address, totalFilterTime);
  }
}


// ******************* Lights *******************

void checkLights() {
  now = rtc.getTime();

  //Checks if lightning is activated
  if (lights_toggle) {

	//Turns lights on and off according to schedule
    if (timeON < timeOFF) {

      if (now.hour >= timeON && now.hour < timeOFF) {
        digitalWrite(relayLights, LOW);

      }
      else if (now.hour >= timeOFF) {
        digitalWrite(relayLights, HIGH);
      }
      else {
        digitalWrite(relayLights, HIGH);
      }
    }
    else if (timeON > timeOFF) {

      if (now.hour >= timeON && now.hour <= 23) {
        digitalWrite(relayLights, LOW);
      }
      else if (now.hour < timeOFF) {
        digitalWrite(relayLights, LOW);
      }
      else if (now.hour >= timeOFF && now.hour < timeON) {
        digitalWrite(relayLights, HIGH);
      }
    }
    else {
      digitalWrite(relayLights, LOW);
    }
  }

  else {
    digitalWrite(relayLights, HIGH);
  }
}


// ******************* Temperature & humidity sensor Read *******************
void readSensor() {
  int err = SimpleDHTErrSuccess;
  if (unixTime > timeDHT + 2) {
    timeDHT = unixTime;
    if ((err = dht22.read2(&temperature_read, &humidity_read, NULL)) != SimpleDHTErrSuccess) {
      //      Serial.print("Read DHT22 failed, err="); Serial.println(err);
      return;
    }
  }
}


// ******************* Humidity Adjustements *******************
void humidityCheck() {

  // Checks if humidity control is activated
  if (humidity_toggle) {
    humidity_setHigh = min(99.9, humidity_set + 2);

    // Activate fogger if current humidity below set humidity
    if ((float)humidity_read <= humidity_set) {
      digitalWrite(relayMist, LOW);
      analogWrite(pinFanMist, fanMistSpeedMax);

      // count time passed misting
      totalMistTime = totalMistTime + unixTime - lastMistCount;
      totalFilterTime = totalFilterTime + unixTime - lastMistCount;

      mistFlag = true;
      lastMistTime = unixTime;
    }
    // Deactivate fogger when humidity + 2% is reached (2% overshoot 
    else if ((float)humidity_read >= humidity_setHigh && mistFlag) {
      digitalWrite(relayMist, HIGH);
      analogWrite(pinFanMist, fanMistSpeedMin);
      mistFlag = false;
    }
	
	// OPTIONAL: Mist every 1h for 5 min
  /*   if (unixTime > lastMistTime + fanMistInterval) {
      analogWrite(pinFanMist, fanMistSpeedMax);
      digitalWrite(relayMist, LOW);

      // count time passed faning and misting
      totalMistTime = totalMistTime + unixTime - lastMistCount;
      totalFilterTime = totalFilterTime + unixTime - lastMistCount;


      // Shut mist down after 5min
      if (unixTime >= lastMistTime + fanMistInterval + 300) {
        digitalWrite(relayMist, HIGH);
        totalMistTime = totalMistTime - unixTime + lastMistCount;
      }
      //Keep fanning 2 min after misting
      if (unixTime >= lastMistTime + fanMistInterval + 520) {
        analogWrite(pinFanMist, fanMistSpeedMin);
        lastMistTime = unixTime;
      }
    } */
	
	//Update Misting time
    lastMistCount = unixTime;

  } else {
    digitalWrite(relayMist, HIGH);
    analogWrite(pinFanMist, 0);
  }
}

// ******************* Temperature Adjustments *******************
void tempCheck() {

  // Checks if temperature control is activated
  if (temp_toggle) {
	
	// PI Controller Speed of peltier module fans
    tempErr = temperature_set - temperature_read;
	
    PID_p = kp * abs(tempErr);

    if (unixTime > timeTemp_I + I_interval) {
    // Change PID_I only if the temperature error (tempErr) stagnate
      if (preTempErr >= tempErr && !tempReachedFlag && PID_value < 255) {
        if (!tempHotFlag) {
          PID_i = max(0, min(255, PID_i + ki * -tempErr));
        } else if (!tempColdFlag) {
          PID_i = max(0, min(255, PID_i + ki * tempErr));
        }
      }
      preTempErr = tempErr;
      timeTemp_I = unixTime;
    }

    PID_value = min(255, max(minSpeedTempFan + PID_i, PID_p + PID_i));
	
	// Reset PID_value if temperature is reached
    if (tempHotFlag && tempColdFlag) {
      digitalWrite(relayHot, HIGH);
      digitalWrite(relayCold, HIGH);
      PID_i = 0;
      PID_value = minSpeedTempFan;
    }
	
	//Set peltier module fan speed
    analogWrite(pinFanPeltier, PID_value);

    // Switch current direction of peltier module to warm or cool
    // Cooling
    if (temperature_read >= temperature_setHigh && tempHotFlag) {
	  //Turn off relays
      digitalWrite(relayHot, HIGH);
      digitalWrite(relayCold, HIGH);

      // Give some time to relays before reactivating
      if (unixTime > timeTemp + 1) {
        digitalWrite(relayCold, LOW);
        timeTemp = unixTime;
        tempHotFlag = false;
        tempColdFlag  = true;
        tempReachedFlag = false;

        temperature_setHigh = temperature_set + tempUndershoot;
        temperature_setLow  = temperature_set - tempOvershoot;
      }
      // Warming
    } else if (temperature_read <= temperature_setLow && tempColdFlag) {
	  //Turn off relays
      digitalWrite(relayHot, HIGH);
      digitalWrite(relayCold, HIGH);

      // Give some time to relays
      if (unixTime > timeTemp + 1) {
        digitalWrite(relayHot, LOW);
        timeTemp = unixTime;
        tempHotFlag = true;
        tempColdFlag  = false;
        tempReachedFlag = false;
        temperature_setHigh = temperature_set + tempOvershoot;
        temperature_setLow  = temperature_set - tempUndershoot;
      }

    }
    // Temp is reached, peltier module is shut down
    if (abs(temperature_read - temperature_set) <= 0.01 && !tempReachedFlag) {
      tempReachedFlag = true;
      tempHotFlag = true;
      tempColdFlag  = true;
    }
  }
  else {
    digitalWrite(relayHot, HIGH);
    digitalWrite(relayCold, HIGH);
    analogWrite(pinFanPeltier, 0);
    tempHotFlag = true;
    tempColdFlag  = true;
    tempReachedFlag  = false;
  }
}



// ************************************************************************* Rotary Encoder *************************************************************************
void rotaryTurn() {

  interruptTime = millis();
  if (interruptTime < lastInterruptTime) {
    lastInterruptTime = 0;
  }

  // If interrupts come faster than 15ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 15) {
    if (digitalRead(data) == LOW)
    {
      counter = counter + increment; 
    }
    else {
      counter = counter - increment;
    }
  }
  // Keep track of when we were here last (no more than every 15ms)
  lastInterruptTime = interruptTime;
  lastRotaryUseTime = unixTime;
}

void rotaryPush() {

  interruptTimeButton = millis();
  if (interruptTimeButton < lastInterruptTimeButton) {
    lastInterruptTimeButton = 0;
  }

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTimeButton - lastInterruptTimeButton > 5) {
    buttonPushed = 1;
  }

  // Keep track of when we were here last (no more than every 5ms)
  lastInterruptTimeButton = interruptTimeButton;
  lastRotaryUseTime = unixTime;
}
