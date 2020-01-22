/**
 * Functions used for the LCD Screen
 * @Author: Alexandre Tchourbassoff 
 **/

// ************************************************************************* LCD *************************************************************************

//Print message on start
void printSplash() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("****STARTING****");
  lcd.setCursor(0, 1);
  lcd.print("****************");
}

//Prints Temperature & humidity sensor readings
void printHomeScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp:    " + (String)temperature_read + (char)223 + "C");
  lcd.setCursor(0, 1);
  lcd.print("RH:      " + (String)humidity_read + " %");
  homeFlag = 1;
  subScreenLevel = 0;
  screenLevel = 0;
  counter = 0;
  increment = 1;
  lastCounter = 0;
  buttonPushed = 0;
  lastTimeScreenPrint = unixTime;
}

// Print setting menus and submenus
void printMenu() {
  lcd.clear();
  //First line
  lcd.setCursor(0, 0);
  // Prints arrow depending on cursor position
  if (!counter) {
    lcd.write(2);
    lcd.print(" ");
  } else if ((int)counter % 2 == 0) {
    lcd.write(1);
    lcd.print(" ");
  }
  lcd.print(menuItems[screenLevel][subScreenLevel][((int)(counter / 2)) * 2]);
  // Second line
  lcd.setCursor(0, 1);
  if ((int)counter % 2 == 1) {
    lcd.write(1);
    lcd.print(" ");
  }
  lcd.print(menuItems[screenLevel][subScreenLevel][((int)(counter / 2)) * 2 + 1]);
}

// LCD print function for alerts every 30 seconds (consumables)
void printAlert() {
  if (unixTime > alertTime + 30) {
    if (totalMistTime >= mistCleanInterval) {

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("     CLEAN");
      lcd.setCursor(0, 1);
      lcd.print("     FOGGER");

      for (int i = 0; i <= 5; i++) {
        lcd.noBacklight();
        delay(400);
        lcd.backlight();
        delay(400);
      }
      delay(3000);
      backlightFlag = true;
    }

    if (totalFilterTime >= filterInterval) {

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("     CHANGE");
      lcd.setCursor(0, 1);
      lcd.print("     FILTER");

      for (int i = 0; i <= 5; i++) {
        lcd.noBacklight();
        delay(400);
        lcd.backlight();
        delay(400);
      }
      delay(3000);
      backlightFlag = true;
    }
    alertTime = unixTime;
  }
}

void printScreen() {

  //Print Splash at startup
  if (millis() <= 1000) {
    printSplash();
    delay(3000);
    printHomeScreen();

  } else {

    // Refresh screen if not off
    if (unixTime <= lastRotaryUseTime + screenOffTime) {
      //Refresh screeen every 3 sec on or when rotary button is touched or when refresh needed (menu update)
      bool menuRefresh = lastCounter != counter || buttonPushed || refreshFlag;
      if (unixTime > refreshTime + 3 || menuRefresh) {

        lcd.backlight();
        refreshFlag = 0;
        refreshTime = unixTime;
        lastCounter = counter;
        backlightFlag = true;

        //Go back to home screen after homeSreenTime seconds of inactivity
        if ((unixTime > lastTimeScreenPrint + homeScreenTime && !homeFlag) || (homeFlag && !menuRefresh)) {
          printHomeScreen();
          return;
        }
        // Stay on home screen until button pushed
        else if (homeFlag && buttonPushed) {
          buttonPushed = 0;
          counter = 0;
          homeFlag = 0;
        }
        // Don't do anything if button only rotated
        else if (homeFlag) {
          // Handle alerts for consumables
          printAlert();
          return;
        }

        // Print menu
        if (menuRefresh) {

          lastTimeScreenPrint = unixTime;
          // First level of menu
          if (screenLevel == 0) {

            printMenu();
            subScreenLevel = 0;
            counter = max(0, min(4, counter));

            if (buttonPushed) {
              buttonPushed = 0;
              // ****** Home ******
              if (counter == 0) {
                printHomeScreen();
              }
              // Settings
              else {
                subScreenLevel = counter - 1;
                screenLevel = 1;
                counter = 0;
                refreshFlag = 1;
              }
            }
          }
		  
          // Second level of menu
          else if (screenLevel == 1) {

            printMenu();
            if (subScreenLevel == 2) {
              counter = max(0, min(3, counter));
            } else {
              counter = max(0, min(2, counter));
            }
            if (buttonPushed) {
              buttonPushed = 0;
              refreshFlag = 1;
              // ****** Go Back ******
              if (counter == 0) {
                counter = subScreenLevel + 1;
                screenLevel = 0;
                subScreenLevel = 0;
              } else {
                screenLevel = 2;
                setting = counter; // used for the setting to be changed

                // ****** Temperature ******
                if (subScreenLevel == 0) {
                  if (counter == 1) {
                    counter = temp_toggle;
                  } else if (counter == 2) {
                    counter = temperature_set;
                    increment = 0.1; // Change counter increment to 0.1°C
                  }
                }
                // ****** Humidity ******
                else if (subScreenLevel == 1) {
                  if (counter == 1) {
                    counter = humidity_toggle;
                  } else if (counter == 2) {
                    counter = humidity_set;
                  }
                }
                // ****** lights ******
                else if (subScreenLevel == 2) {
                  if (counter == 1) {
                    counter = lights_toggle;
                  } else if (counter == 2) {
                    counter = timeON;
                  } else if (counter == 3) {
                    counter = timeOFF;
                  }
                }
                // ****** Resets ******
                else {
                  counter = 0;
                }
              }
            }
          }
          // Change Setting Level
          else if (screenLevel == 2) {

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(menuItems[1][subScreenLevel][setting]);
            lcd.setCursor(0, 1);
            lcd.write(1);

            // Settings for ON/OFF & YES/NO
            if (setting == 1 || subScreenLevel == 3) {
              counter = abs((int)counter % 2);
              if (subScreenLevel < 3) {
                lcd.print(" " + (String)menuItems[screenLevel][0][(int)counter]);
              }
              else {
                lcd.print(" " + (String)menuItems[screenLevel][1][(int)counter]);
              }
              settingText = "";
            }
            //Settings for numerical values
            else if (setting == 2 || setting == 3) {
              // ****** Temperature ******
              if (subScreenLevel == 0) {
                counter = max(setMinTemp, min(setMaxTemp, counter)); //Prevent user to input out of bound values
                settingText = "        " + (String)counter + (char)223 + "C"; //Display Setting's value
              }
              // ****** Humidity ******
              else if (subScreenLevel == 1) {
                counter = max(setMinHum, min(setMaxHum, counter)); //Prevent user to input out of bound values
                settingText = "         " + (String)(int)counter + ".00%"; //Display Setting's value
              }
              // ****** Lights Schedule ******
              else if (subScreenLevel == 2) {
                counter = max(0, min(23, counter)); //Prevent user to input out of bound values
                settingText = "          " + (String)(int)counter + ":00"; //Display Setting's value
              }
            }

            lcd.print(settingText);
			
			//Stores new settings to EEPROM
            if (buttonPushed) {
              buttonPushed = 0;
              refreshFlag = 1;
              screenLevel = 1;
              // ****** Temperature ******
              if (subScreenLevel == 0) {
                if (setting == 1) {
                  temp_toggle = counter;
                  EEPROM.put(temp_toggle_address, temp_toggle); //Store data in case off power off
                } else if (setting == 2) {
                  temperature_set = max(setMinTemp, min(setMaxTemp, counter));
                  EEPROM.put(temperature_address, temperature_set); //Store data in case off power off
                  temperature_setHigh = temperature_set;
                  temperature_setLow  = temperature_set;
                  tempHotFlag     = true;
                  tempColdFlag    = true;
                  tempReachedFlag = false;
                  increment = 1;
                }
              }
              // ****** Humidity ******
              else if (subScreenLevel == 1) {
                if (setting == 1) {
                  humidity_toggle = counter;
                  EEPROM.put(humidity_toggle_address, humidity_toggle); //Store data in case off power off
                } else if (setting == 2) {
                  humidity_set = max(setMinHum, min(setMaxHum, counter));
                  EEPROM.put(humidity_address, humidity_set); //Store data in case off power off
                }
              }
              // ****** lights ******
              else if (subScreenLevel == 2) {
                if (setting == 1) {
                  lights_toggle = counter;
                  EEPROM.put(lights_toggle_address, lights_toggle); //Store data in case off power off
                } else if (setting == 2) {
                  timeON = counter;
                  EEPROM.put(lights_ON_address, timeON); //Store data in case off power off
                } else if (setting == 3) {
                  timeOFF = counter;
                  EEPROM.put(lights_OFF_address, timeOFF); //Store data in case off power off
                }
              }
              else if (subScreenLevel == 3) {
                if (counter) {
                  // Reset Fogger
                  if (setting == 1) {
                    totalMistTime = 0;
                    EEPROM.put(totalMistTime_address, totalMistTime);
                  }
                  // Reset Filter
                  else if (setting == 2) {
                    totalFilterTime = 0;
                    EEPROM.put(totalFilterTime_address, totalFilterTime);
                  }
                }
              }
              counter = setting;
            }
          }
        }
      }
    }
    // Turn off backlight after screenOffTime seconds
    else if (backlightFlag) {

      lcd.noBacklight();
      backlightFlag = false;
    }
    else {
      // Handle alerts for consumables if screen is off
      printAlert();
    }
  }
}
