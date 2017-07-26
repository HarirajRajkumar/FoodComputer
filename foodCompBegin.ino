 void foodCompBegin(void)
  {
    
  myLCD.begin();
  myLCD.backlight();
  myLCD.setCursor(0,1);// next line
  myLCD.print("Food Computer");
  myLCD.setCursor(0,2);// next line
  myLCD.print("VERSION 1.2.2 \"MEGA\"");
   myLCD.setCursor(0,3);// next line
   myLCD.print("_Peltier");
  //Serial.println("Food Computer");
  //Serial.println("Data Print v1");
  //Serial.print("\n");
  //Serial.print("\n");
  delay(3500);
  myLCD.clear();

  //Serial.println("SETPOINT(*T)\tPID INPUT(*T)\tPID OUTPUT(*T)");
  myLCD.print("HEATR");
  myLCD.setCursor(0,1);// next line
  myLCD.print("PWM%");
  

  myLCD.setCursor(0,2);// next line
  myLCD.print("1:");
  
  myLCD.setCursor(0,3);
  myLCD.print("2:");
  
  
  myLCD.setCursor(6,0);//next column
  myLCD.print("TEMP");
  myLCD.setCursor(11,0);
  myLCD.print("ENV/HTRs");
  myLCD.setCursor(6,1);
  myLCD.print("SET:");
  myLCD.setCursor(10,1);
  myLCD.print(setEnvTemp);
  myLCD.print("/");
  myLCD.print(setHtrTemp);
  myLCD.print((char)223);
  myLCD.print("C");
  myLCD.setCursor(6,2);
  myLCD.print("Cur:");
  myLCD.setCursor(6,3);
  myLCD.print("FAN-");
  myLCD.setCursor(12,3);
  myLCD.print("H:");
  }// close foodCompBegin

