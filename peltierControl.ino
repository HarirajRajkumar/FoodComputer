void peltierControl(unsigned long peltierMillis)
{
  if(peltierMillis > 15000)
  {
    fan[0].peltierRelayState = 1;
    fan[0].RPM = readRPM_1();
    fan[1].RPM = readRPM_2();
  }

  if((fan[0].RPM<4000 || fan[1].RPM < 4000) && peltierMillis >15000)
  {
    fan[0].fault =1;
    fan[1].fault =1;
  } 

  if(fan[0].fault == 1){
  fan[0].peltierRelayState = 0;
  digitalWrite(fanRelay,LOW);
  }
  else if(fan[1].fault  == 1)
  {
  fan[0].peltierRelayState = 0;
  digitalWrite(fanRelay,LOW); 
  } 

  //if(fan[0].fault == 1 || fan[1].fault == 1 ||  fan[0].peltierRelayState == 0 && peltierMillis > 15000)
  //LCDpeltierEmergenyPrint();
  
  digitalWrite(PeltierRelay ,  fan[0].peltierRelayState);
  
  Serial.print(peltierMillis);
 Serial.print("\t");
  Serial.print(fan[0].RPM);Serial.print("\t");
  Serial.print(fan[1].RPM);Serial.print("\t");
  Serial.print(fan[0].fault);Serial.print("\t");
  Serial.print(fan[1].fault);Serial.print("\t");
  Serial.println(fan[0].peltierRelayState);
  
}

unsigned long pulseDuration,pulseDuration_2;

int readRPM_1()
{
pulseDuration = pulseIn(Tacho_1, LOW);
int frequency = 1000000/pulseDuration;

return (frequency/2*60);
}
int readRPM_2()
{
pulseDuration_2 = pulseIn(Tacho_2, LOW);
int frequency = 1000000/pulseDuration_2;

return (frequency/2*60);
 
}

