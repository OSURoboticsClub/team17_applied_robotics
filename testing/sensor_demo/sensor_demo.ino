#define PRESSURE_INPUT_PIN A5
#define VALVE_CONTROL_PIN 9
#define COMPRESSOR_CONTROL_PIN 2

#define LED_PIN 13

//Code safeties
#define SAFETY_PSI_MAX 25

String input;

void setup() {

  pinMode(COMPRESSOR_CONTROL_PIN, OUTPUT);
  digitalWrite(COMPRESSOR_CONTROL_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(PRESSURE_INPUT_PIN, INPUT);

  pinMode(VALVE_CONTROL_PIN, OUTPUT);
  digitalWrite(VALVE_CONTROL_PIN, HIGH);

  Serial.begin(9600);
}

void loop() {
  uint16_t targetPSI = 0;
  float    currentPSI = 0;
  uint8_t  shouldCompress = 0;
  digitalWrite(VALVE_CONTROL_PIN, HIGH) ;
  
  if(Serial.available()){
    input = Serial.readStringUntil('\n');
    Serial.flush();

    if(isValidNumber(input)){
      targetPSI = input.toInt();
      
      Serial.print("Target PSI: ");
      Serial.println(targetPSI);

      if(targetPSI > SAFETY_PSI_MAX){
        Serial.print("Target PSI too high, bounding to safety max of ");
        Serial.println(SAFETY_PSI_MAX);
      }

      targetPSI = constrain(targetPSI, 0, SAFETY_PSI_MAX);
      
      shouldCompress = 1;

      //Start the compressor
      digitalWrite(COMPRESSOR_CONTROL_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);

      while(shouldCompress){
        currentPSI = readPressure();
        if(currentPSI >= targetPSI){
          shouldCompress = 0;
        }
        Serial.print("Current Pressure: ");
        Serial.print(currentPSI);
        Serial.print(" psi | Target Pressure: ");
        Serial.print(targetPSI);
        Serial.println(" psi");
        delay(150);
      }

      //Stop the compressor
      digitalWrite(COMPRESSOR_CONTROL_PIN, LOW);
      digitalWrite(LED_PIN, LOW);

      //Wait for half a second
      delay(500) ;
      //Release Valve
      digitalWrite(VALVE_CONTROL_PIN, LOW) ;
      //Wait to let all the pressure out
      delay(2500) ;
    }
    
  }

  delay(150);  // ¯\_(ツ)_/¯

  Serial.print("Current Pressure: ");
  Serial.print(readPressure());
  Serial.println(" psi");
  Serial.flush();


  /*
  //Turn off the compressor
  digitalWrite(COMPRESSOR_CONTROL_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  for(int i = 0; i < 2; ++i){
    delay(160);
    Serial.print("Read Analog Value: ");
    Serial.print(readPressure());
    Serial.println(" psi");
    Serial.flush();
  }

  */

  /*
  //Turn on the compressor
  digitalWrite(COMPRESSOR_CONTROL_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  for(int i = 0; i < 6; ++i){
    delay(160);
    Serial.print("Read Analog Value: ");
    Serial.print(readPressure());
    Serial.println(" psi");
    Serial.flush();
  }
  */


}

/*
 * 
 * Pressure is a signal from 1 - 5 volts with 5 volts signifying 1 MPa 
 * See http://nickmccomb.ddns.net:8090/display/AR/Sensors#Sensors-ISE50-T2-62L
 */
float readPressure(){
  analogRead(PRESSURE_INPUT_PIN);
  uint16_t raw_adc = analogRead(PRESSURE_INPUT_PIN);

  float voltage = 0.004887585532746823 * (float) raw_adc;  //ADC count (10 bit) to voltage conversion

  float pressure = (250000.0 * voltage -250000.0) * 0.000145038; //Convert voltage to pascals, then to PSI

  return pressure;
}


boolean isValidNumber(String str){
   for(byte i=0;i<str.length();i++)
   {
      if(isDigit(str.charAt(i))) return true;
        }
   return false;
} 





