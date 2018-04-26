#define PRESSURE_INPUT_PIN A5
#define VALVE_CONTROL_PIN 9
#define COMPRESSOR_CONTROL_PIN 2

#define LED_PIN 13

String input;

void setup() {

  pinMode(COMPRESSOR_CONTROL_PIN, OUTPUT);
  digitalWrite(COMPRESSOR_CONTROL_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(PRESSURE_INPUT_PIN, INPUT);

  pinMode(VALVE_CONTROL_PIN, OUTPUT);
  digitalWrite(VALVE_CONTROL_PIN, LOW);

  Serial.begin(9600);
}

void loop() {
  uint16_t targetPSI = 0;
  
  if(Serial.available()){
    input = Serial.readStringUntil('\n');
    Serial.flush();

    if(isValidNumber(input)){
      targetPSI = input.toInt();
      
      Serial.print("Target PSI: ");
      Serial.println(targetPSI);

      
    }
    
  }

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





