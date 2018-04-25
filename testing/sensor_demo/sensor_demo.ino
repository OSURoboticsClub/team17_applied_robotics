void setup() {

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(A5, INPUT);

  Serial.begin(9600);
}

void loop() {
  digitalWrite(2, LOW);
  digitalWrite(13, LOW);
  for(int i = 0; i < 250; ++i){
    delay(20);
    Serial.print("Read Analog Value: ");
    Serial.println(analogRead(A5));
  }
  digitalWrite(2, HIGH);
  digitalWrite(13, HIGH);
  for(int i = 0; i < 50; ++i){
    delay(20);
    Serial.print("Read Analog Value: ");
    Serial.println(analogRead(A5));
  }


}
