void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
}

void loop() {
  delay(500);
  digitalWrite(4, HIGH);
  delay(500);
  digitalWrite(4,LOW);
  digitalWrite(5, HIGH);
  delay(500);
  digitalWrite(4, HIGH);
  delay(500);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
}
