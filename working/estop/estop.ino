const int buttonPin = 2; 
int state = 0;
bool toggle = false;

void setup() {
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  state = digitalRead(buttonPin);

    if (state == 0)
    {
      toggle = !toggle;
    }

  if (toggle == true)
  {
    Serial.write(49);
  }
  else
  {
    Serial.write(48);
  }
  delay(100);
}
