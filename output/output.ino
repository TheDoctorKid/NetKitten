void setup() {
  Serial.begin(9600);      // Initialize Serial communication at 9600 baud

  // Configure pins D4-D7 as inputs for reading
  for (int i = 4; i <= 7; i++) {
    pinMode(i, INPUT);
  }

  // Configure pins D8-D11 as outputs for writing
  for (int i = 8; i <= 11; i++) {
    pinMode(i, OUTPUT);
  }
}

void loop() 
{
  Serial.write((PIND >> 4) & 0x0F);
  delay(60);
}
