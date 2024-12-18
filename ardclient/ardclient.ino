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
  if (Serial.available() > 0) {
    char command = Serial.read();  // Read the command sent from the PC

    if (command == 'R') {          // 'R' for Read
      // Read the state of pins D4-D7 (last 4 bits of PIND)
      uint8_t pinState = (PIND >> 4) & 0x0F;  // Extract bits 4-7 (D4-D7)
      
      Serial.write(pinState);      // Send the 4-bit data back to the PC
    } 
    else if (command == 'W') {     // 'W' for Write
      while (Serial.available() < 1) {
        // Wait for the next byte, which should be the 4-bit data to write
      }

      uint8_t data = Serial.read();  // Read the 4-bit data to write
      data &= 0x0F;                  // Ensure only the last 4 bits are used

      // Write the 4-bit data to D8-D11 (PORTB pins 0-3)
      PORTB = (PORTB & 0xF0) | data; // Preserve the upper bits of PORTB and set the lower bits
    }
  }
}
