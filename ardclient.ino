



void setup() {
  Serial.begin(28800);      // Initialize Serial communication at 28800 baud
  pinMode(13, OUTPUT);      // Example pin, can be configured as needed
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    char command = Serial.read();       // Read the command sent from the PC

    if (command == 'R')                 // 'R' for Read
    { 
      uint8_t pinState = PINB;          // Example: Read PINB register
      Serial.write(pinState);           // Send the state of the PINB register back to the PC
    } 


    else if (command == 'W')            // 'W' for Write
    {
      while (Serial.available() < 1) 
      {
        // Wait for the next byte, which should be the data to write
      }

      uint8_t data = Serial.read();     // Read the data to write
      PORTB = data;                     // Example: Write to PORTB register
    }
  }
}
