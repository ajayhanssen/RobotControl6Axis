void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the built-in LED pin as an output
  pinMode(2, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
  delay(200);                     // Wait for 1 second
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
  delay(200);                     // Wait for 1 second

  digitalWrite(2, LOW);
}