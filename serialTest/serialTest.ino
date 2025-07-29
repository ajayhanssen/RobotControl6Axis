#define NUMJOINTS 6

int16_t receivedVals[NUMJOINTS];
float jointangles[NUMJOINTS];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() >= NUMJOINTS * 2){ // 6 times 2 byte per num
    uint8_t buffer[NUMJOINTS * 2];
    Serial.readBytes(buffer, NUMJOINTS * 2);

    for (int i = 0; i < NUMJOINTS; i++){
      // Matlab schickt Little-Endian -> erst niederwertiges Byte (buffer[2*i]), dann höherwertiges (buffer[2*i + 1])
      // Bitverschiebung des höherwertigen Bytes um 8 nach links, bitweises ODER zum Hinzufügen des niederwertigen Bytes; casten zu int16_t
      receivedVals[i] = (int16_t)(buffer[2 * i + 1] << 8 | buffer[2 * i]);
      jointangles[i] = (float)receivedVals[i] / 100.0f;
    }

    if (jointangles[0] == 56.09){
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
}
