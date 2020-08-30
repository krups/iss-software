
// the setup routine runs once when you press reset:
void setup() {
  // initialize the spi port to high impedance
  Serial.begin(115200);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(12, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  Serial.println("SPI bus in high z state");
  delay(1000);
}
