
#define MOSI_PIN    4
#define SCK_PIN     3
#define MISO_PIN    2
#define CS_TC1      1
#define CS_TC2      13
#define SEC_CTRL_2  0

#define SEC_ACT     7
#define SEC_CTRL_1  8
#define MUX0        23
#define MUX1        24
#define TC1_FAULT   21
#define TC2_FAULT   22


// the setup routine runs once when you press reset:
void setup() {
  // initialize the spi port to high impedance
  Serial.begin(115200);
  pinMode(MOSI_PIN, INPUT);
  pinMode(SCK_PIN, INPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(CS_TC1, INPUT);
  pinMode(CS_TC2, INPUT);
  pinMode(SEC_CTRL_1, INPUT);
  pinMode(SEC_CTRL_2, INPUT);
  pinMode(MUX0, INPUT);
  pinMode(MUX1, INPUT);
  pinMode(TC1_FAULT, INPUT);
  pinMode(TC2_FAULT, INPUT);
  
  pinMode(SEC_ACT, OUTPUT);
  digitalWrite(SEC_ACT, HIGH);
}

// the loop routine runs over and over again forever:
void loop() {
  Serial.println("All pins but SEC_ACT in high z state, SEC_ACT=1");
  delay(1000);
}
