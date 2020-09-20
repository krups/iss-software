
    #define CS_TC1 20
    #define CS_TC2 21
    #define MUX0 16
    #define MUX1 17
    #define TC1_FAULT 25
    #define TC2_FAULT 26
    #define SEC_CTRL1 27
    #define SEC_CTRL2 30
    
void setup() {

  Serial.begin(115200);
  pinMode(SEC_CTRL1, INPUT);
  pinMode(SEC_CTRL2, INPUT);
  pinMode(CS_TC1, INPUT);
  pinMode(CS_TC2, INPUT);
  pinMode(MUX0, INPUT);
  pinMode(MUX1, INPUT);
  pinMode(TC1_FAULT, INPUT);
  pinMode(TC2_FAULT, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("All Pin in High Z");
  delay(1000);
}
