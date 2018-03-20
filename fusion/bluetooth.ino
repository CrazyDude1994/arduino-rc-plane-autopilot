bool enableBt() {
  Serial1.begin(19200);

  Serial1.println("AT+BTPOWER=1");
//  Serial1.println("AT+BTHOST=RC Plane");
}

void btLoop() {
  while (Serial1.available())
    Serial.print((char)Serial1.read());
}

