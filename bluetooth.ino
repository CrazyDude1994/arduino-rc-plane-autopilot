void setup()
{
  Serial1.begin(19200);
  Serial.begin(19200);

  Serial1.println("AT+BTPOWER=1");
}

void loop()
{

  if (Serial.available())

    Serial1.print((char)Serial.read());

  else  if (Serial1.available())

    Serial.print((char)Serial1.read());
}

