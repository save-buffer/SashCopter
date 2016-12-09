
void connect_headset()
{
  Serial.write(0xc2);
  double attention = 0;
}

byte wait_one_byte()
{
  while (!Serial.available());
  return Serial.read();
}
byte read_one_byte()
{
  return Serial.available() ? Serial.read() : 0;
}

void smart_flush()
{
  if (Serial.available() > 64)
    Serial.flush();
}

bool update_attention()
{
  //The protocol states that the first 2 bytes of a valid transmission are 170
  if (read_one_byte() != 170 && Serial.available() > 0)
  {
    smart_flush();
    return false;
  }
  if (read_one_byte() != 170)
  {
    return false;
  }
  //we have read 2 170's. transmission is a go
  int payload_length = wait_one_byte();
  if (payload_length > 169)
    return false;

  byte generated_checksum = 0;
  byte payload[64] = { 0 };
  for (int i = 0; i < payload_length; i++)
  {
    payload[i] = wait_one_byte();
    generated_checksum += payload[i];
  }
  generated_checksum = 255 - generated_checksum;
  byte checksum = wait_one_byte();

  if (checksum != generated_checksum)
    return false;

  int poor_quality = 0;
  for (int i = 0; i < payload_length; i++)
  {
    switch (payload[i])
    {
      case 0xD0:
        //headset is connected
        break;
      case 4:
        attention = payload[++i] / 100.0;
        break;
      case 2:
        poor_quality = payload[++i];
        if (poor_quality > 170)
        {
          attention = 0;
          return false;
        }
        break;
      case 0xD1:
      case 0xD2:
      case 0xD3:
      case -70:
        attention = 0.0;
        return false;
      case 0x80:
        i += 3;
        break;
      case 0x83:
        i += 25;
        break;
    }
  }
  return true;
}

