#include <Arduino.h>
#include <ManchesterDaliBus.h>
// #include <CustomDali.h>
// #include <DaliNZU.h>

ManchesterDaliBus dali;
// Dali dali;
// DALI dali;

void setup()
{
  Serial.begin(9200);
  delay(1000);
  pinMode(15, OUTPUT);
  dali.begin(2, 23);
}

void loop()
{
  // Using custome pq_dali library
  // uint8_t msg[2];
  // msg[0] = 0xFE;
  // msg[1] = 0x7D;
  // dali.send(msg, 2);

  // Using homemade simplified library
  dali.sendFrame(0xFE, 0x7D);

  // Using DaliNZU library
  // dali.Commande_speciale((byte)0xFE, (byte)0x7D);

  digitalWrite(15, HIGH);
  delay(2000);

  // Using custome pq_dali library
  // msg[1] = 0x00;
  // dali.send(msg, 2);

  // Using homemade simplified library
  dali.sendFrame(0xFE, 0x00);

  // Using DaliNZU library
  // dali.Commande_speciale(0xFE, 0x00);

  digitalWrite(15, LOW);
  delay(2000);
}
