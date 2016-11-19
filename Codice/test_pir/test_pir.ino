/* PIR test code
 *
 */
#define segnal   2      //D4
#define ledRed   13      //D6
#define ledGreen 12      //D5

long ritardo = 0;
long startDetect;
long durata;
bool pirSegnal;


void setup()
{
  Serial.begin(115200);
  pinMode( ledGreen, OUTPUT );
  pinMode( ledRed, OUTPUT );
  pinMode(segnal, INPUT);

  digitalWrite( ledGreen, LOW );
  digitalWrite( ledRed, LOW );
  pirSegnal = LOW;
  Serial.println("\n\nInizio test taratura PIR tra 2 secondi");
  delay(2000);

}

void loop()
{
  if (digitalRead(segnal) != pirSegnal)
  {
    if (digitalRead(segnal) == HIGH)
    {
      startDetect = millis();
      Serial.println("Rilevato movimento");
      digitalWrite(ledRed, HIGH);
      pirSegnal = HIGH;
    }
    else
    {
      digitalWrite(ledRed, LOW);
      durata = millis() - startDetect;
      Serial.println(durata);
      pirSegnal = LOW;
    }

    //  Serial.println("Stato non cambiato");
    //  Serial.println(millis(), " - Nessun movimento");*/
    //  digitalWrite(ledRed, digitalRead(segnal));
  }
  else
  {
//    Serial.println("...");
    delay(100);
  }
}

