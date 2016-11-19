// Calcolo di valor medio di N letture di valori di un sensore


#define tempValNumbers 10   //  numero valori per media

int i = 0;
int itemp = 0;
int timer = 500;
float tempSeries[10];
float tempMean = 0;

void setup()
{
  for (i = 0; i < tempValNumbers; i = i + 1)
  {
    tempSeries[i] = 0;
  }
}

void loop()
{
  if (itemp < tempValNumbers)
  {
    tempSeries[itemp] = analogRead(A0);
    itemp = itemp + 1;
  }
  else
  {
    tempMean = 0;
    Serial.print("valori serie: ");
  } for (i = 0; i < 9; i++)
  {
    tempMean = tempMean + tempSeries[i];
    Serial.print(tempSeries[i]);
    Serial.print(" ");
  }

  tempMean = tempMean / tempValNumbers;
  Serial.print("Media valori: ");
  Serial.println(tempMean);

  delay(timer);
}
