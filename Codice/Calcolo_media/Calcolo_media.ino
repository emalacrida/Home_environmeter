/*void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
*/
/////////////////////
#define PIN 5 //un pin analogico

int NumValTemp = 10;
int valoriTemp[NumValTemp];    //Array valori letti
int T_campionamento;
int sommaTemp;
float mediaTemp;

void setup(){
}

void loop(){
  //così ti salvi 5 valori
  for(int i=0; i<NumValTemp; i++){
    valoriTemp[i] = analogRead(PIN);
    delay(T_campionamento); //se vuoi fare una lettura ogni tot millisecondi
                            //non è necessario
  } 
  sommaTemp = 0; // devi essere sicuro che il primo valore sia zero, altrimenti poi sballa tutto
  
  // finito di acquisire, fai la somma di tutti i campioni
  for(int i=0; i<5; i++)
    sommaTemp = sommaTemp + valoriTemp[i];

  //ottenuta la somma puoi fare la media
  mediaTemp = sommaTemp / NumValTemp;
  
  //il loop finisce e di seguito ricomincia
  //per questo è mecessario azzerare di volta in volta la variabile somma
}
