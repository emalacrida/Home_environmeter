/* Home Environment monitor station
 * based on NodeMCU (ESP8266).
 * 
 * Detects:
 * Atmosferic pressure, temperature and humidity.
 * Number of mouvements in a period of time.
 * Ambient light intensity.
 * Air pollution index (concentration of NH3,NOx, alcohol, Benzene, smoke,CO2 ,etc.)
 * 
 * Used sensors:
 * DHT 22  (temp and humidity)
 * BMP 180 (pessure and temperature)
 * PIR     (IR mouvements)
 * GY 30   (BH 1750 - Digital light sensor)
 * MQ 135  (MQ-2 gas sensor)
 * 
 * Data are sent and stored on ThingSpeak, availlable on Blynk APP and visible 
 * accessing the web server on ESP
 * 
 * ===============================================
 * 
 *   V0 = lightStatus
 *   V1 = temperature from BMP180
 *   V2 = uptime
 *   V3 = ambientLight
 *   V4 = currentMode
 *   V5 = Humidity
 *   V6 = Temp from DHT22
 *   V7 = counter
 *   V8 = LEDstatus
 *   V9 = Pressure
 *   V10 = 
 *   V15 = airqual              Aggiunto
 *
 *
 */

//#define BLYNK_DEBUG           // slowdown x10 operation more detailed print
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space

#include <time.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
//#include <OneWire.h>       // serve per sensore temperatura
#include <RemoteSwitch.h>
#include <NTPtime.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "DHT.h"
#include <SFE_BMP180.h>
#include <Wire.h>
#include <ArduinoJson.h>
// #include <switchKaKu.h>
// #include <EEPROM.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ArduinoOTA.h>
#include <BH1750.h>           //lib sensore digitale luce
#include <Average.h>          //lib statistiche
//#define HOME                //uncomment for home server
//#define ADDR_KITCHEN 0        //EEPROM adress for kitchen light intensity
//#define ADDR_HALL 4
//#define ADDR_LIGHTTRESHOLD 8
#define DHTPIN 12             // D6 sensore umidita
#define DHTTYPE DHT22         // tipo snsore umidita
#define LED 16
//#define transmitterPIN 14     // D5  da liberare
#define airqualityPIN 12      // D6 analog inpot A0
#define movementPIN 2         // D4

// Sensor calibration values ===================================

float bhtTempCalib = 0;
float bmpTempCalib = 0;
float bhtHumiCalib = 0;

// Specific installation setup parameters ======================

#define SID  "adircalam2"     // wifi SSID
#define PAS  "casa.em.230552"
//#define SID  "BiblioWiFi"     // wifi bilioteca
//#define PAS  ""
#define HOMESERVER  "X.X.X.X"
#define thingspeakAPIkey "XNP8Q4DF8DWU7F0P"
#define MOVEMENT_TIMEOUT 600000
#define TIMEZONE 1
#define DAYLIGHTSAVINGTIME 1
#define ALTITUDE 200.0        // sensor altitude for sea level pressure calc.
#define HOSTNAME "emiot"
#define VERSION "0.042"
#ifdef HOME
#define AUTH "7bb63a4186694150b71c66ce29ee6c9c"
#else
#define AUTH "7bb63a4186694150b71c66ce29ee6c9c"
#endif

// End specific installation setup parameters ==================

float temperature = 0;
float bhttemp = 0;
bool movementDetected = 0;
bool updating = 0;
int countMovmentsInterval = 300000;  // aggiunto intervallo calcolo numero movimenti
int countMovments = 0;               // agiunto Contatore sensore movimento
int currentMode = 3;
int kitchenIntensity;        // togliere quando di sistema weberver
int hallIntensity;
int airQuality;              // togliere quando di sistema weberver
double bar, hum;
uint16_t ambientLight;
unsigned long updateStarted_time = 0;
String s_hum;
// OneWire  ds(temperaturePIN);     // serve solo per sensore temperatura
//bool lightStatus = LOW;
//bool alarmArmed = LOW;
//int lightTreshold;
//KaKuSwitch kaKuSwitch(transmitterPIN);

SimpleTimer timer;
SFE_BMP180 pressure;
DHT dht(DHTPIN, DHTTYPE);

BH1750 lightMeter;

ESP8266WebServer server(80);
//ESP8266HTTPconnectWiFiUpdateServer httpUpdater;      //?????

void connectWiFi(const char* ssid = SID, const char* pass = PAS, int timeout = 10);
void callBack();
void readNTP();
//void readTemp();             // modificare per NTP
void readAirquality();
void readAmbientLight();
void readHumiditySensor();
void readPressure();
void periodicUpdateThingspeak();
void interruptHandler();
void startWebServer();
void handleSensorData();
void stateMachine();
//void licht_uit();
//void licht_aan();
void updateThingspeak(String APIkey, String tsData);
void handleRoot();

void setup()
{
  /* Commentato non si usa EEPROM
   *
    EEPROM.begin(512);
    EEPROM.get(ADDR_KITCHEN, kitchenIntensity);
    EEPROM.get(ADDR_HALL, hallIntensity);
    EEPROM.get(ADDR_LIGHTTRESHOLD, lightTreshold);*/
  connectWiFi();
#ifdef HOME
  Blynk.config(AUTH, HOMESERVER);
#else
  Blynk.config(AUTH);
#endif

// Set time intervals for routines execution

  timer.setInterval(1000L, callBack);
  //timer.setInterval(3600000L, notifyUptime);
  //timer.setInterval(15000L, readNTP);                // valore originale 1500 solo NTP get
  timer.setInterval(5000L, readAirquality);
  timer.setInterval(5000L, readHumiditySensor);
  timer.setInterval(5000L, readPressure);               // valore originale 15000L
  timer.setInterval(300000, periodicUpdateThingspeak);  // valore originale 60000
  timer.setInterval(5000L, readAmbientLight);           // aggiunto

// pins definitons
  
  pinMode(LED, OUTPUT);
//  pinMode(transmitterPIN, OUTPUT);
  pinMode(movementPIN, INPUT);                          // modificato INPUT_PULLUP
  //pinMode(temperaturePIN, INPUT_PULLUP);
  pinMode(airqualityPIN, INPUT_PULLUP);

  Serial.begin(115200);
  dht.begin();
  lightMeter.begin();
  delay(2000);  
  attachInterrupt(digitalPinToInterrupt(movementPIN), interruptHandler, HIGH); //cambiato CHANGE in HIGH
  pressure.begin();
  startWebServer();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  /* togliere per OTA
  server.handleClient();
  ArduinoOTA.handle();
  if (!updating){*/
  Blynk.run();
  timer.run();
  Blynk.run();

  //    stateMachine();         // non dovrebbe servire
  //    Blynk.run();

  //  } else {
  //      if ((millis() - updateStarted_time) > 300000){
  //        updating = false;
  //attachInterrupt(digitalPinToInterrupt(movementPIN), interruptHandler, CHANGE); ????????
  //      }
  //  }
}

/* da togliere non richiamata
void stateMachine()
{
  static int prevMode = 3;
  static unsigned long lastMovementDetected = 0;
  static unsigned long previousTime = 0;
  unsigned long currentMillis = millis();
  unsigned long currentTime = (getTime(TIMEZONE, DAYLIGHTSAVINGTIME) % 86400L);
  if (currentMode != prevMode){
    prevMode = currentMode;
    switch (currentMode) {
    case 0:
      licht_uit();
      break;
    case 2:
      licht_aan();
      break;
    }
  }
  switch (currentMode) {
  case 1:
    if ((lightStatus == LOW) & (ambientLight < lightTreshold)) {
      lightStatus == HIGH;
      licht_aan();
    }
    break;
  case 3:
    if (movementDetected) {
      if ((lightStatus == LOW) & (ambientLight < lightTreshold)){
            licht_aan();
            lastMovementDetected = currentMillis;
      }
    }
    if (lightStatus == HIGH){
      if ((currentMillis - lastMovementDetected) > MOVEMENT_TIMEOUT){
        licht_uit();
      }
    }
    break;
  }
  if (movementDetected) {
    if (alarmArmed){
      alarmArmed = LOW;
      Blynk.notify("Movement detected in the livingroom!!");
    }
    lastMovementDetected = currentMillis;
    movementDetected = 0;
  }
  if ((previousTime < SWITCH_MODE3) & (currentTime >= SWITCH_MODE3)) {
    currentMode = 3;
    BLYNK_LOG("Switching to mode %d", currentMode);
    previousTime = currentTime;
  }
  if ((previousTime < SWITCH_MODE2) & (currentTime >= SWITCH_MODE2)) {
    currentMode = 2;
    BLYNK_LOG("Switching to mode %d", currentMode);
    previousTime = currentTime;
  }
  if ((previousTime < SWITCH_MODE1) & (currentTime >= SWITCH_MODE1)) {
    currentMode = 1;
    BLYNK_LOG("Switching to mode %d", currentMode);
    previousTime = currentTime;
  }
  if ((previousTime < SWITCH_MODE0) & (currentTime >= SWITCH_MODE0)) {
    currentMode = 0;
    BLYNK_LOG("Switching to mode %d", currentMode);
    previousTime = currentTime;
  }
  previousTime = currentTime;
}
fine togliere state machine */

void callBack() {
  // copiate per compilare
  static long counter = 0;
  static bool LEDstatus = HIGH;
  digitalWrite(LED, LEDstatus);
  /* non si legge la EEPROM
  int oldHallIntensity;
  EEPROM.get(ADDR_HALL, oldHallIntensity);
  int oldKitchenIntensity;
  EEPROM.get(ADDR_KITCHEN, oldKitchenIntensity);
  int oldLightTreshold;
  EEPROM.get(ADDR_LIGHTTRESHOLD, oldLightTreshold);
  static long counter = 0;
  static bool LEDstatus = HIGH;
  digitalWrite(LED, LEDstatus);
  if (oldHallIntensity != hallIntensity){
    if (lightStatus){
      switchKaku(transmitterPIN, NEW_KAKU_TRANSMITTERID2, 1, 3, true, 3, hallIntensity);
    }
    EEPROM.put(ADDR_HALL, hallIntensity);
    EEPROM.commit();
  }
  if (oldKitchenIntensity != kitchenIntensity){
    if (lightStatus){
      switchKaku(transmitterPIN, NEW_KAKU_TRANSMITTERID2, 1, 4, true, 3, kitchenIntensity);
    }
    EEPROM.put(ADDR_KITCHEN, kitchenIntensity);
    EEPROM.commit();
  }
  if (oldLightTreshold != lightTreshold){
    EEPROM.put(ADDR_LIGHTTRESHOLD, lightTreshold);
    EEPROM.commit();
  }  */
  LEDstatus = !LEDstatus;
  counter += 1;
  if (counter > 1000) {
    counter = 0;
  }
  long uptime = millis() / 60000L;

  //  ambientLight = analogRead(A0);   //sostituito da misura gas
  Blynk.virtualWrite(V2, uptime);
  Blynk.run();
  Blynk.virtualWrite(V8, LEDstatus * 255);
  Blynk.run();
  Blynk.virtualWrite(V7, counter);
  Blynk.run();
  Blynk.virtualWrite(V3, ambientLight);
  Blynk.run();
  // commentate le righe delle variabili non usate
  //  Blynk.virtualWrite(V4, currentMode);
  //  Blynk.run();
  //  Blynk.virtualWrite(V28, alarmArmed*255);
  //  Blynk.run();
  //  Blynk.virtualWrite(V0, lightStatus*255);
  //  Blynk.run();
  //  Blynk.virtualWrite(V11, kitchenIntensity);
  //  Blynk.run();
  //  Blynk.virtualWrite(V13, hallIntensity);
  //  Blynk.run();
  Blynk.virtualWrite(V15, airQuality);
  Blynk.run();
  Blynk.virtualWrite(V1, temperature);
  Blynk.run();
  Blynk.virtualWrite(V14, countMovments); // Aggiunto
  Blynk.run();                            // Aggiunto
}

void notifyUptime()
{
  long uptime = millis() / 60000L;
  Blynk.notify(String("Running for ") + uptime + " minutes.");
}

/*BLYNK_WRITE(31)
{
  if(param[0].asInt()){
    currentMode = (currentMode + 1) % 4;
  }
}

BLYNK_WRITE(27)
{
  if(param[0].asInt()){
    alarmArmed = HIGH;
  }
}

BLYNK_WRITE(29)
{
  if(param[0].asInt()){
    if (lightStatus){
      licht_uit();
    } else {
      licht_aan();
    }
  }
}

BLYNK_WRITE(26)
{
  lightTreshold = param[0].asInt();
}

BLYNK_WRITE(10)
{
  kitchenIntensity = param[0].asInt();
}

BLYNK_WRITE(12)
{
  hallIntensity = param[0].asInt();
}*/

void readAmbientLight()                        // con GY 30
{
  uint16_t lux = lightMeter.readLightLevel();
  ambientLight = lux;
  //Serial.print("Luce ambiente lux: ");
  //Serial.println(ambientLight);
  BLYNK_LOG("Ambient light: %i", ambientLight);
}

void readAirquality()
{
  airQuality = analogRead(A0);
  //Serial.print("Qualita aria: ");
  //Serial.println(airQuality);
  BLYNK_LOG("Airquality: %i", airQuality);
  
}

void readNTP()
//void readTemp()                            // Rimane solo NTP read cambiare nome!!
{
  unsigned long epoch = getTime(TIMEZONE, DAYLIGHTSAVINGTIME);
  int hours = (epoch  % 86400L) / 3600;
  int minutes = (epoch % 3600) / 60;
  int seconds = (epoch % 60);
  char timeString[8];
  sprintf(timeString, "%02d:%02d:%02d", hours, minutes, seconds);
  BLYNK_LOG("The time is %s", timeString);       // UTC is the time at Greenwich Meridian (GMT)

}

void periodicUpdateThingspeak()
{
  char h_buffer[10];
  char p_buffer[10];
  char t_buffer[10];
  char l_buffer[10];
  char m_buffer[10];                     // aggiunto per contare movimenti
  char a_buffer[10];                     // aggiunto airquality
  char tb_buffer[10];                    // temperatura da BHT
  String humidityString = dtostrf(hum, 5, 2, h_buffer);
  String pressureString = dtostrf(bar, 7, 2, p_buffer);
  String tempString = dtostrf(temperature, 5, 1, t_buffer);
  String airString = dtostrf(airQuality, 4, 1, a_buffer);
  String lightString = dtostrf(ambientLight, 5, 1, l_buffer);
  String movementsString = dtostrf(countMovments, 4, 0, m_buffer);  // aggiunto per contare movimenti
  String temphString = dtostrf(bhttemp, 5, 1, tb_buffer);
  updateThingspeak(thingspeakAPIkey, "field1=" + tempString + "&field2=" + humidityString + "&field3=" + pressureString + "&field4=" + lightString + "&field5=" + movementsString + "&field6=" + airString + "&field7=" + temphString);

}

void connectWiFi(const char* ssid, const char* pass, int timeout)
{
  int timeoutCounter = 0;

  while (WiFi.status() != WL_CONNECTED) {
    timeoutCounter = 0;
    BLYNK_LOG("Connecting to %s", ssid);
    if (pass && strlen(pass)) {
      WiFi.begin(ssid, pass);
    } else {
      WiFi.begin(ssid);
    }

    while ((WiFi.status() != WL_CONNECTED) & (timeoutCounter < timeout * 2)) {
      timeoutCounter += 1;
      ::delay(500);
    }
  }
  BLYNK_LOG("Connected to WiFi");

  IPAddress myip = WiFi.localIP();
  BLYNK_LOG("My IP: %d.%d.%d.%d", myip[0], myip[1], myip[2], myip[3]);

}

void updateThingspeak(String APIkey, String tsData)
{
  WiFiClient client;
  const int httpPort = 80;
  //if (!client.connect("api.thingspeak.com", httpPort)) {
  if (!client.connect("184.106.153.149", httpPort)) {
    BLYNK_LOG("connection to thingspeak failed");
    return;
  }

  client.print("POST /update HTTP/1.1\n");
  client.print("Host: api.thingspeak.com\n");
  client.print("Connection: close\n");
  client.print("X-THINGSPEAKAPIKEY: " + APIkey + "\n");
  client.print("Content-Type: application/x-www-form-urlencoded\n");
  client.print("Content-Length: ");
  client.print(tsData.length());
  client.print("\n\n");
  client.print(tsData);

  client.stop();
  countMovments = 0;                // Reset contatore movimenti dopo invio dati a TS
}

void interruptHandler()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 2000)  // valore originario 2000
  {
    //  Serial.printf("movement detected");
    //  Serial.printf("movement detected\n");          // Modificato
    movementDetected = 1;
    countMovments = countMovments + 1;               // Aggiunto per contare movimenti
    //  Serial.printf(" - count: %d \n", countMovments); // Aggiunto per contare movimenti
  }
  last_interrupt_time = interrupt_time;
}

void startWebServer() {
  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    server.on("/reboot", HTTP_GET, []() {
      server.send(200, "text/plain", "Rebooting!!");
      ESP.restart();
    });
    server.on ( "/", handleRoot );
    server.on ( "/licht.html", handleRoot );
    server.on ( "/data", handleSensorData );
    server.on ( "/on", HTTP_GET, []() {
      //licht_aan();
      server.send(200, "text/plain", "Lights on!!");
    });
    server.on("/version", HTTP_GET, []() {
      server.send ( 200, "text/html", VERSION );
    });
    server.on ( "/off", HTTP_GET, []() {
      //licht_uit();
      server.send(200, "text/plain", "Lights off!!");
    });
    //httpUpdater.setup(&server);
    server.begin();
    MDNS.addService("http", "tcp", 80);

    Serial.printf("Ready! Open http://%s.local in your browser\n", HOSTNAME);
  } else {
    Serial.printf("WiFi Failed");
  }
}

void handleSensorData() {
  char sensorData[256];
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["Temperature"] = temperature;
  json["Humidity"] = hum;
  json["Pressure"] = bar;
  json["ambientLight"] = ambientLight;

  json.prettyPrintTo(sensorData, sizeof(sensorData));
  server.send ( 200, "text/html", sensorData );
}


void readHumiditySensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    BLYNK_LOG("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  Blynk.virtualWrite(V5, h);
  Blynk.run();
  Blynk.virtualWrite(V6, t);
  Blynk.run();
  hum = h;
  bhttemp = t;      // Sostituisce set var temperatura letta in readTemp
  // Print for test
  Serial.print("Humidity: ");
  Serial.print(h);
  //  Serial.print(" %\t");
  Serial.print(" - Temperature: ");
  Serial.print(t);
  Serial.print("*C ");
  Serial.print(" - Heat index: ");
  Serial.print(hic);
  Serial.println("*C ");
  //BLYNK_LOG("Humidity: %E", hum);
}

void readPressure()
{
  char status;
  static double T, P, p0, a;

  // Loop here getting pressure readings every 10 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:


  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P, p0);
        }
        else Serial.println("error retrieving pressure measurement");
      }
      else Serial.println("error starting pressure measurement");
    }
    else Serial.println("error retrieving temperature measurement");
  }
  else Serial.println("error starting temperature measurement");
  Blynk.virtualWrite(V9, int(p0));
  Blynk.run();
  bar = p0;
  Serial.print("Pressure - temp: ");
  Serial.print(bar);
  Serial.print(" - ");
  Serial.println(T);
  temperature = T;       // Sostituisce set var temperatura letta in readTemp
}

void handleRoot() {
  char page[2500];

  if (server.hasArg("keuken") && server.hasArg("gang")) {
    kitchenIntensity = server.arg("keuken").toInt();
    hallIntensity = server.arg("gang").toInt();
  }

  if (server.hasArg("button")) {
    Serial.println(server.arg("button"));
    if (server.arg("button") == "Licht aan") {
      //licht_aan();
    }
    if (server.arg("button") == "Licht uit") {
      //licht_uit();
    }
  }

  snprintf ( page, 2500,

             "<html><head><body><h1>Questa e la pagina dell ESP8266</h1> </body></head> </html>"

             /* Pagina originale he da errore sostituita con uba dummy
              *              "<!DOCTYPE html>\n\
             <html>\n\
             <head>\n\
             <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n\
             <link rel=\"stylesheet\" href=\"http://code.jquery.com/mobile/1.4.5/jquery.mobile-1.4.5.min.css\">\n\
             <script src=\"http://code.jquery.com/jquery-1.11.3.min.js\"></script>\n\
             <script src=\"http://code.jquery.com/mobile/1.4.5/jquery.mobile-1.4.5.min.js\"></script>\n\
             </head>\n\
             <style type=\"text/css\">\n\
                 .ui-header .ui-title {margin: 0 10%}\n\
             </style>\n\
             <body>\n\
             <div data-role=\"page\">\n\
               <div data-role=\"header\">\n\
                 <h1>Huis automatisering V%s</h1>\n\
               </div>\n\
               <div data-role=\"main\" class=\"ui-content\">\n\
                 <form method=\"post\" action=\"licht.html\">\n\
                   <label for=\"keuken\">Keuken dimmer</label>\n\
                   <input type=\"range\" name=\"keuken\" id=\"keuken\" value=\"%d\" min=\"0\" max=\"15\">\n\
                   <label for=\"gang\">Gang dimmer</label>\n\
                   <input type=\"range\" name=\"gang\" id=\"gang\" value=\"%d\" min=\"0\" max=\"15\">\n\
                   <input type=\"submit\" data-inline=\"true\" name=\"button\" value=\"Licht aan\">\n\
                   <input type=\"submit\" data-inline=\"true\" name=\"button\" value=\"Licht uit\">\n\
                 </form>\n\
               </div>\n\
               <div data-role=\"main\" class=\"ui-content\">\n\
               <text id=\"time\"></text><br><br>\n\
               Temperatuur: <text id=\"temperatureField\"></text> C<br>\n\
               Luchtdruk: <text id=\"pressureField\"></text> mb<br>\n\
               Luchtvochtigheid: <text id=\"humidityField\"></text><br>\n\
               Omgevingslicht: <text id=\"ambientLightField\"></text><br>\n\
               </div>\n\
             </div>\n\
             <script>\n\
             setInterval(getData, 10000);\n\
             function getData() {\n\
                     $.getJSON(\"http://%s.local/data\", function(data){\n\
                         var d = new Date();\n\
                         document.getElementById(\"time\").innerHTML=d;\n\
                         document.getElementById(\"temperatureField\").innerHTML=data[\"Temperature\"];\n\
                         document.getElementById(\"humidityField\").innerHTML=data[\"Humidity\"];\n\
                         document.getElementById(\"pressureField\").innerHTML=data[\"Pressure\"];\n\
                         document.getElementById(\"ambientLightField\").innerHTML=data[\"ambientLight\"];\n\
                     });\n\
             }\n\
             getData();\n\
             </script>\n\
             </body>\n\
             </html>"
             Fine pagina originale */

             , VERSION, kitchenIntensity, hallIntensity, HOSTNAME
           );

  server.send ( 200, "text/html", page );
}



