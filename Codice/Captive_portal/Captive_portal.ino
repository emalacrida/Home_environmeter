/* Home Environment monitor station
 *  
 * Test di connetività in biblioteca
 * 
 */

//#define BLYNK_DEBUG           // slowdown x10 operation more detailed print
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <NTPtime.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
//#include <WiFiClient.h>
//#include <WiFiServer.h>
//#include <SPI.h>

// Specific installation setup parameters ======================

#define SID  "BiblioWiFi"     // wifi bilioteca
//#define SID  "em-AndroidAP"   // Telefono
#define PAS  ""

//#define CAFELIBSERVER "10.10.55.1"     // Busto
#define CAFELIBSERVER "10.10.56.1"     // Canegrate
//#define CAFELIBSERVER "10.10.59.1"     // Cinisello
#define CAFELIBNAME  "11111"
#define CAFELIBPASSWD  "11111"

#define HOSTNAME "bibliot"
#define VERSION "0.012"

// End specific installation setup parameters ==================

ESP8266WebServer server(80);
//ESP8266HTTPconnectWiFiUpdateServer httpUpdater;      //?????

void connectWiFi(const char* ssid = SID, const char* pass = PAS, int timeout = 10);
void autenticate(const char* server = CAFELIBSERVER, const char* userid = CAFELIBNAME, const char* password = CAFELIBPASSWD, int timeout = 10);
void startWebServer();


void setup()
{
  Serial.begin(115200);
  connectWiFi();
  startWebServer();
  Serial.println("adesso mi autentico in Cafelib");
  autenticate();  
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
}

void connectWiFi(const char* ssid, const char* pass, int timeout)
{
  int timeoutCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    timeoutCounter = 0;
    Serial.print("Connecting to: ");
    Serial.println(ssid);
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
  Serial.print("Connected to: ");
  Serial.print(ssid);
  IPAddress myip = WiFi.localIP();
  Serial.print(" - with IP address: ");
  Serial.println(myip);

}

void autenticate(const char* server, const char* username, const char* userpasswd, int timeout) 
{
  Serial.println("Qui si autentica l'utente in Cafelib");

/*   
client.println("POST / HTTP/1.1");
// $PORTAL_HOST should be the host of the captive portal, e.g. 10.1.1.1
client.println("Host: $PORTAL_HOST");
client.println("Content-Type: application/x-www-form-urlencoded");
client.println("Content-Length: 8");
client.print("\n");
client.print("Answer=1");
 */
}

void startWebServer() {
  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    server.on("/reboot", HTTP_GET, []() {
      server.send(200, "text/plain", "Rebooting!!");
      ESP.restart();
    });
    server.on ( "/", handleRoot );

    server.begin();
    MDNS.addService("http", "tcp", 80);

    Serial.printf("Ready! Open http://%s.local in your browser\n", HOSTNAME);
  } else {
    Serial.printf("WiFi Failed");
  }
}


void handleRoot() {
  char page[2500];

  snprintf ( page, 2500,
             "<html><head><body><h1>Questa e la pagina dell ESP8266</h1> </body></head> </html>"
             , "VERSION", "kitchenIntensity", HOSTNAME
           );
  server.send ( 200, "text/html", page );
}



