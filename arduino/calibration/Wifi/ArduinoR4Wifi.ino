#include "WiFiS3.h"

// Enter your network credentials
char ssid[] = "Fede pedes";    
char pass[] = "WCprut12";   //Comment out if no password

int status = WL_IDLE_STATUS;
WiFiServer server(80);            // Create a server on port 80

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  // Connect to Wi-Fi:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  server.begin();
  printWifiData();
}

void loop() {
  // Listen for incoming clients (web browsers)
  WiFiClient client = server.available();   

  if (client) {
    Serial.println("New client connected.");
    String currentLine = "";
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          // If the client sent a blank line, the HTTP request has ended, so we can send a reply:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK) and a content-type
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // The content of the HTTP response follows:
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            client.println("<h1>Hello from Arduino UNO R4 WiFi!</h1>");
            client.println("<p>Your Wi-Fi connection is working perfectly.</p>");
            client.println("</html>");
            
            // The HTTP response ends with another blank line:
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    // Close the connection:
    client.stop();
    Serial.println("Client disconnected.");
  }
}

void printWifiData() {
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Open your web browser and go to this IP address!");
}