#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Genies1";
const char* password = "######";
const char* mqtt_server = "192.168.178.13";
const int mqtt_port = 1883;  // Standard MQTT-Port

// MQTT-Zugangsdaten
const char* mqtt_user = "mqttuser";
const char* mqtt_password = "######";

// MQTT-Themen
const char* temp_topic = "home/esp32/temperature";
const char* relay_topic = "home/esp32/relay";
const char* fan_topic = "home/esp32/fantacho";
const char* water_topic = "home/esp32/water";
const char* command_topic = "home/esp32/relay/set";
const char* fan_cmd_topic = "home/esp32/fantacho/pwm";


WiFiClient espClient;
PubSubClient client(espClient);

// Steuerungs-Pins
#define RELAY_O          4  // GPIO04 für Relais Oben
#define RELAY_U          5  // GPIO05 für Relais Unten
#define RELAY_I          6  // GPIO06 für Relais Innen
#define RELAY_ND         7  // GPIO07 für Relais Undefiniert
                        
#define FAN_PWM_O       42  // GPIO42 für PWM-Signal an Lüfter
#define FAN_PWM_U       37  // GPIO37 für PWM-Signal an Lüfter
#define FAN_PWM_I       46  // GPIO46 für PWM-Signal an Lüfter

#define FAN_TACHO_O     41  // GPIO41 für Tacho-Signal an Lüfter
#define FAN_TACHO_U     36  // GPIO36 für Tacho-Signal an Lüfter
#define FAN_TACHO_I     47  // GPIO47 für Tacho-Signal an Lüfter

// Wasserstand-Pin
#define WASSER_PIN_HALB 15  // GPIO15 für WasserSensor Oben (weiß)  
#define WASSER_PIN_LEER 16  // GPIO16 für WasserSensor Unten (Grün)  

// DHT11-Sensor Einstellungen
#define DHT_PIN_O       10  // GPIO04 für DHT11
#define DHT_PIN_U       18  // GPIO04 für DHT11
#define DHTTYPE DHT11       // Nutzung der DHT11 Bibliothek

DHT dht_oben(DHT_PIN_O, DHTTYPE);
DHT dht_unten(DHT_PIN_U, DHTTYPE);

// PWM-Einstellungen für ESP32 (LEDC-Kanal)
#define PWM_FREQ 12500 // 25 kHz für Lüftersteuerung
#define PWM_RESOLUTION 8  // 10 Bit Auflösung (0-1023)

// Wifi init
void setup_wifi() {
  delay(10);
  Serial.println("Verbinde mit WLAN...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nVerbunden mit WLAN!");
}

// Schnittstelle Befehle
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Nachricht empfangen: ");
  Serial.println(topic);
  if (strcmp(topic, command_topic) == 0) {
    if ((char)payload[0] == '1') {
      digitalWrite(RELAY_O, HIGH);
      client.publish(relay_topic, "1");  // Relaisstatus aktualisieren
    } else {
      digitalWrite(RELAY_O, LOW);
      client.publish(relay_topic, "0");
    }
  }
}

// Schnelle neu verbindung bei dc's
void reconnect() {
  while (!client.connected()) {
    Serial.print("Verbindung zu MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Verbunden!");
      client.subscribe(command_topic);  // Abonniert Steuerungsnachricht
    } else {
      Serial.print("Fehlgeschlagen, rc=");
      Serial.print(client.state());
      Serial.println(" Warte 5 Sekunden...");
      delay(5000);
    }
  }
}

void setup() {
    client.setServer(mqtt_server, mqtt_port);  // MQTT-Server mit Port verbinden
    client.setCallback(callback);
    Serial.begin(115200);
    setup_wifi();

    Serial.begin(115200);
    Serial.println("ESP32-S3 GrowTardis WLAN TEST");

    // Pins als Ausgang setzen
    pinMode(RELAY_O, OUTPUT);
    pinMode(RELAY_U, OUTPUT);
    pinMode(WASSER_PIN_HALB, INPUT);
    pinMode(WASSER_PIN_LEER, INPUT);
    digitalWrite(RELAY_O, HIGH);
    digitalWrite(RELAY_U, HIGH);

    // DHT11 starten
    dht_oben.begin();
    dht_unten.begin();

    // Ventilator Steuerung initialisieren.
    ledcAttach(FAN_PWM_O, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(FAN_PWM_O, 64);  // 50% Leistung (255 max)
}

void loop() {
    // Temperatur & Feuchtigkeit auslesen
    float humidity_oben = dht_oben.readHumidity();
    float temperature_oben = dht_oben.readTemperature();
    float humidity_unten = dht_unten.readHumidity();
    float temperature_unten = dht_unten.readTemperature();


    // Sensor Funktionsfähigkeit Test
    if (isnan(humidity_oben) || isnan(temperature_oben)) {
        Serial.println("Fehler beim Lesen des DHT-Sensors Oben!");
    } else if (isnan(humidity_unten) || isnan(temperature_unten)) {
        Serial.println("Fehler beim Lesen des DHT-Sensors Unten!");
    }
    //Wasserstandmessung
    bool wasserHalb, wasserLeer;
    if (digitalRead(WASSER_PIN_HALB) == LOW) {wasserHalb = false;} else {wasserHalb = true;}
    if (digitalRead(WASSER_PIN_LEER) == LOW) {wasserLeer = false;} else {wasserLeer = true;}
    String wasserstand;
    if (!wasserHalb && !wasserLeer) {wasserstand = "WASSER VOLL";}
    else if (!wasserHalb && wasserLeer) {wasserstand = "WASSER HALB";}
    else if (wasserHalb && wasserLeer) {wasserstand = "WASSER LEER";}
    else {wasserstand = "WASSER KAPOTT";}

    // Serielle Eingaben für Debugging
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        // Relais 1 steuern
        if (input.equals("R11")) {
            Serial.println("Relais 1 EIN");
            digitalWrite(RELAY_O, LOW);
        } else if (input.equals("R10")) {
            Serial.println("Relais 1 AUS");
            digitalWrite(RELAY_O, HIGH);
        }
        // Relais 2 steuern
        else if (input.equals("R21")) {
            Serial.println("Relais 2 EIN");
            digitalWrite(RELAY_U, LOW);
        } else if (input.equals("R20")) {
            Serial.println("Relais 2 AUS");
            digitalWrite(RELAY_U, HIGH);
        }
        // Lüfter PWM einstellen
        else if (input.equals("PWM")) {
            Serial.println("PWM einstellen (0-255):");
            while (Serial.available() == 0) {}            // Warten auf Eingabe
            String input1 = Serial.readStringUntil('\n');
            input1.trim();
            uint32_t drehTakt = input1.toInt();           // Eingabe in Integer umwandeln
            if (drehTakt >= 0 && drehTakt <= 255) {
                ledcWrite(FAN_PWM_O, drehTakt);
                Serial.print("PWM auf ");
                Serial.print(drehTakt);
                Serial.println(" Duty-Cicle gesetzt.");
            } else {
                Serial.println("Ungültige Eingabe! Bitte 0-255 eingeben.");
            }
        }
    }

    // MQTT Verbindung sicherstellen
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Temperatur & Feuchtigkeit als MQTT-Nachricht senden
    char temp_oben_str[8], temp_unten_str[8], hum_oben_str[8], hum_unten_str[8];

    dtostrf(temperature_oben, 1, 2, temp_oben_str);
    dtostrf(temperature_unten, 1, 2, temp_unten_str);
    dtostrf(humidity_oben, 1, 2, hum_oben_str);
    dtostrf(humidity_unten, 1, 2, hum_unten_str);

    client.publish("home/esp32/temp_oben", temp_oben_str);
    client.publish("home/esp32/temp_unten", temp_unten_str);
    client.publish("home/esp32/hum_oben", hum_oben_str);
    client.publish("home/esp32/hum_unten", hum_unten_str);

    // Wasserstand senden
    client.publish(water_topic, wasserstand.c_str());

    // Relaisstatus senden
    char relay1State[2], relay2State[2];
    sprintf(relay1State, "%d", digitalRead(RELAY_O));
    sprintf(relay2State, "%d", digitalRead(RELAY_U));

    client.publish("home/esp32/relay1", relay1State);
    client.publish("home/esp32/relay2", relay2State);


    delay(5000);
    // Seriell: Sensorwerte ausgeben
    Serial.print("Relay 1 ist ");
    Serial.print(digitalRead(RELAY_O) ? "AUS" : "AN");
    Serial.print("  |  Relay 2 ist ");
    Serial.print(digitalRead(RELAY_U) ? "AUS" : "AN");
    Serial.print("  |  Feuchtigkeit:  ");
    Serial.print(humidity_oben);
    Serial.print("% | ");
    Serial.print(humidity_unten);
    Serial.print("%  |  Temperatur: ");
    Serial.print(temperature_oben);
    Serial.print("°C | ");
    Serial.print(temperature_unten);
    Serial.print("°C  |  ");
    Serial.println(wasserstand);
    delay(1000);
}
