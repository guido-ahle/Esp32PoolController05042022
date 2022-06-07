#include <ArduinoOTA.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <time.h>
#include <WiFiClient.h>
#include <Adafruit_BusIO_Register.h>
#include <PubSubClient.h>
#include <Timedate.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>

// Globales FehlerFlag
boolean result = false;
byte data = 0x00;

// LCD-Display
LiquidCrystal_I2C lcd(0x27, 16, 4);

// Fritz-Box
#define SSID "FRITZ!Box 7590 PE"
#define PSK "66251262094036972079"
#define WIFI_TIME_OUT 20000
#define hostname "Esp32_1"

// MQTT-Broker
const char *mqtt_broker = "192.168.178.98";
const char *mqtt_username = "openhabian";
const char *mqtt_password = "Jona2010";
const int mqtt_port = 1883;

uint64_t chipid = ESP.getEfuseMac(); // MAC address of ESP32
uint16_t chip = (uint16_t)(chipid >> 32);
char clientid[25];

// Temperaturmessung
long intervallTemperatureMeasuring = 20000;
char topicVorlauf[] = "esp32/Pool/Vorlauftemperatur";
char topicRuecklauf[] = "esp32/Pool/Ruecklauftemperatur";
char topicAussen[] = "esp32/Pool/Aussentemperatur";

// Externe Schaltausgäng per MQTT
char topicSwitchFilterpumpe[] = "esp32/Reinigung/Filterpumpe";
char topicSwitchFilterPumpeManuell[] = "esp32/Reinigung/FilterpumpeManuell";
char topicSwitchUnterwasserscheinwerfer[] = "esp32/Beleuchtung/Schweinwerfer";
char topicSwitchVentilSchwalldusche[] = "esp32/Ventil/Schwalldusche";
char topicSwitchVentilRegenwalddusche[] = "esp32/Ventil/Regenwalddusche";
char topicSwitchPumpeDusche[] = "esp32/Pumpe/Duschen";

// Infos per MQTT
char topicInfoFirmware[] = "esp32/Info/FirmwareModseg";
char topicInfoUhrzeit[] = "esp32/Info/MeasurementTime";

// Flags für den MQTT-Server
boolean flagFilterPumpe = false;
boolean flagFilterPumpeManuell = false;
boolean flagUnterwasserscheinwerfer = false;
boolean flagVentilSchwallDusche = false;
boolean flagVentilRegenwaldDusche = false;
boolean flagPumpeDusche = false;
String messageTemp = "";
boolean messageReceived = false;

// konstanten für I2C
const byte allOutPutsLow = 0x00;
byte pcfData = 0x00;

// I2C-Bus
#define pcf_Aktoren 0x21    // Schaltet alle Aktoren
#define pcf_Keys 0x22       // Liest Tastendruck ein
#define pcf_keysStatus 0x23 // Status Leds Keys
byte pcfReadData = 0;
boolean sendMessage = false;

// Eingänge der Temperatursensoren
#define ONE_WIRE_BUS_Vorlauf 5    // GPIO5
#define ONE_WIRE_BUS_Ruecklauf 18 // GPI18
#define ONE_WIRE_BUS_Aussen 19    // GPI19
#define ONE_WIRE_BUS_Reserve 27   // GPIO27

// Init der Temperaturwerte
float temperatureVl = 0.0f;
float temperatureRl = 0.0f;
float temperatureAu = 0.0f;

// Datums- und Zeitausgabe
DateTime now;
struct tm timeInfo;
char timeString[32];
String strMTime;
String timeStampMeasurement = "00:00:00";

// ModSeg-Kennung
#define fwModSeg "01.06.2022 1.0/00"

// Init des NTP-Servers
#define ntpServer "pool.ntp.org"
const long gmOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// Durchflussmesser
float cfactor = 5.0;
byte interrupt = 0;
int FlowSensor = 2;
byte pulseCount;
float flowRate;
unsigned int f_ml;
unsigned long t_ml;
unsigned long oldTime;

// WifiClient
WiFiClient espClient;

// MQTTClient
PubSubClient client(espClient);

char msg[50];
int value = 0;

// Dallas 18B20 Sensoren
OneWire oneWireVorlauf(ONE_WIRE_BUS_Vorlauf);
DallasTemperature sensorVorlauf(&oneWireVorlauf);

OneWire oneWireRuecklauf(ONE_WIRE_BUS_Ruecklauf);
DallasTemperature sensorRuecklauf(&oneWireRuecklauf);

OneWire oneWireAussen(ONE_WIRE_BUS_Aussen);
DallasTemperature sensorAussen(&oneWireAussen);

// StatusLed's auf dem PCBA
int BB_Led = 23; // GPIO 23
boolean bb_on = false;

int ErrorLed = 2; // GPIO2
boolean error_Led = false;

// Init der Zeitkonstanten
unsigned long previousTime = 0;
unsigned long previousTimeMqtt = 0;
unsigned long lastMillis = 0;
unsigned long previousTimeFlowMeter = 0;

// RealTimeClock
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag"};

// The Program starts here
// callback-Funktion für den MQTT-Client
void callback(char *topic, byte *payload, unsigned int length)
{
  messageTemp = "";
  messageReceived = true;
  Serial.println("Messages arrived in topic: ");
  Serial.print("Message [");
  Serial.print(topic);
  Serial.print("]");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  Serial.println();
  Serial.println("-----------------");

  // Auswerten der Nachricht
  // FilterPumpe
  if (String(topic) == topicSwitchFilterpumpe)
  {
    Serial.println("FilterPumpe Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      flagFilterPumpe = true;
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      flagFilterPumpe = false;
    }
  }

  // FilterPumpeManuell
  if (String(topic) == topicSwitchFilterPumpeManuell)
  {
    Serial.println("FilterPumpeManuell Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      flagFilterPumpeManuell = true;
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      flagFilterPumpeManuell = false;
    }
  }

  // Pumpe Dusche
  if (String(topic) == topicSwitchPumpeDusche)
  {
    Serial.println("PumpeDusche Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      flagPumpeDusche = true;
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      flagPumpeDusche = false;
    }
  }

  // Unterwasserscheinwerfer
  if (String(topic) == topicSwitchUnterwasserscheinwerfer)
  {
    Serial.println("Unterwasserscheinwerfer Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      flagUnterwasserscheinwerfer = true;
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      flagUnterwasserscheinwerfer = false;
    }
  }

  // Unterwasserscheinwerfer
  if (String(topic) == topicSwitchVentilRegenwalddusche)
  {
    Serial.println("RegenwaldDusche Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      flagVentilRegenwaldDusche = true;
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      flagVentilRegenwaldDusche = false;
    }
  }

  // Unterwasserscheinwerfer
  if (String(topic) == topicSwitchVentilSchwalldusche)
  {
    Serial.println("Schwalldusche Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      flagVentilSchwallDusche = true;
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      flagVentilSchwallDusche = false;
    }
  }
}

// TemperaturMessung Vorlauf
float getTemperatureFromSensorVl()
{
  float value = 0.0f;
  sensorVorlauf.requestTemperaturesByIndex(0);
  value = sensorVorlauf.getTempCByIndex(0);
  Serial.println("Vl = " + String(value));
  return value;
}

// TemperaturMessung Reucklauf
float getTemperatureFromSensorRl()
{
  float value = 0.0f;
  sensorRuecklauf.requestTemperaturesByIndex(0);
  value = sensorRuecklauf.getTempCByIndex(0);
  Serial.println("Rl = " + String(value));
  return value;
}

// TemperaturMessung Aussentemperatur
float getTemperatureFromSensorAu()
{
  float value = 0.0f;
  sensorAussen.requestTemperaturesByIndex(0);
  value = sensorAussen.getTempCByIndex(0);
  Serial.println("Au = " + String(value));
  return value;
}

// Datenausgabe auf seriellen Port
void writeDataToSerial()
{
  Serial.println("Temperature Vorlauf");
  Serial.print(temperatureVl);
  Serial.print(" C");
  Serial.println("\n");

  Serial.println("Temperature Ruecklauf");
  Serial.print(temperatureRl);
  Serial.print(" C");
  Serial.println("\n");

  Serial.println("Temperature Aussen");
  Serial.print(temperatureAu);
  Serial.print(" C");
  Serial.println("\n");
}

// ModSeg über seriellen Port ausgeben
void writeModSeg()
{
  Serial.println(fwModSeg);
}

// WLAN-Verbindung herstellen
void connectToWifi()
{
  Serial.println("Connecting to Wifi");
  WiFi.mode(WIFI_STA);

  unsigned long startAttemptTime = millis();
  while ((WiFi.status() != WL_CONNECTED) && (millis() - startAttemptTime < WIFI_TIME_OUT))
  {
    WiFi.begin(SSID, PSK);
    Serial.print(".");
    delay(3000);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wifi failed!");
  }
  else
  {
    Serial.println("Connected!");
  }
}

// Uhrzeit ermitteln
void printLocalTime()
{

  if (!getLocalTime(&timeInfo))
  {
    Serial.println("Falied to obtain time");
    return;
  }

  Serial.println(&timeInfo, "%A, %B %d %Y %H:%M:%S");
}

// Datenausgabe auf LCD
void lcdPrintOut()
{
  lcd.setCursor(0, 0);
  lcd.print("Temperatur Vl/RL/Au");

  lcd.setCursor(0, 1);
  lcd.print("VL = ");
  lcd.print(temperatureVl);
  lcd.print(" Grad");

  lcd.setCursor(0, 2);
  lcd.print("RL = ");
  lcd.print(temperatureRl);
  lcd.print(" Grad");

  lcd.setCursor(0, 3);
  lcd.print("Au = ");
  lcd.print(temperatureAu);
  lcd.print(" Grad");

  delay(1000);
}

// Verbindung zum MQTT-Server herstellen
void reconnect()
{
  Serial.println("Starte MQTT-Verbindung");
  int a = 0;
  while (!client.connected())
  {
    Serial.print("Attempting MQTT-Connection");
    if (client.connect(clientid, "openhabian", "Jona2010"))
    {
      Serial.println();
      Serial.println("MQTT-Broker connected!");
      Serial.println("Starte Subscription!");
      client.subscribe(topicSwitchFilterpumpe);
      client.subscribe(topicSwitchPumpeDusche);
      client.subscribe(topicSwitchUnterwasserscheinwerfer);
      client.subscribe(topicSwitchVentilRegenwalddusche);
      client.subscribe(topicSwitchVentilSchwalldusche);
      client.subscribe(topicSwitchFilterPumpeManuell);
    }
    else
    {
      Serial.print("failed, rc = ");
      client.print(client.state());
      Serial.println("Try again in 5 Seconds...");
      delay(5000);
    }

    a++;
  }
}

// Gets the actual Time from RTC.
String getRtcTime()
{
  // Aktuelles Datum holen
  now = rtc.now();
  String strDate = String(now.day()) + "." + String(now.month(), DEC) + "." + String(now.year(), DEC);
  String strTime = String(now.hour(), DEC) + ":" + String(now.minute(), DEC) + ":" + String(now.second(), DEC);
  timeStampMeasurement = strDate + " " + strTime;
  Serial.println("TimeStamp = " + timeStampMeasurement);
  return timeStampMeasurement;
}

// Setup-Routine
void setup()
{
  // DurchlussSensor
  pinMode(FlowSensor, INPUT);

  // Start serieller Port
  Serial.begin(9600);

  // I2C-Display
  Serial.println("Starte Lcd");
  lcd.init();
  lcd.backlight();

  // Starte Sensor
  Serial.println("Starte Sensor VL");
  sensorVorlauf.begin();

  // Starte Sensor
  Serial.println("Starte Sensor Rl");
  sensorRuecklauf.begin();

  // Starte Sensor
  Serial.println("Starte Sensor Aussen");
  sensorAussen.begin();

  // Start Rtc
  if (!rtc.begin())
  {
    Serial.println("Keine RTC gefunden");
    while (1)
      ;
  }
  else
  {
    Serial.println("RTC gestartet.");
  }

  // Connecto to WiFi
  connectToWifi();
  Serial.print("IP-Adress is ");
  Serial.print(WiFi.localIP());
  Serial.println();

  // Init der Status LED's
  pinMode(BB_Led, OUTPUT);
  pinMode(ErrorLed, OUTPUT);

  // Init des NTP-Server/Zeitmanagments
  configTime(gmOffset_sec, daylightOffset_sec, ntpServer);

  // Ausgabe der Uhrzeit
  printLocalTime();

  // Connectiong to MQTT-Broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  // RTC-Setup
  rtc.adjust(DateTime(__DATE__, __TIME__));

  // OTA-Updates
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword("Test123");
  ArduinoOTA.begin();

  getRtcTime();

  // Init des I2C-Bus
  Serial.println("Starte I2C-Bus.");
  Wire.begin();
  Wire.setClock(400000L);
  Wire.beginTransmission(pcf_Aktoren);
  Wire.write(allOutPutsLow);
  Wire.endTransmission();
}

void loop()
{
  // Connect MQTT-Client
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // OTA-Funktionen
  ArduinoOTA.handle();

  // MQTT-Broker
  // Publish a message roughly every x seconds.
  if (millis() - previousTimeMqtt > intervallTemperatureMeasuring)
  {
    previousTimeMqtt = millis();
    int t_BUFFER_SIZE = 5;
    char buffer[t_BUFFER_SIZE];

    // Temperaturmessung & Datenausgabe Vorlauf
    temperatureVl = getTemperatureFromSensorVl();
    temperatureRl = getTemperatureFromSensorRl();
    temperatureAu = getTemperatureFromSensorAu();

    // Ausgabe auf LCD
    lcdPrintOut();

    // Ausgabe auf seriellen POrt
    writeDataToSerial();

    // Publishen der Sensorwerte
    Serial.println("Publish topicVorlauf = " + String(topicVorlauf));
    client.publish(topicVorlauf, dtostrf(temperatureVl, t_BUFFER_SIZE - 1, 2, buffer));

    Serial.println("Publish topicRuecklauf = " + String(topicRuecklauf));
    client.publish(topicRuecklauf, dtostrf(temperatureRl, t_BUFFER_SIZE - 1, 2, buffer));

    Serial.println("Publish topicAussen = " + String(topicAussen));
    client.publish(topicAussen, dtostrf(temperatureAu, t_BUFFER_SIZE - 1, 2, buffer));

    char buft[35];
    Serial.println("Publish topicMeasurementTime = " + String(topicInfoUhrzeit));
    getRtcTime().toCharArray(buft, 35);
    Serial.println("Rtc is " + getRtcTime());
    Serial.print("LocalTime = ");
    Serial.print(&timeInfo, "%A, %B %d %Y %H:%M:%S");
    client.publish(topicInfoUhrzeit, buft);

    Serial.println("Publish topicFirmwareModSeg = " + String(topicInfoFirmware));
    client.publish(topicInfoFirmware, fwModSeg);
  }

  // Schalten der BB-Led
  if ((millis() - previousTime) > 1000)
  {
    previousTime = millis();
    if (bb_on == false)
    {
      digitalWrite(BB_Led, HIGH);
      bb_on = true;
    }
    else if (bb_on == true)
    {
      digitalWrite(BB_Led, LOW);
      bb_on = false;
    }
  }

  if (messageReceived)
  {
    Serial.println("Nachricht erhalten!");
    // Auswertung der Nachrichten
    // Filterpumpe
    if (flagFilterPumpe)
    {
      pcfData = (pcfData | 0x01); // 0000 0001
      Wire.beginTransmission(pcf_Aktoren);
      Serial.println("FilterPumpe ein!");
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();

      exit;
    }
    else if (!flagFilterPumpe)
    {
      pcfData = (pcfData & 0xFE); // 1111 1110
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles aus
      Serial.println("FilterPumpe aus!");
      Wire.endTransmission();
      exit;
    }

    // FilterPumpeManuell
    if (flagFilterPumpeManuell)
    {
      pcfData = (pcfData | 0x02); // 0000 0010
      Wire.beginTransmission(pcf_Aktoren);
      Serial.println("FilterPumpeManuell ein!");
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();
      exit;
    }
    else if (!flagFilterPumpeManuell)
    {
      pcfData = (pcfData & 0xFD); // 1111 1101
      Wire.beginTransmission(pcf_Aktoren);
      Serial.println("FilterPumpeManuell aus!");
      Wire.write(pcfData); // alles aus
      Wire.endTransmission();
      exit;
    }

    if (Serial.available())
    {
      char x = Serial.read();
      switch (x)
      {
      case 'a':
      { // LED 1 schalten
        Wire.beginTransmission(0x21);
        data = (data | 0x01);
        Wire.write(data);
        Wire.endTransmission();
        delay(1500);
        break;
      }
      case 'A':
      {
        Wire.beginTransmission(0x21);
        data = (data & 0xFE);
        Wire.write(data);
        Wire.endTransmission();
        delay(1500);
        break;
      }
      case 'b':
      {
        // LED 2 schalten
        Wire.beginTransmission(0x21);
        data = (data | 0x02);
        Wire.write(data);
        Wire.endTransmission();
        delay(1500);
        break;
      }
      case 'B':
      {
        Wire.beginTransmission(0x21);
        data = (data & 0xFD);
        Wire.write(data);
        Wire.endTransmission();
        delay(1500);
        break;
      }
      }
    }

    // // Pumpe der Duschen
    // if (flagPumpeDusche)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Pumpe Dusche ein!");
    //   Wire.write(0x04); // alles ein
    //   Wire.endTransmission();
    //   exit;
    // }
    // else if (!flagPumpeDusche)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Pumpe Dusche aus!");
    //   Wire.write(0x00); // alles aus
    //   Wire.endTransmission();
    //   exit;
    // }

    // // Unterwasserscheinwerfer
    // if (flagUnterwasserscheinwerfer)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Scheinwerfer ein!");
    //   Wire.write(0x08); // alles ein
    //   Wire.endTransmission();
    //   exit;
    // }
    // else if (!flagPumpeDusche)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Scheinwerfer aus!");
    //   Wire.write(0x00); // alles aus
    //   Wire.endTransmission();
    // }

    // // Ventil RegenwaldDusche
    // if (flagVentilRegenwaldDusche)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Ventil RegenwaldDusche ein!");
    //   Wire.write(0x10); // alles ein
    //   Wire.endTransmission();
    // }
    // else if (!flagPumpeDusche)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Ventil RegenwaldDusche aus!");
    //   Wire.write(0x00); // alles aus
    //   Wire.endTransmission();
    // }

    // // Ventil RegenwaldDusche
    // if (flagVentilSchwallDusche)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Ventil SchwallDusche ein!");
    //   Wire.write(0x40); // alles ein
    //   Wire.endTransmission();
    // }
    // else if (!flagPumpeDusche)
    // {
    //   Wire.beginTransmission(pcf_Aktoren);
    //   Serial.println("Ventil SchwalldDusche aus!");
    //   Wire.write(0x00); // alles aus
    //   Wire.endTransmission();
    // }

    messageReceived = false;
  }

  // Durchflussmesser
  // if ((millis() - previousTimeFlowMeter) > 10000)
  // {
  //   measureFlow();
  // }
}
