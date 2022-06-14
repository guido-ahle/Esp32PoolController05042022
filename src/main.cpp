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

// ModSeg-Kennung
#define fwModSeg "14.06.2022 1.0/03"

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
long intervallTemperatureMeasuring = 60000;
char topicVorlauf[] = "esp32/Pool/Vorlauftemperatur";
char topicRuecklauf[] = "esp32/Pool/Ruecklauftemperatur";
char topicAussen[] = "esp32/Pool/Aussentemperatur";
char topicReserve[] = "esp32/Pool/Reservetemperatur";

char commandMqtt = 'x';

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
#define ONE_WIRE_BUS_Vorlauf 27   // GPIO5
#define ONE_WIRE_BUS_Ruecklauf 18 // GPI18
#define ONE_WIRE_BUS_Aussen 19    // GPI19
#define ONE_WIRE_BUS_Reserve 5    // GPIO27

// Init der Temperaturwerte
float temperatureVl = 0.0f;
float temperatureRl = 0.0f;
float temperatureAu = 0.0f;
float temperatureReserve = 0.0f;

// Datums- und Zeitausgabe
DateTime now;
struct tm timeInfo;
char timeString[32];
String strMTime;
String timeStampMeasurement = "00:00:00";


// Init des NTP-Servers
#define ntpServer "ptbtime1.ptb.de"
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

OneWire oneWireReserve(ONE_WIRE_BUS_Reserve);
DallasTemperature sensorReserve(&oneWireReserve);

// StatusLed's auf dem PCBA
int BB_Led = 23; // GPIO 23
boolean bb_on = false;

int ErrorLed = 2; // GPIO2
boolean flagError_Led = false;

// Init der Zeitkonstanten
unsigned long previousTime = 0;
unsigned long previousTimeMqtt = 0;
unsigned long previousTimeLcdTime = 0;
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
  commandMqtt = '*';
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
      commandMqtt = 'a';
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      commandMqtt = 'A';
    }
  }

  // FilterPumpeManuell
  if (String(topic) == topicSwitchFilterPumpeManuell)
  {
    Serial.println("FilterPumpeManuell Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      commandMqtt = 'b';
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      commandMqtt = 'B';
    }
  }

  // Pumpe Dusche
  if (String(topic) == topicSwitchPumpeDusche)
  {
    Serial.println("PumpeDusche Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      commandMqtt = 'c';
    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      commandMqtt = 'C';

    }
  }

  // Unterwasserscheinwerfer
  if (String(topic) == topicSwitchUnterwasserscheinwerfer)
  {
    Serial.println("Unterwasserscheinwerfer Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      commandMqtt = 'd';

    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      commandMqtt = 'D';

    }
  }

  // Unterwasserscheinwerfer
  if (String(topic) == topicSwitchVentilRegenwalddusche)
  {
    Serial.println("RegenwaldDusche Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
      commandMqtt = 'e';

    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      commandMqtt = 'E';

    }
  }

  // Unterwasserscheinwerfer
  if (String(topic) == topicSwitchVentilSchwalldusche)
  {
    Serial.println("Schwalldusche Message!");
    if (messageTemp == "On")
    {
      Serial.println("On");
            commandMqtt = 'f';

    }
    else if (messageTemp == "Off")
    {
      Serial.println("Off");
      commandMqtt = 'F';

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

// TemperaturMessung Reserve
float getTemperatureFromSensorReserve()
{
  float value = 0.0f;
  sensorReserve.requestTemperaturesByIndex(0);
  
  value = sensorReserve.getTempCByIndex(0);
  Serial.println("Reserve = " + String(value));
  return value;
}

// Datenausgabe auf seriellen Port
void writeDataToSerial()
{
  Serial.println("Temperature Vorlauf");
  Serial.print(temperatureVl);
  Serial.print(" °C");
  Serial.println("\n");

  Serial.println("Temperature Ruecklauf");
  Serial.print(temperatureRl);
  Serial.print(" °C");
  Serial.println("\n");

  Serial.println("Temperature Aussen");
  Serial.print(temperatureAu);
  Serial.print(" °C");
  Serial.println("\n");

  Serial.println("Temperature Reserve");
  Serial.print(temperatureReserve);
  Serial.print(" °C");
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
void lcdPrintOut(String text1, String text2, String text3, String text4)
{
  
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(text1);

  lcd.setCursor(0, 1);
  lcd.print(text2);

  lcd.setCursor(0, 2);
  lcd.print(text3);

  lcd.setCursor(0, 3);
  lcd.print(text4);
  delay(1000);
  
}

// Verbindung zum MQTT-Server herstellen
void reconnect()
{

  int maxRetries = 5;
  Serial.println("Starte MQTT-Verbindung");
  int a = 0;
  while ((!client.connected()) || (a <= maxRetries))
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
      break;
    }
    else
    {
      Serial.println("failed, rc = ");
      client.print(client.state());
      Serial.println("Try again in 5 Seconds...");
      delay(10000);
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
  return (strDate + " " + strTime);
}

// Setup-Routine
void setup()
{
  // DurchlussSensor
  pinMode(FlowSensor, INPUT);

  // Init der Status LED's
  pinMode(BB_Led, OUTPUT);
  digitalWrite(BB_Led, HIGH); // BB Led aus!

  pinMode(ErrorLed, OUTPUT);
  digitalWrite(ErrorLed, HIGH); // ErrorLed aus!

  // Start serieller Port
  Serial.begin(9600);

  // I2C-Display
  Serial.println("Starte Lcd");
  lcd.init();
  lcd.backlight();
  lcdPrintOut("LCD is started1", "", "", "");

  // Starte Sensor Vorlauf
  Serial.println("Starte Sensor VL");
  lcdPrintOut("Start Sensor VL", "", "", "");
  sensorVorlauf.begin();
  sensorVorlauf.setResolution(10);
  sensorVorlauf.setWaitForConversion(true);
  if (sensorVorlauf.getDeviceCount() == 1)
  {
    flagError_Led = false;
  }
  else
  {
    flagError_Led = true;
  }

  // Starte Sensor Rücklauf
  Serial.println("Starte Sensor Rl");
  lcdPrintOut("Start Sensor RL", "", "", "");
  sensorRuecklauf.begin();
  sensorRuecklauf.setResolution(10);
  sensorRuecklauf.setWaitForConversion(true);
  if (sensorRuecklauf.getDeviceCount() == 1)
  {
    flagError_Led = false;
  }
  else
  {
    flagError_Led = true;
  }

  // Starte Sensor Aussen
  Serial.println("Starte Sensor Aussen");
  lcdPrintOut("Start Sensor AU", "", "", "");
  sensorAussen.begin();
  sensorAussen.setResolution(10);
  sensorAussen.setWaitForConversion(true);
  if (sensorAussen.getDeviceCount() == 1)
  {
    flagError_Led = false;
  }
  else
  {
    flagError_Led = true;
  }

  // Starte Sensor Reserve
  Serial.println("Starte Sensor Reserve");
  lcdPrintOut("Start Sensor RT", "", "", "");
  sensorReserve.begin();
  sensorReserve.setResolution(10);
  sensorReserve.setWaitForConversion(true);
  if (sensorReserve.getDeviceCount() == 1)
  {
    flagError_Led = false;
  }
  else
  {
    flagError_Led = true;
  }

  // Start Rtc
  lcdPrintOut("Start RTC!", "Wait...", "", "");
  if (!rtc.begin())
  {
    Serial.println("Keine RTC gefunden");
    lcdPrintOut("Found no RTC!", "", "", "");
    while (1)
      ;
  }
  else
  {
    Serial.println("RTC gestartet.");
    lcdPrintOut("RTC started!", "", "", "");
  }

  // Connecto to WiFi
  lcdPrintOut("Start Wifi", "Wait...", "", "");
  connectToWifi();
  Serial.print("IP-Adress is ");
  Serial.print(WiFi.localIP());
  lcdPrintOut("WiFi strted", "IP-Adress", String(WiFi.localIP()), "");
  Serial.println();

  // Init des NTP-Server/Zeitmanagments
  lcdPrintOut("Config NTP-server", "Wait...", "", "");
  configTime(gmOffset_sec, daylightOffset_sec, ntpServer);

  // Ausgabe der Uhrzeit
  printLocalTime();

  // Connectiong to MQTT-Broker
  lcdPrintOut("Start MQTT", "Wait...", "", "");
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  // RTC-Setup
  lcdPrintOut("Start RTC-Adjust", "Wait...", "", "");
  getLocalTime(&timeInfo);
  rtc.adjust(DateTime(__DATE__, __TIME__));
  getRtcTime();

  // OTA-Updates
  lcdPrintOut("Start OTA", "Wait...", "", "");
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword("Test123");
  ArduinoOTA.begin();

  // Init des I2C-Bus
  Serial.println("Starte I2C-Bus.");
  lcdPrintOut("Start I2c-Bus", "Wait...", "", "");
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
    temperatureReserve = getTemperatureFromSensorReserve();

    // Ausgabe auf LCD
    lcdPrintOut("VL : " + String(temperatureVl), "RL : " + String(temperatureRl), "AU : " + String(temperatureAu), "RT : " + String(temperatureReserve));

    // Ausgabe auf seriellen POrt
    writeDataToSerial();

    // Publishen der Sensorwerte
    Serial.println("Publish topicVorlauf = " + String(topicVorlauf));
    client.publish(topicVorlauf, dtostrf(temperatureVl, t_BUFFER_SIZE - 1, 2, buffer));

    Serial.println("Publish topicRuecklauf = " + String(topicRuecklauf));
    client.publish(topicRuecklauf, dtostrf(temperatureRl, t_BUFFER_SIZE - 1, 2, buffer));

    Serial.println("Publish topicAussen = " + String(topicAussen));
    client.publish(topicAussen, dtostrf(temperatureAu, t_BUFFER_SIZE - 1, 2, buffer));

    Serial.println("Publish topicReserve = " + String(topicReserve));
    client.publish(topicReserve, dtostrf(temperatureReserve, t_BUFFER_SIZE - 1, 2, buffer));

    char buft[35];
    Serial.println("Publish topicMeasurementTime = " + String(topicInfoUhrzeit));
    getRtcTime().toCharArray(buft, 35);
    Serial.println("Rtc is " + getRtcTime());
    Serial.print("LocalTime = ");
    Serial.println(&timeInfo, "%A, %B %d %Y %H:%M:%S");
    client.publish(topicInfoUhrzeit, buft);

    Serial.println("Publish topicFirmwareModSeg = " + String(topicInfoFirmware));
    client.publish(topicInfoFirmware, fwModSeg);
  }

//Einfach mal die Uhrzeit auf dem Display ausgeben
if ((millis() - previousTimeLcdTime) > 5000)
  {
    previousTimeLcdTime = millis();
    char buf[20];
    String timeForLcd = getRtcTime();
    lcdPrintOut("Messzeit", timeForLcd, fwModSeg,"");
  }

  // Schalten der BB-Led
  // Setzen der Error-LED
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

    if (flagError_Led = true)
    {
      digitalWrite(ErrorLed, LOW);
    }
    else
    {
      digitalWrite(ErrorLed, HIGH);
    }
  }

  if (messageReceived)
  {
    Serial.println("Nachricht erhalten!");
    // Auswertung der Nachrichten
    // Filterpumpe
    if (commandMqtt == 'a')
    {
      lcdPrintOut("FilterPumpe ein!", "Wait..", "", "");
      Serial.println("FilterPumpe ein!");
      pcfData = (pcfData | 0x01); // 0000 0001
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();
    }

    if (commandMqtt == 'A')
    {
      lcdPrintOut("FilterPumpe aus!", "Wait..", "", "");
      Serial.println("FilterPumpe aus!");
      pcfData = (pcfData & 0xFE); // 1111 1110
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles aus
      Wire.endTransmission();
    }

    // FilterPumpeManuell
    if (commandMqtt == 'b')
    {
      Serial.println("FilterPumpeManuell ein!");
      lcdPrintOut("FilterPumpeManuell ein!", "Wait..", "", "");
      pcfData = (pcfData | 0x02); // 0000 0010
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();
    }

    if (commandMqtt == 'B')
    {
      Serial.println("FilterPumpeManuell aus!");
      lcdPrintOut("FilterPumpeManuell aus!", "Wait..", "", "");
      pcfData = (pcfData & 0xFD); // 1111 1101
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles aus
      Wire.endTransmission();
      messageReceived = false;
    }

    // PumpeDusche
    if (commandMqtt == 'c')
    {
      Serial.println("PumpeDusche ein!");
      lcdPrintOut("PumpeDusche ein!", "Wait..", "", "");
      pcfData = (pcfData | 0x04); // 0000 0100
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();
    }

    if (commandMqtt == 'C')
    {
      Serial.println("PumpeDusche aus!");
      lcdPrintOut("PumpeDusche aus!", "Wait..", "", "");
      pcfData = (pcfData & 0xFC); // 1111 1011
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles aus
      Wire.endTransmission();
      messageReceived = false;
    }

    // Unterwasserscheinwerfer
    if (commandMqtt == 'd')
    {
      Serial.println("Unterwasserscheinwerfer ein!");
      lcdPrintOut("Uw-Scheinwerfer ein!", "Wait..", "", "");
      pcfData = (pcfData | 0x08); // 0000 1000
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();
    }

    if (commandMqtt == 'D')
    {
      Serial.println("Unterwasserscheinwerfer aus!");
      lcdPrintOut("Uw-Scheinwerfer aus!", "Wait..", "", "");
      pcfData = (pcfData & 0xFB); // 1111 0111
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles aus
      Wire.endTransmission();
    }

    // Regenwalddusche
    if (commandMqtt == 'e')
    {
      Serial.println("Regenwalddusche ein!");
      lcdPrintOut("Regenwalddusche ein!", "Wait..", "", "");
      pcfData = (pcfData | 0x14); // 0000 1000
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();
    }

    if (commandMqtt == 'E')
    {
      Serial.println("Regenwalddusche aus!");
      lcdPrintOut("Regenwalddusche aus!", "Wait..", "", "");
      pcfData = (pcfData & 0xEB); // 1111 0111
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles aus
      Wire.endTransmission();
    }

    // Regenwalddusche
    if (commandMqtt == 'f')
    {
      Serial.println("Schwalldusche ein!");
      lcdPrintOut("Schwalldusche ein!", "Wait..", "", "");
      pcfData = (pcfData | 0x24); // 0000 1000
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles ein
      Wire.endTransmission();
    }

    if (commandMqtt == 'F')
    {
      Serial.println("Schwalldusche aus!");
      lcdPrintOut("Schwalldusche aus!", "Wait..", "", "");
      pcfData = (pcfData & 0xDB); // 1111 0111
      Wire.beginTransmission(pcf_Aktoren);
      Wire.write(pcfData); // alles aus
      Wire.endTransmission();
    }

      messageReceived = false;

  };
    // Test mit seriellem Monitor
  if (Serial.available())
  {
    char x = Serial.read();
    switch (x)
    {
    case 'a':
    { // LED 1 schalten
      Wire.beginTransmission(pcf_Aktoren);
      data = (data | 0x01);
      Wire.write(data);
      Wire.endTransmission();
      delay(1500);
      break;
    }
    case 'A':
    {
      Wire.beginTransmission(pcf_Aktoren);
      data = (data & 0xFE);
      Wire.write(data);
      Wire.endTransmission();
      delay(1500);
      break;
    }
    case 'b':
    {
      // LED 2 schalten
      Wire.beginTransmission(pcf_Aktoren);
      data = (data | 0x02);
      Wire.write(data);
      Wire.endTransmission();
      delay(1500);
      break;
    }
    case 'B':
    {
      Wire.beginTransmission(pcf_Aktoren);
      data = (data & 0xFD);
      Wire.write(data);
      Wire.endTransmission();
      delay(1500);
      break;
    }
    }
  }

  }
