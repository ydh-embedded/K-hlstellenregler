/*
 * EK-3030E Modbus TCP Data Logger via Waveshare Gateway
 * 
 * Liest alle verfuegbaren Register des Elitech EK-3030E Kuehlstellenreglers
 * ueber ein Waveshare RS485-zu-Ethernet Gateway (Modbus TCP) aus und speichert 
 * die Werte als CSV-Datei auf einer SD-Karte.
 * 
 * Benoetigte Bibliotheken (im Bibliotheks-Manager installieren):
 * - Ethernet (fuer W5100 / W5500 Shields)
 * - ArduinoModbus (von Arduino)
 * - ArduinoRS485 (wird von ArduinoModbus benoetigt, auch bei TCP)
 * - SD (Standard)
 * - SPI (Standard)
 * 
 * Hardware-Verkabelung:
 * - Arduino mit Ethernet-Shield (W5100/W5500)
 * - SD-Kartenmodul: CS=4 (Standard beim Ethernet Shield, anpassen falls abweichend)
 * 
 * Waveshare Gateway Konfiguration:
 * - Modus: Modbus TCP <-> RTU
 * - Standard IP: 192.168.1.200
 * - Modbus Port: 502
 */

#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoModbus.h>
#include <SD.h>

// --- Konfiguration SD-Karte ---
const int chipSelect = 4; // CS Pin fuer SD-Karte (Oft Pin 4 beim Ethernet Shield)
const char* logFileName = "EK3030E.csv";

// --- Konfiguration Netzwerk & Modbus TCP ---
// MAC-Adresse des Ethernet-Shields (auf Aufkleber, oder eine erfinden)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// IP-Adresse des Arduino (muss im gleichen Subnetz wie das Gateway sein)
IPAddress ip(192, 168, 1, 199);

// IP-Adresse des Waveshare Gateways (Standard: 192.168.1.200)
IPAddress gatewayIP(192, 168, 1, 200);
const int gatewayPort = 502; // Standard Modbus TCP Port

// Modbus-Adresse des EK-3030E (Parameter H17 am Regler)
const int EK3030E_ADDRESS = 1;

EthernetClient ethClient;
ModbusTCPClient modbusTCPClient(ethClient);

// Intervall fuer die Datenabfrage in Millisekunden (z.B. 10000 = 10 Sekunden)
const unsigned long LOG_INTERVAL = 10000;
unsigned long lastLogTime = 0;

// --- Variablen fuer die ausgelesenen Daten ---
float cabinetTemp = 0.0;
float defrostTemp = 0.0;
uint16_t softwareVersion = 0;

float onTempCooling = 0.0;
float offTempCooling = 0.0;
uint16_t defrostTime = 0;
uint16_t defrostCycle = 0;

uint16_t relayStatus = 0;
uint16_t alarmStatus = 0;
uint16_t deviceStatus = 0;

// Hilfsfunktion zur Umwandlung von vorzeichenbehafteten 16-Bit Werten (Temperatur)
float decodeTemperature(uint16_t rawValue) {
  int16_t signedValue = (int16_t)rawValue;
  return signedValue / 10.0;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println(F("EK-3030E Modbus TCP Logger startet..."));

  // SD-Karte initialisieren
  Serial.print(F("Initialisiere SD-Karte... "));
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Fehler bei SD-Karten Initialisierung!"));
  } else {
    Serial.println(F("Erfolgreich."));
    if (!SD.exists(logFileName)) {
      File dataFile = SD.open(logFileName, FILE_WRITE);
      if (dataFile) {
        dataFile.println(F("Timestamp(ms),CabinetTemp_C,DefrostTemp_C,SW_Version,OnTempCooling_C,OffTempCooling_C,DefrostTime_min,DefrostCycle_h,RelayStatus_Raw,AlarmStatus_Raw,DeviceStatus_Raw"));
        dataFile.close();
        Serial.println(F("CSV Header geschrieben."));
      }
    }
  }

  // Ethernet initialisieren
  Serial.println(F("Initialisiere Ethernet..."));
  Ethernet.begin(mac, ip);
  
  // Kurze Pause, damit das Shield hochfahren kann
  delay(1500);
  
  Serial.print(F("Arduino IP: "));
  Serial.println(Ethernet.localIP());
}

void loop() {
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = millis();
    
    // Sicherstellen, dass die Modbus TCP Verbindung steht
    if (!modbusTCPClient.connected()) {
      Serial.println(F("Verbinde zum Modbus Gateway..."));
      if (modbusTCPClient.begin(gatewayIP, gatewayPort)) {
        Serial.println(F("Verbunden!"));
      } else {
        Serial.println(F("Verbindung fehlgeschlagen!"));
        return; // Abbruch, naechster Versuch im naechsten Intervall
      }
    }

    readModbusData();
    logDataToSD();
    printDataToSerial();
  }
}

void readModbusData() {
  // 1. Block: Messwerte (0x0100 bis 0x0102 -> 3 Register)
  if (modbusTCPClient.requestFrom(EK3030E_ADDRESS, HOLDING_REGISTERS, 0x0100, 3)) {
    cabinetTemp     = decodeTemperature(modbusTCPClient.read());
    defrostTemp     = decodeTemperature(modbusTCPClient.read());
    softwareVersion = modbusTCPClient.read();
  } else {
    Serial.print(F("Fehler Block 1: "));
    Serial.println(modbusTCPClient.lastError());
  }

  // 2. Block: Wichtige Einstellungen (0x0400 bis 0x0405 -> 6 Register)
  if (modbusTCPClient.requestFrom(EK3030E_ADDRESS, HOLDING_REGISTERS, 0x0400, 6)) {
    onTempCooling   = decodeTemperature(modbusTCPClient.read());
    offTempCooling  = decodeTemperature(modbusTCPClient.read());
    modbusTCPClient.read(); // On Temp Heating (ueberspringen)
    modbusTCPClient.read(); // Off Temp Heating (ueberspringen)
    defrostTime     = modbusTCPClient.read();
    defrostCycle    = modbusTCPClient.read();
  } else {
    Serial.print(F("Fehler Block 2: "));
    Serial.println(modbusTCPClient.lastError());
  }

  // 3. Block: Status Register (0x0800 bis 0x0802 -> 3 Register)
  if (modbusTCPClient.requestFrom(EK3030E_ADDRESS, HOLDING_REGISTERS, 0x0800, 3)) {
    relayStatus     = modbusTCPClient.read();
    modbusTCPClient.read(); // Digital Input (ueberspringen)
    alarmStatus     = modbusTCPClient.read();
  } else {
    Serial.print(F("Fehler Block 3: "));
    Serial.println(modbusTCPClient.lastError());
  }

  // 4. Block: Geraetestatus (0x0A00 -> 1 Register)
  if (modbusTCPClient.requestFrom(EK3030E_ADDRESS, HOLDING_REGISTERS, 0x0A00, 1)) {
    deviceStatus = modbusTCPClient.read();
  } else {
    Serial.print(F("Fehler Block 4: "));
    Serial.println(modbusTCPClient.lastError());
  }
}

void logDataToSD() {
  File dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.print(millis()); dataFile.print(",");
    dataFile.print(cabinetTemp, 1); dataFile.print(",");
    dataFile.print(defrostTemp, 1); dataFile.print(",");
    dataFile.print(softwareVersion); dataFile.print(",");
    dataFile.print(onTempCooling, 1); dataFile.print(",");
    dataFile.print(offTempCooling, 1); dataFile.print(",");
    dataFile.print(defrostTime); dataFile.print(",");
    dataFile.print(defrostCycle); dataFile.print(",");
    dataFile.print(relayStatus, BIN); dataFile.print(",");
    dataFile.print(alarmStatus, BIN); dataFile.print(",");
    dataFile.println(deviceStatus, BIN);
    dataFile.close();
  } else {
    Serial.println(F("Fehler beim Schreiben auf SD-Karte!"));
  }
}

void printDataToSerial() {
  Serial.println(F("--- EK-3030E Modbus TCP Daten ---"));
  Serial.print(F("Kuehlraum-Temp: ")); Serial.print(cabinetTemp, 1); Serial.println(F(" C"));
  Serial.print(F("Abtau-Temp:     ")); Serial.print(defrostTemp, 1); Serial.println(F(" C"));
  
  bool compOn = !(relayStatus & 0x01);
  bool heatOn = !(relayStatus & 0x02);
  bool fanOn  = !(relayStatus & 0x04);
  
  Serial.print(F("Kompressor: ")); Serial.println(compOn ? "AN" : "AUS");
  Serial.print(F("Luefter:    ")); Serial.println(fanOn ? "AN" : "AUS");
  Serial.print(F("Abtauung:   ")); Serial.println(heatOn ? "AN" : "AUS");
  
  if (alarmStatus > 0) {
    Serial.print(F("WARNUNG! Alarm-Status: ")); 
    Serial.println(alarmStatus, BIN);
  }
  Serial.println(F("---------------------------------"));
}
