/*
 * EK-3030E Modbus RTU Data Logger
 * 
 * Liest alle verfuegbaren Register des Elitech EK-3030E Kuehlstellenreglers
 * ueber RS-485 (Modbus RTU) aus und speichert die Werte als CSV-Datei 
 * auf einer SD-Karte.
 * 
 * Benoetigte Bibliotheken:
 * - ModbusMaster (von Doc Walker)
 * - SD (Standard Arduino Bibliothek)
 * - SPI (Standard Arduino Bibliothek)
 * - SoftwareSerial (Standard Arduino Bibliothek)
 * 
 * Hardware-Verkabelung (Beispiel fuer Arduino UNO/Nano):
 * - SD-Kartenmodul: CS=10, MOSI=11, MISO=12, SCK=13
 * - MAX485-Modul:   RO=2 (RX), DI=3 (TX), DE/RE=4
 */

#include <SPI.h>
#include <SD.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

// --- Konfiguration SD-Karte ---
const int chipSelect = 10; // CS Pin fuer das SD-Kartenmodul
const char* logFileName = "EK3030E.csv";

// --- Konfiguration RS485 & Modbus ---
#define MAX485_RE_NEG  4
#define MAX485_DE      4
#define RX_PIN         2
#define TX_PIN         3

// Modbus-Adresse des EK-3030E (Parameter H17 am Geraet, Standard oft 1)
const uint8_t EK3030E_ADDRESS = 1; 

SoftwareSerial modbusSerial(RX_PIN, TX_PIN);
ModbusMaster node;

// Intervall fuer die Datenabfrage in Millisekunden (z.B. 10000 = 10 Sekunden)
const unsigned long LOG_INTERVAL = 10000;
unsigned long lastLogTime = 0;

// --- Variablen fuer die ausgelesenen Daten ---
// Block 1: 0x0100 - 0x0102 (Messwerte)
float cabinetTemp = 0.0;
float defrostTemp = 0.0;
uint16_t softwareVersion = 0;

// Block 2: 0x0400 - 0x042C (Einstellungen - Auswahl wichtiger Werte)
float onTempCooling = 0.0;
float offTempCooling = 0.0;
float onTempHeating = 0.0;
float offTempHeating = 0.0;
uint16_t defrostTime = 0;
uint16_t defrostCycle = 0;
float defrostStopTemp = 0.0;
float overTempAlarm = 0.0;
float cabinetCalibration = 0.0;

// Block 3: 0x0800 - 0x0802 (Status)
uint16_t relayStatus = 0;
uint16_t digitalInputStatus = 0;
uint16_t alarmStatus = 0;

// Block 4: 0x0A00 (Geraetestatus)
uint16_t deviceStatus = 0;


// --- Hilfsfunktionen fuer ModbusMaster ---
void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

// Hilfsfunktion zur Umwandlung von vorzeichenbehafteten 16-Bit Werten (Temperatur)
// Da Modbus 16-Bit unsigned uebertraegt, muessen negative Werte korrekt konvertiert werden.
float decodeTemperature(uint16_t rawValue) {
  int16_t signedValue = (int16_t)rawValue;
  return signedValue / 10.0;
}

void setup() {
  // Serielle Kommunikation fuer Debugging starten
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println(F("EK-3030E Modbus Data Logger startet..."));

  // MAX485 Pins initialisieren
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus Kommunikation starten (EK-3030E nutzt 9600 Baud, 8N1)
  modbusSerial.begin(9600);
  node.begin(EK3030E_ADDRESS, modbusSerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // SD-Karte initialisieren
  Serial.print(F("Initialisiere SD-Karte... "));
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Fehler bei der SD-Karten Initialisierung!"));
    // Wir halten das Programm hier nicht an, falls man nur den Serial Monitor nutzen will
  } else {
    Serial.println(F("Erfolgreich."));
    
    // Pruefen, ob die Datei existiert, falls nicht -> CSV Header schreiben
    if (!SD.exists(logFileName)) {
      File dataFile = SD.open(logFileName, FILE_WRITE);
      if (dataFile) {
        dataFile.println(F("Timestamp(ms),CabinetTemp_C,DefrostTemp_C,SW_Version,OnTempCooling_C,OffTempCooling_C,DefrostTime_min,DefrostCycle_h,RelayStatus_Raw,AlarmStatus_Raw,DeviceStatus_Raw"));
        dataFile.close();
        Serial.println(F("CSV Header geschrieben."));
      }
    }
  }
  
  Serial.println(F("Setup abgeschlossen. Beginne mit dem Auslesen..."));
}

void loop() {
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = millis();
    readModbusData();
    logDataToSD();
    printDataToSerial();
  }
}

void readModbusData() {
  uint8_t result;
  
  // --------------------------------------------------------
  // 1. Block: Messwerte lesen (0x0100 bis 0x0102 -> 3 Register)
  // --------------------------------------------------------
  result = node.readHoldingRegisters(0x0100, 3);
  if (result == node.ku8MBSuccess) {
    cabinetTemp     = decodeTemperature(node.getResponseBuffer(0));
    defrostTemp     = decodeTemperature(node.getResponseBuffer(1));
    softwareVersion = node.getResponseBuffer(2);
  } else {
    Serial.print(F("Fehler beim Lesen von Block 1: 0x"));
    Serial.println(result, HEX);
  }
  delay(50); // Kurze Pause fuer den RS485 Bus

  // --------------------------------------------------------
  // 2. Block: Wichtige Einstellungen lesen (0x0400 bis 0x0410 -> 17 Register)
  // Hinweis: ModbusMaster kann max. 32 Register auf einmal lesen.
  // --------------------------------------------------------
  result = node.readHoldingRegisters(0x0400, 17);
  if (result == node.ku8MBSuccess) {
    onTempCooling   = decodeTemperature(node.getResponseBuffer(0));
    offTempCooling  = decodeTemperature(node.getResponseBuffer(1));
    onTempHeating   = decodeTemperature(node.getResponseBuffer(2));
    offTempHeating  = decodeTemperature(node.getResponseBuffer(3));
    defrostTime     = node.getResponseBuffer(4);
    defrostCycle    = node.getResponseBuffer(5);
    defrostStopTemp = decodeTemperature(node.getResponseBuffer(9));
    overTempAlarm   = decodeTemperature(node.getResponseBuffer(14));
    cabinetCalibration = decodeTemperature(node.getResponseBuffer(16));
  } else {
    Serial.print(F("Fehler beim Lesen von Block 2: 0x"));
    Serial.println(result, HEX);
  }
  delay(50);

  // --------------------------------------------------------
  // 3. Block: Status Register lesen (0x0800 bis 0x0802 -> 3 Register)
  // --------------------------------------------------------
  result = node.readHoldingRegisters(0x0800, 3);
  if (result == node.ku8MBSuccess) {
    relayStatus        = node.getResponseBuffer(0);
    digitalInputStatus = node.getResponseBuffer(1);
    alarmStatus        = node.getResponseBuffer(2);
  } else {
    Serial.print(F("Fehler beim Lesen von Block 3: 0x"));
    Serial.println(result, HEX);
  }
  delay(50);

  // --------------------------------------------------------
  // 4. Block: Geraetestatus lesen (0x0A00 -> 1 Register)
  // --------------------------------------------------------
  result = node.readHoldingRegisters(0x0A00, 1);
  if (result == node.ku8MBSuccess) {
    deviceStatus = node.getResponseBuffer(0);
  } else {
    Serial.print(F("Fehler beim Lesen von Block 4: 0x"));
    Serial.println(result, HEX);
  }
}

void logDataToSD() {
  File dataFile = SD.open(logFileName, FILE_WRITE);
  
  if (dataFile) {
    // CSV Format: Timestamp, CabinetTemp, DefrostTemp, SW_Version, OnTempCooling, OffTempCooling, DefrostTime, DefrostCycle, RelayStatus, AlarmStatus, DeviceStatus
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(cabinetTemp, 1);
    dataFile.print(",");
    dataFile.print(defrostTemp, 1);
    dataFile.print(",");
    dataFile.print(softwareVersion);
    dataFile.print(",");
    dataFile.print(onTempCooling, 1);
    dataFile.print(",");
    dataFile.print(offTempCooling, 1);
    dataFile.print(",");
    dataFile.print(defrostTime);
    dataFile.print(",");
    dataFile.print(defrostCycle);
    dataFile.print(",");
    dataFile.print(relayStatus, BIN); // Als Binaerwert speichern fuer leichtere Auswertung
    dataFile.print(",");
    dataFile.print(alarmStatus, BIN);
    dataFile.print(",");
    dataFile.println(deviceStatus, BIN);
    
    dataFile.close();
  } else {
    Serial.println(F("Fehler beim Oeffnen der Datei auf SD-Karte!"));
  }
}

void printDataToSerial() {
  Serial.println(F("--- Aktuelle EK-3030E Daten ---"));
  Serial.print(F("Kuehlraum-Temp: ")); Serial.print(cabinetTemp, 1); Serial.println(F(" C"));
  Serial.print(F("Abtau-Temp:     ")); Serial.print(defrostTemp, 1); Serial.println(F(" C"));
  
  // Relais Status auswerten (Bit 0 = Kompressor, Bit 1 = Heizung, Bit 2 = Luefter)
  // Achtung: Laut Doku bedeutet 0 = Close (An/Geschlossen), 1 = Open (Aus/Offen)
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
  Serial.println(F("-------------------------------"));
}
