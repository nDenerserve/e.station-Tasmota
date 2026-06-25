/*
  xdrv_100_pvstation.ino - Tasmota-Treiber für die PV-Station Energiemessung

  Zählt S0-Impulse an bis zu 12 digitalen Eingängen über Hardware-Interrupts und
  veröffentlicht die aufsummierten Energiewerte (kWh) zyklisch per MQTT.

  Ablauf:
    - Jeder Eingangsimpuls löst einen Hardware-Interrupt aus (RISING-Flanke).
    - Die ISR prüft per Zeitstempel, ob der Mindestimpulsabstand (20 ms) eingehalten
      wurde (Software-Entprellung), und inkrementiert dann den Impulszähler des Kanals.
    - Ein Hardware-Timer (10 kHz, Alarm alle 10 000 Ticks = 1 s) prüft minütlich, ob ein
      neues MQTT-Telegrammm verschickt werden soll.
    - PVStationProcessing() wird alle 100 ms vom Tasmota-Scheduler aufgerufen und sendet
      bei gesetztem Flag den JSON-Payload via MQTT.

  Konfiguration:
    - Impulse/kWh je Kanal werden über das Webinterface eingestellt und in einer
      JSON-Datei im UFS gespeichert (Schlüssel: XDRV_100_KEY).


Zusammenfassung der LED-Logik:

Zustand	                    Blau (IO38)	Gelb (IO39)	Rot (IO40)
Boot / kein Netzwerk	        —	          —	           ✓
WLAN-IP, kein Internet	      ✓	          —	          ✓
Ethernet-IP, kein Internet	  —	          ✓	          ✓
WLAN + Internet	              ✓	          —	          —
Ethernet + Internet	          ✓	          ✓	          —
Impuls-Flash (100 ms)	        —	          —	           —
Internet-Erkennung erfolgt über RtcTime.valid — sobald NTP erfolgreich synchronisiert hat, ist Internet erreichbar. Das ist nicht-blockierend und ohne separaten Ping-Task.



*/




#ifdef USE_PVSTATION
/*********************************************************************************************\
 * My IoT Device bare minimum
 *
 *
\*********************************************************************************************/

// Treiber-ID für das Tasmota-Dispatch-System
#define XDRV_100 100
// Schlüssel für die persistente JSON-Einstellungsdatei im UFS
#define XDRV_100_KEY                      "pvstationdrv100"

// Anzeigestrings für das Webinterface (Deutsch)
#define D_CONFIGURE_PVSTATION "PV-Station"
#define D_PVSTATION_PARAMETERS "PV-Station Parameters"
#define D_PVSTATION_COUNTER "Zähler"
#define D_PVSTATION_IMPKWH "Imp/kWh"

// Präfix der JSON-Felder für die Imp/kWh-Konfiguration je Kanal (z. B. "s0counter1")
#define D_PVSTATION_JSON_S0 "s0counter"

// MQTT-Sendeintervall in Minuten; 1 = jede Minute
#define PVSTATION_SEND_INTERVALL        1


/*********************************************************************************************\
 * Tasmota Functions
\*********************************************************************************************/

// Wird am Ende von PVStationInit() auf true gesetzt; verhindert Verarbeitung vor Abschluss der Initialisierung
bool initSuccess = false;

// Konfigurierter Imp/kWh-Faktor je Kanal (Standardwert 1000 Imp/kWh)
volatile uint32_t s0counter_idx[12] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};

// Handle des ESP32-Hardware-Timers für den Sekundentakt
static hw_timer_t *pvstation_timer = nullptr;


// Zuletzt ausgewertete RTC-Minute; verhindert mehrfaches Auslösen innerhalb derselben Minute
volatile uint32_t actualMinute = 0;
// volatile bool validRtcTime = false;
// Flag: wird vom Timer-ISR gesetzt, wenn ein neuer MQTT-Snapshot bereitsteht
volatile bool sendPVStationValues = false;

// Debug-Variablen: Spiegelung der ISR-Zustandswechsel für Logging im Hauptloop
volatile uint32_t debugActualMinute = 0;
volatile uint32_t debugActualMinuteAfter = 0;
// volatile bool debugValidRtcTime = false;
volatile bool debugSendPVStationValues = false;
volatile bool debugTimerFired = false;
volatile uint32_t debugtmpTime = 0;
volatile uint32_t tmpTime;

// Flags, die von den ISRs gesetzt und vom Hauptloop nach dem Logging gelöscht werden
volatile bool pvstationInputStatus[12] = {false,false,false,false,false,false,false,false,false,false,false,false};
// Laufende Impulszähler je Kanal; werden von den ISRs inkrementiert und nach jedem MQTT-Snapshot zurückgesetzt
volatile uint32_t pvstationInputCount[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
// Snapshot der Impulszähler zum Sendezeitpunkt (atomare Kopie aus dem Timer-ISR)
volatile uint32_t pvstationInputCountToSend[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
// Berechnete Energiewerte in kWh, abgeleitet aus dem Snapshot; werden per MQTT gesendet
volatile float pvstationInputWhToSend[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// Zeitstempel des letzten gültigen Impulses je Kanal in ms; Basis für die Software-Entprellung
volatile uint32_t pvstationLastISRTime[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// Mindestabstand zwischen zwei aufeinanderfolgenden gültigen Impulsen in ms
#define PVSTATION_DEBOUNCE_MS 25

// GPIO-Pin je Kanal; wird im ISR zur Pegelvalidierung genutzt (Rauschimpulse auf fallender Flanke abfangen)
const uint8_t pvstationInputPin[12] = {4, 5, 2, 1, 6, 7, 15, 16, 14, 21, 47, 48};


/*********************************************************************************************\
 * RGB-LED (Pin 31=IO38 Blau, Pin 32=IO39 Gelb, Pin 33=IO40 Rot)
\*********************************************************************************************/

#define PVSTATION_LED_PIN_BLUE   38   // IO38
#define PVSTATION_LED_PIN_YELLOW 39   // IO39
#define PVSTATION_LED_PIN_RED    40   // IO40

// Dauer des LED-Ausblitzes bei einem registrierten Impuls in ms
#define PVSTATION_LED_FLASH_MS   100

// Netzwerkzustände, die über die LED-Farbe signalisiert werden
enum PVStationLedState : uint8_t {
  PVLED_RED,     // Startphase / kein Netzwerk      → Rot
  PVLED_PURPLE,  // WLAN-IP, kein Internet           → Blau + Rot = Lila
  PVLED_ORANGE,  // Ethernet-IP, kein Internet       → Gelb + Rot = Orange
  PVLED_BLUE,    // WLAN + Internet                  → Blau
  PVLED_GREEN,   // Ethernet + Internet              → Blau + Gelb ≈ Grün
  PVLED_OFF      // Ausblitz bei Impuls              → Alle aus
};

PVStationLedState pvstationLedState = PVLED_RED;  // aktuell dargestellter LED-Zustand
volatile bool pvstationLedFlash = false;           // vom ISR gesetzt wenn Impuls zählt; nicht ISR-sicher aber unkritisch
uint32_t pvstationLedFlashEnd = 0;                 // millis()-Zeitpunkt, bis zu dem die LED ausgeblendet bleibt


// Schaltet die RGB-LED auf den gewünschten Zustand.
// Blau + Gelb ergibt in additiver Lichtmischung eine grünlich-weiße Farbe (bestmögliche Annäherung an Grün).
// uint8_t statt PVStationLedState als Parameter, damit Tasmotás Auto-Prototyp-Generator
// keinen Prototyp mit unbekanntem Enum-Typ vor dem #ifdef-Block erzeugt.
void PVStationLED_set(uint8_t state) {
  bool b = false, y = false, r = false;
  switch (state) {
    case PVLED_RED:    r = true;          break;
    case PVLED_PURPLE: b = true; r = true; break;
    case PVLED_ORANGE: y = true; r = true; break;
    case PVLED_BLUE:   b = true;          break;
    case PVLED_GREEN:  b = true; y = true; break;
    case PVLED_OFF:                       break;
  }
  digitalWrite(PVSTATION_LED_PIN_BLUE,   b ? HIGH : LOW);
  digitalWrite(PVSTATION_LED_PIN_YELLOW, y ? HIGH : LOW);
  digitalWrite(PVSTATION_LED_PIN_RED,    r ? HIGH : LOW);
}

// Bestimmt den aktuellen Netzwerkzustand und aktualisiert die LED-Farbe.
// RtcTime.valid dient als nicht-blockierender Internet-Indikator:
// Eine erfolgreiche NTP-Synchronisation setzt voraus, dass der NTP-Server erreichbar ist.
void PVStationLED_update(void) {
  uint32_t now = millis();

  // Neuen Impuls-Flash starten: LED kurz ausschalten
  if (pvstationLedFlash) {
    pvstationLedFlash = false;
    pvstationLedFlashEnd = now + PVSTATION_LED_FLASH_MS;
    PVStationLED_set(PVLED_OFF);
    return;
  }

  // Innerhalb des Flash-Zeitfensters nichts tun
  if (now < pvstationLedFlashEnd) { return; }

  // Netzwerkzustand bestimmen
  bool hasInternet = RtcTime.valid;
  bool hasWifi     = WifiHasIPv4();
#ifdef USE_ETHERNET
  bool hasEth = EthernetHasIPv4();
#else
  bool hasEth = false;
#endif

  PVStationLedState target;
  if      (hasEth  && hasInternet) target = PVLED_GREEN;
  else if (hasWifi && hasInternet) target = PVLED_BLUE;
  else if (hasEth)                 target = PVLED_ORANGE;
  else if (hasWifi)                target = PVLED_PURPLE;
  else                             target = PVLED_RED;

  // LED nur neu setzen wenn sich Zustand geändert hat oder gerade Flash beendet wurde
  if (target != pvstationLedState || pvstationLedFlashEnd > 0) {
    pvstationLedState  = target;
    pvstationLedFlashEnd = 0;
    PVStationLED_set(target);
  }
}


// Lädt die Imp/kWh-Konfiguration aller 12 Kanäle aus der UFS-JSON-Datei.
// Gibt true zurück, wenn die Datei vorhanden und lesbar war.
bool PVStationLoadData(void) {
  char key[] = XDRV_100_KEY;
  char jsonCounter[24];
  String json = UfsJsonSettingsRead(key);
  if (json.length() == 0) { return false; }

  JsonParser parser((char*)json.c_str());
  JsonParserObject root = parser.getRootObject();
  if (!root) { return false; }

  for (uint32_t i = 0; i < 12; i++) {
    snprintf_P(jsonCounter, sizeof(jsonCounter), PSTR(D_PVSTATION_JSON_S0 "%d"), i+1);
    s0counter_idx[i] = root.getUInt(jsonCounter, s0counter_idx[i]);
  }

  return true;
}



/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/

#ifdef USE_WEBSERVER

// URL-Pfad der Konfigurationsseite im eingebetteten Webserver
#define WEB_HANDLE_PVSTATION "pvs"

const char S_CONFIGURE_PVSTATION[] PROGMEM = D_CONFIGURE_PVSTATION;

// HTML-Button im Konfigurationsmenü, der zur PV-Station-Seite führt
const char HTTP_BTN_MENU_PVSTATION[] PROGMEM =
  "<p></p><form action='" WEB_HANDLE_PVSTATION "' method='get'><button>" D_CONFIGURE_PVSTATION "</button></form>";

// Kopf der Konfigurationstabelle
const char HTTP_FORM_PVSTATION1[] PROGMEM =
  "<fieldset><legend><b>&nbsp;" D_PVSTATION_PARAMETERS "&nbsp;</b></legend>"
  "<form method='get' action='" WEB_HANDLE_PVSTATION "'>"
  "<table>";
// Eine Tabellenzeile pro Kanal: Kanalbezeichnung, Eingabefeld für Imp/kWh-Wert
const char HTTP_FORM_PVSTATION_COUNTER[] PROGMEM =
  "<tr><td style='width:260px'><b>" D_PVSTATION_COUNTER " %d</b></td><td style='width:70px'><input id='c%d' placeholder='1000' value='%d'></td><td> " D_PVSTATION_IMPKWH "</td></tr>";


// HTTP-Handler für GET /pvs: zeigt die Konfigurationsseite an oder speichert Änderungen.
void HandlePVStationConfiguration(void)
{
  if (!HttpCheckPriviledgedAccess()) { return; }

  AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_HTTP D_CONFIGURE_PVSTATION));

  PVStationLoadData();


  if (Webserver->hasArg(F("save"))) {
    PVStationSaveSettings();
    WebRestart(1);
    return;
  }

  char str[TOPSZ];

  WSContentStart_P(PSTR(D_CONFIGURE_PVSTATION));
  WSContentSendStyle();
  WSContentSend_P(HTTP_FORM_PVSTATION1);
  for (uint32_t i = 0; i < 12; i++) {
    WSContentSend_P(HTTP_FORM_PVSTATION_COUNTER,
      i +1, i,  s0counter_idx[i]);
  }
  WSContentSend_P(PSTR("</table>"));

  WSContentSend_P(HTTP_FORM_END);
  WSContentSpaceButton(BUTTON_CONFIGURATION);
  WSContentStop();
}

// Liest die Formularwerte (Imp/kWh je Kanal) aus der HTTP-Anfrage aus und speichert
// sie als JSON-Datei im UFS. Gibt true zurück bei erfolgreichem Schreiben.
bool PVStationSaveSettings(void) {

  bool result = true;
  String tmpString = "";
  char stringBuffer[22];

  String pvStationSaveString = PSTR("{\"" XDRV_100_KEY "\":{");


  for (uint32_t i = 0; i < 12; i++) {
    s0counter_idx[i] = Webserver->arg(String("c") + String(i)).toInt();
    pvStationSaveString += String(PSTR("\"" D_PVSTATION_JSON_S0 )) + String(i+1) + String(PSTR("\":\"")) + String(s0counter_idx[i]) + String(PSTR("\""));
    if (i < 11) {
      pvStationSaveString += String(PSTR(","));
    }
  }

  pvStationSaveString += PSTR("}}");

  Response_P(PSTR(pvStationSaveString.c_str()));
  result &= UfsJsonSettingsWrite(ResponseData());

  return result;
}

#endif  // USE_WEBSERVER




// Forward declaration: IRAM_ATTR nur an der Definition, nicht hier – sonst Sections-Konflikt
void PVStationTimer_intr();

// Gemeinsame ISR-Hilfsfunktion mit Software-Entprellung.
// Wird aus den kanalspezifischen ISRs aufgerufen; muss im IRAM liegen (ARDUINO_ISR_ATTR).
// esp_timer_get_time() liefert Mikrosekunden seit Boot und ist ISR-sicher auf dem ESP32.
// Impulse mit einem Abstand < PVSTATION_DEBOUNCE_MS werden als Prellimpulse verworfen.
static void ARDUINO_ISR_ATTR pvstationHandleISR(uint8_t idx) {
  // Pegelvalidierung: Rauschspitzen auf der fallenden Flanke erzeugen am ESP32 manchmal
  // einen Schein-RISING-Interrupt, obwohl der Pin bereits LOW ist → verwerfen.
  if (digitalRead(pvstationInputPin[idx]) == LOW) { return; }

  uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);  // µs -> ms
  if (now - pvstationLastISRTime[idx] >= PVSTATION_DEBOUNCE_MS) {
    pvstationLastISRTime[idx] = now;
    pvstationInputStatus[idx] = true;
    pvstationInputCount[idx]++;
    pvstationLedFlash = true;  // LED-Ausblitz im nächsten Processing-Zyklus auslösen
  }
}

// Kanalspezifische ISRs; delegieren ausschließlich an pvstationHandleISR()
void ARDUINO_ISR_ATTR input1ISR()  { pvstationHandleISR(0);  }
void ARDUINO_ISR_ATTR input2ISR()  { pvstationHandleISR(1);  }
void ARDUINO_ISR_ATTR input3ISR()  { pvstationHandleISR(2);  }
void ARDUINO_ISR_ATTR input4ISR()  { pvstationHandleISR(3);  }
void ARDUINO_ISR_ATTR input5ISR()  { pvstationHandleISR(4);  }
void ARDUINO_ISR_ATTR input6ISR()  { pvstationHandleISR(5);  }
void ARDUINO_ISR_ATTR input7ISR()  { pvstationHandleISR(6);  }
void ARDUINO_ISR_ATTR input8ISR()  { pvstationHandleISR(7);  }
void ARDUINO_ISR_ATTR input9ISR()  { pvstationHandleISR(8);  }
void ARDUINO_ISR_ATTR input10ISR() { pvstationHandleISR(9);  }
void ARDUINO_ISR_ATTR input11ISR() { pvstationHandleISR(10); }
void ARDUINO_ISR_ATTR input12ISR() { pvstationHandleISR(11); }

// Timer-ISR: feuert jede Sekunde (10 kHz Timer, Alarm nach 10 000 Ticks).
// Prüft, ob eine neue Sendeminute erreicht wurde, und erstellt dann einen
// atomaren Snapshot der Impulszähler für den Hauptloop. Muss im IRAM liegen.
void IRAM_ATTR PVStationTimer_intr() {

  debugTimerFired = true;
  debugActualMinute = actualMinute;
  // debugValidRtcTime = validRtcTime;


  if (!RtcTime.valid) {
    // Keine gültige RTC-Zeit vorhanden – noch kein Senden möglich
    return;
  } else if (RtcTime.valid ) {
    // validRtcTime = true;
    actualMinute = RtcTime.minute;
    // Prüfen, ob die aktuelle Minute ein Vielfaches des Sendeintervalls ist
    if (actualMinute % PVSTATION_SEND_INTERVALL == 0) {
    // uint32_t tmpTime; // Global
      // tmpTime verhindert, dass innerhalb derselben Minute mehrfach ein Snapshot erstellt wird
      if (actualMinute != tmpTime) {
        // Atomare Kopie der laufenden Zähler; Hauptloop liest pvstationInputCountToSend
        memcpy((void*)pvstationInputCountToSend, (const void*)pvstationInputCount, sizeof(pvstationInputCount));
        // Laufende Zähler zurücksetzen, damit das nächste Intervall sauber beginnt
        memset((void*)pvstationInputCount, 0, sizeof(pvstationInputCount));
        sendPVStationValues = true;
        debugSendPVStationValues = sendPVStationValues;
        actualMinute = RtcTime.minute;
        debugActualMinuteAfter = actualMinute;
        tmpTime = actualMinute;
      }
    }
    // if (actualMinute + PVSTATION_SEND_INTERVALL >= 60) {
    //   tmpTime = (actualMinute + PVSTATION_SEND_INTERVALL) - 60;
    // } else {
    //   tmpTime = actualMinute + PVSTATION_SEND_INTERVALL;
    // }
    debugtmpTime = tmpTime;
    // if (RtcTime.minute >= tmpTime) {
    //   memcpy((void*)pvstationInputCountToSend, (const void*)pvstationInputCount, sizeof(pvstationInputCount));
    //   sendPVStationValues = true;
    //   debugSendPVStationValues = sendPVStationValues;
    //   memset((void*)pvstationInputCount, 0, sizeof(pvstationInputCount));
    //   actualMinute = RtcTime.minute;
    //   debugActualMinuteAfter = actualMinute;
    // }
  }

}


// Initialisierung: wird einmalig beim Tasmota-FUNC_INIT aufgerufen.
// Konfiguriert GPIO-Pins, hängt ISRs ein und startet den Hardware-Timer.
void PVStationInit()
{

  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("PV Station init..."));

  // Serial.begin(115200);

  if (PVStationLoadData()) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: PVStation loaded from file"));
  }

  // GPIO-Pins als Eingänge konfigurieren und Interrupts auf steigende Flanke registrieren.
  // Kanalzuordnung: Kanal 1–12 → GPIO 4, 5, 2, 1, 6, 7, 15, 16, 14, 21, 47, 48
  pinMode(4, INPUT);
  attachInterrupt(4, input1ISR, RISING);
  pinMode(5, INPUT);
  attachInterrupt(5, input2ISR, RISING);
  pinMode(2, INPUT);
  attachInterrupt(2, input3ISR, RISING);
  pinMode(1, INPUT);
  attachInterrupt(1, input4ISR, RISING);
  pinMode(6, INPUT);
  attachInterrupt(6, input5ISR, RISING);
  pinMode(7, INPUT);
  attachInterrupt(7, input6ISR, RISING);
  pinMode(15, INPUT);
  attachInterrupt(15, input7ISR, RISING);
  pinMode(16, INPUT);
  attachInterrupt(16, input8ISR, RISING);
  pinMode(14, INPUT);
  attachInterrupt(14, input9ISR, RISING);
  pinMode(21, INPUT);
  attachInterrupt(21, input10ISR, RISING);
  pinMode(47, INPUT);
  attachInterrupt(47, input11ISR, RISING);
  pinMode(48, INPUT);
  attachInterrupt(48, input12ISR, RISING);

  // Hardware-Timer: 10 kHz Taktfrequenz, Alarm nach 10 000 Ticks = alle 1 Sekunde, auto-reload
  pvstation_timer = timerBegin(10000);
  timerAttachInterrupt(pvstation_timer, &PVStationTimer_intr);
  timerAlarm(pvstation_timer, 10000, true, 0);

  // RGB-LED initialisieren und sofort auf Rot setzen (Startzustand: kein Netzwerk)
  pinMode(PVSTATION_LED_PIN_BLUE,   OUTPUT);
  pinMode(PVSTATION_LED_PIN_YELLOW, OUTPUT);
  pinMode(PVSTATION_LED_PIN_RED,    OUTPUT);
  PVStationLED_set(PVLED_RED);

  // Initialisierung erfolgreich – Verarbeitung im Hauptloop freigeben
  initSuccess = true;

}



// Hauptverarbeitungsroutine; wird alle 50 ms von FUNC_EVERY_50_MSECOND aufgerufen.
// Gibt ausgelöste Eingänge im Debug-Log aus und sendet bei gesetztem Flag den MQTT-Payload.
void PVStationProcessing(void)
{
  // LED-Zustand aktualisieren (Netzwerkstatus, Impuls-Flash)
  PVStationLED_update();

  // Pulse-Debug-Logging: jeder Kanal meldet sich einmalig nach dem ersten Impuls
    if (debugTimerFired) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 1: %d impulses"), pvstationInputCount[0]);
        pvstationInputStatus[0] = false;
    }
    if (pvstationInputStatus[1]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 2: %d impulses"), pvstationInputCount[1]);
        pvstationInputStatus[1] = false;
    }
    if (pvstationInputStatus[2]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 3: %d impulses"), pvstationInputCount[2]);
        pvstationInputStatus[2] = false;
    }
    if (pvstationInputStatus[3]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 4: %d impulses"), pvstationInputCount[3]);
        pvstationInputStatus[3] = false;
    }
    if (pvstationInputStatus[4]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 5: %d impulses"), pvstationInputCount[4]);
        pvstationInputStatus[4] = false;
    }
    if (pvstationInputStatus[5]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 6: %d impulses"), pvstationInputCount[5]);
        pvstationInputStatus[5] = false;
    }
    if (pvstationInputStatus[6]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 7: %d impulses"), pvstationInputCount[6]);
        pvstationInputStatus[6] = false;
    }
    if (pvstationInputStatus[7]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 8: %d impulses"), pvstationInputCount[7]);
        pvstationInputStatus[7] = false;
    }
    if (pvstationInputStatus[8]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 9: %d impulses"), pvstationInputCount[8]);
        pvstationInputStatus[8] = false;
    }
    if (pvstationInputStatus[9]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 10: %d impulses"), pvstationInputCount[9]);
        pvstationInputStatus[9] = false;
    }
    if (pvstationInputStatus[10]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 11: %d impulses"), pvstationInputCount[10]);
        pvstationInputStatus[10] = false;
    }
    if (pvstationInputStatus[11]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 12: %d impulses"), pvstationInputCount[11]);
        pvstationInputStatus[11] = false;
    }

    // Timer-ISR-Diagnose: Debug-Ausgabe der Zustandswerte zum Zeitpunkt des Timer-Feuerns
    if (debugTimerFired) {
      debugTimerFired = false;
      AddLog(LOG_LEVEL_DEBUG, PSTR("PV-Station Timer fired..."));
      // AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugValidRtcTime: %d"), debugValidRtcTime);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> RtcMinute: %d"), RtcTime.minute);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugActualMinute before: %d"), debugActualMinute);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugtmpTime: %d"), debugtmpTime);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugActualMinute after: %d"), debugActualMinuteAfter);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugSendPVStationValues: %d"), debugSendPVStationValues);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> sendPVStationValues: %d"), sendPVStationValues);
      AddLog(LOG_LEVEL_DEBUG, NetworkMacAddress().c_str());
      AddLog(LOG_LEVEL_DEBUG, NetworkAddress().toString().c_str());
      AddLog(LOG_LEVEL_DEBUG, NetworkUniqueId().c_str());
    }


    // MQTT-Versand: vom Timer-ISR gesetztes Flag auswerten, kWh berechnen und JSON senden
    if (sendPVStationValues) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("e.station: sending values..."));
      sendPVStationValues = false;
      // Impulszähler in kWh umrechnen: kWh = Impulse / (Imp/kWh * 1000 W/kW)
      for (uint32_t i = 0; i < 12; i++) {
        if ((pvstationInputCountToSend[i] > 0) && (s0counter_idx[i] > 0)) {
          AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter %d: %d impulses"), i+1, pvstationInputCountToSend[i]);
          // Serial.println(pvstationInputCountToSend[i]);
          pvstationInputWhToSend[i] = (float)pvstationInputCountToSend[i] / (float)s0counter_idx[i] *1000.0; // assuming Imp/kWh
          AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter %d: %d Wh"), i+1, pvstationInputWhToSend[i]);
          // Serial.println(pvstationInputWhToSend[i]);
          pvstationInputCountToSend[i] = 0;
        } else {
          pvstationInputWhToSend[i] = 0.0;
          pvstationInputCountToSend[i] = 0;
        }

        // MqttPublishPrefixTopicRulesProcess_P(TELE, subtopic, "Test");
        // Serial.println(PSTR(D_CMND_TOPIC));
      }
      // snprintf_P(bdt, sizeof(bdt), PSTR("%d" D_YEAR_MONTH_SEPARATOR "%02d" D_MONTH_DAY_SEPARATOR "%02d" D_DATE_TIME_SEPARATOR "%s"), year, month, day, PSTR(__TIME__));

      // Response_P(PSTR("{\"Time\":\"\":%d}}"), "25");

      // JSON-Payload zusammenbauen: Zeitstempel, Netzwerkinfos, Energiewerte aller 12 Kanäle
      String pvstationMQTTstring = "";

      pvstationMQTTstring = String(PSTR("{\"Time\":\""));
      pvstationMQTTstring += String(GetPVStationDateAndTime());
      pvstationMQTTstring += String(PSTR("\",\"IpAdress\":\""));
      pvstationMQTTstring += NetworkAddress().toString();
      pvstationMQTTstring += String(PSTR("\",\"MacAdress\":\""));
      pvstationMQTTstring += NetworkMacAddress();
      pvstationMQTTstring += String(PSTR("\",\"UniqueNetworkId\":\""));
      pvstationMQTTstring += NetworkUniqueId();
      pvstationMQTTstring += String(PSTR("\",\"CounterValues\":{"));
      for (uint32_t i = 0; i < 12; i++) {
        pvstationMQTTstring += String(PSTR("\"Counter"));
        pvstationMQTTstring += String(i+1);
        pvstationMQTTstring += String(PSTR("\":"));
        pvstationMQTTstring += String(pvstationInputWhToSend[i], 3);
        if (i < 11) {
          pvstationMQTTstring += String(PSTR(","));
        }
      }
      pvstationMQTTstring += String(PSTR("}}"));;
      AddLog(LOG_LEVEL_INFO,PSTR(pvstationMQTTstring.c_str()));

      Response_P(PSTR(pvstationMQTTstring.c_str()));

      MqttPublishTeleSensor();
      memset((void*)pvstationInputCountToSend, 0, sizeof(pvstationInputCountToSend));
      // memset((void*)pvstationInputWhToSend, 0.0, sizeof(pvstationInputWhToSend));
    }
}


// Gibt den aktuellen RTC-Zeitstempel als ISO-8601-String zurück (z. B. "2024-06-18T14:30:00").
String GetPVStationDateAndTime(void) {
  // "2017-03-07T11:08:02" - ISO8601:2004
  char pvstationdate[21];

  snprintf_P(pvstationdate, sizeof(pvstationdate), PSTR("%d" D_YEAR_MONTH_SEPARATOR "%02d" D_MONTH_DAY_SEPARATOR "%02d" D_DATE_TIME_SEPARATOR "%02d" D_HOUR_MINUTE_SEPARATOR "%02d" D_MINUTE_SECOND_SEPARATOR "%02d"), RtcTime.year, RtcTime.month, RtcTime.day_of_month, RtcTime.hour, RtcTime.minute, RtcTime.second);
  return String(pvstationdate);  // 2017-03-07T11:08:02
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

// Tasmota-Dispatch-Funktion: leitet Systemereignisse (Init, Zyklus, Webserver) an die
// entsprechenden PV-Station-Funktionen weiter.
bool Xdrv100(uint32_t function)
{


  bool result = false;


  if (FUNC_INIT == function) {
    PVStationInit();
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("PV-Station init is done..."));
  }
  else if (initSuccess) {

    // Serial.print("Function: ");
    // Serial.println(function);

    //  if (function != 8) {
    //           Serial.println("PV-Station Web Default...");
    //           Serial.println(function);
    //         }
    switch (function) {

      case FUNC_EVERY_50_MSECOND:
        PVStationProcessing();
        break;
#ifdef USE_WEBSERVER
// #ifndef FIRMWARE_MINIMAL    // not needed in minimal/safeboot because of disabled feature and Settings are not saved anyways
          case FUNC_WEB_ADD_BUTTON:
              WSContentSend_P(HTTP_BTN_MENU_PVSTATION);
              break;
          case FUNC_WEB_ADD_HANDLER:
              WebServer_on(PSTR("/" WEB_HANDLE_PVSTATION), HandlePVStationConfiguration);
              break;
      // #ifdef USE_WEB_STATUS_LINE
      //     case FUNC_WEB_STATUS_RIGHT:
      //         // if (MqttIsConnected()) {
      //         // if (MqttTLSEnabled()) {
      //         //     WSContentStatusSticker(PSTR(D_PVSTATION_TLS_ENABLE));
      //         // } else {
      //         //     WSContentStatusSticker(PSTR(D_PVSTATION));
      //         // }
      //         // }
      //         break;
// #endif  // USE_WEB_STATUS_LINE
// #endif  // not FIRMWARE_MINIMAL
#endif  // USE_WEBSERVER



    }

  }

  return result;
}

#endif  // USE_PVSTATION
