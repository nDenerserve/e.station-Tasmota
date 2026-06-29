/*
  xdrv_100_estation.ino - Tasmota-Treiber für die e.station Energiemessung

  Zählt S0-Impulse an bis zu 12 digitalen Eingängen über Hardware-Interrupts und
  veröffentlicht die aufsummierten Energiewerte (kWh) zyklisch per MQTT.

  Ablauf:
    - Jeder Eingangsimpuls löst einen Hardware-Interrupt aus (RISING-Flanke).
    - Die ISR prüft per Zeitstempel, ob der Mindestimpulsabstand (20 ms) eingehalten
      wurde (Software-Entprellung), und inkrementiert dann den Impulszähler des Kanals.
    - Ein Hardware-Timer (10 kHz, Alarm alle 10 000 Ticks = 1 s) prüft minütlich, ob ein
      neues MQTT-Telegrammm verschickt werden soll.
    - EStationProcessing() wird alle 100 ms vom Tasmota-Scheduler aufgerufen und sendet
      bei gesetztem Flag den JSON-Payload via MQTT.

  Konfiguration:
    - Impulse/kWh je Kanal werden über das Webinterface eingestellt und in einer
      JSON-Datei im UFS gespeichert (Schlüssel: XDRV_100_KEY).


  Zustand	LED-Farbe
  Boot / kein Netzwerk	Rot
  Ethernet-IP, kein Internet	Orange
  Ethernet + Internet	Grün
  Impuls an S0-Eingang	kurz aus (100 ms)



*/





#ifdef USE_ESTATION
/*********************************************************************************************\
 * My IoT Device bare minimum
 *
 *
\*********************************************************************************************/

// Treiber-ID für das Tasmota-Dispatch-System
#define XDRV_100 100
// Schlüssel für die persistente JSON-Einstellungsdatei im UFS
#define XDRV_100_KEY                      "estationdrv100"

// Anzeigestring (Produktname, nicht übersetzt)
#define D_CONFIGURE_ESTATION "e.station"
// D_ESTATION_PARAMETERS, D_ESTATION_IMPULSES, D_ESTATION_LAST_SENT, D_ESTATION_NOT_YET_SENT
// werden aus der jeweiligen Sprachdatei (language/xx_XX.h) bezogen.
// D_COUNTER wird ebenfalls aus der Sprachdatei bezogen ("Counter" / "Zähler").
#define D_ESTATION_IMPKWH "Imp/kWh"

// Präfix der JSON-Felder für die Imp/kWh-Konfiguration je Kanal (z. B. "s0counter1")
#define D_ESTATION_JSON_S0 "s0counter"

// MQTT-Sendeintervall in Minuten; 1 = jede Minute
#define ESTATION_SEND_INTERVALL        1


/*********************************************************************************************\
 * Tasmota Functions
\*********************************************************************************************/

// Wird am Ende von EStationInit() auf true gesetzt; verhindert Verarbeitung vor Abschluss der Initialisierung
bool initSuccess = false;

// Konfigurierter Imp/kWh-Faktor je Kanal (Standardwert 1000 Imp/kWh)
volatile uint32_t s0counter_idx[12] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};

// Handle des ESP32-Hardware-Timers für den Sekundentakt
static hw_timer_t *estation_timer = nullptr;


// Zuletzt ausgewertete RTC-Minute; verhindert mehrfaches Auslösen innerhalb derselben Minute
volatile uint32_t actualMinute = 0;
// volatile bool validRtcTime = false;
// Flag: wird vom Timer-ISR gesetzt, wenn ein neuer MQTT-Snapshot bereitsteht
volatile bool sendEStationValues = false;

// Debug-Variablen: Spiegelung der ISR-Zustandswechsel für Logging im Hauptloop
volatile uint32_t debugActualMinute = 0;
volatile uint32_t debugActualMinuteAfter = 0;
// volatile bool debugValidRtcTime = false;
volatile bool debugSendEStationValues = false;
volatile bool debugTimerFired = false;
volatile uint32_t debugtmpTime = 0;
volatile uint32_t tmpTime;

// Flags, die von den ISRs gesetzt und vom Hauptloop nach dem Logging gelöscht werden
volatile bool estationInputStatus[12] = {false,false,false,false,false,false,false,false,false,false,false,false};
// Laufende Impulszähler je Kanal; werden von den ISRs inkrementiert und nach jedem MQTT-Snapshot zurückgesetzt
volatile uint32_t estationInputCount[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
// Snapshot der Impulszähler zum Sendezeitpunkt (atomare Kopie aus dem Timer-ISR)
volatile uint32_t estationInputCountToSend[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
// Berechnete Energiewerte in kWh, abgeleitet aus dem Snapshot; werden per MQTT gesendet
volatile float estationInputWhToSend[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// Zeitstempel des letzten gültigen Impulses je Kanal in ms; Basis für die Software-Entprellung
volatile uint32_t estationLastISRTime[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

// ISO-8601-Zeitstempel des letzten MQTT-Versands; leer bis zum ersten Senden
char estationLastSendTime[21] = "";

// Mindestabstand zwischen zwei aufeinanderfolgenden gültigen Impulsen in ms
#define ESTATION_DEBOUNCE_MS 25

// GPIO-Pin je Kanal; wird im ISR zur Pegelvalidierung genutzt (Rauschimpulse auf fallender Flanke abfangen)
const uint8_t estationInputPin[12] = {4, 5, 2, 1, 6, 7, 15, 16, 14, 21, 47, 48};


/*********************************************************************************************\
 * RGB-LED (Pin 31=IO38 Blau, Pin 32=IO39 Gelb, Pin 33=IO40 Rot)
\*********************************************************************************************/

#define ESTATION_LED_PIN_BLUE   38   // IO38
#define ESTATION_LED_PIN_YELLOW 39   // IO39
#define ESTATION_LED_PIN_RED    40   // IO40

// Dauer des LED-Ausblitzes bei einem registrierten Impuls in ms
#define ESTATION_LED_FLASH_MS   100

// Netzwerkzustände, die über die LED-Farbe signalisiert werden
enum EStationLedState : uint8_t {
  ELED_RED,     // Startphase / kein Netzwerk      → Rot
  ELED_PURPLE,  // WLAN-IP, kein Internet           → Blau + Rot = Lila
  ELED_ORANGE,  // Ethernet-IP, kein Internet       → Gelb + Rot = Orange
  ELED_BLUE,    // WLAN + Internet                  → Blau
  ELED_GREEN,   // Ethernet + Internet              → Gelb (Blau+Gelb = Weiß, daher nur Gelb)
  ELED_OFF      // Ausblitz bei Impuls              → Alle aus
};

EStationLedState estationLedState = ELED_RED;  // aktuell dargestellter LED-Zustand
volatile bool estationLedFlash = false;           // vom ISR gesetzt wenn Impuls zählt; nicht ISR-sicher aber unkritisch
uint32_t estationLedFlashEnd = 0;                 // millis()-Zeitpunkt, bis zu dem die LED ausgeblendet bleibt


// Schaltet die RGB-LED auf den gewünschten Zustand.
// Blau + Gelb ergibt in additiver Lichtmischung eine grünlich-weiße Farbe (bestmögliche Annäherung an Grün).
// uint8_t statt EStationLedState als Parameter, damit Tasmotás Auto-Prototyp-Generator
// keinen Prototyp mit unbekanntem Enum-Typ vor dem #ifdef-Block erzeugt.
void EStationLED_set(uint8_t state) {
  bool b = false, y = false, r = false;
  switch (state) {
    case ELED_RED:    r = true;          break;
    case ELED_PURPLE: b = true; r = true; break;
    case ELED_ORANGE: y = true; r = true; break;
    case ELED_BLUE:   b = true;          break;
    case ELED_GREEN:  y = true;           break;  // nur Gelb: Blau+Gelb wäre Weiß
    case ELED_OFF:                       break;
  }
  // LED ist common-anode: LOW = AN, HIGH = AUS → Logik invertiert
  digitalWrite(ESTATION_LED_PIN_BLUE,   b ? LOW : HIGH);
  digitalWrite(ESTATION_LED_PIN_YELLOW, y ? LOW : HIGH);
  digitalWrite(ESTATION_LED_PIN_RED,    r ? LOW : HIGH);
}

// Bestimmt den aktuellen Netzwerkzustand und aktualisiert die LED-Farbe.
// RtcTime.valid dient als nicht-blockierender Internet-Indikator:
// Eine erfolgreiche NTP-Synchronisation setzt voraus, dass der NTP-Server erreichbar ist.
void EStationLED_update(void) {
  uint32_t now = millis();

  // Neuen Impuls-Flash starten: LED kurz ausschalten
  if (estationLedFlash) {
    estationLedFlash = false;
    estationLedFlashEnd = now + ESTATION_LED_FLASH_MS;
    EStationLED_set(ELED_OFF);
    return;
  }

  // Innerhalb des Flash-Zeitfensters nichts tun
  if (now < estationLedFlashEnd) { return; }

  // Netzwerkzustand bestimmen (USE_ETHERNET ist in user_config_override.h aktiviert)
  bool hasInternet = RtcTime.valid;
  bool hasWifi     = WifiHasIPv4();
  bool hasEth      = EthernetHasIPv4();

  // Diagnoselog bei jedem Zustandswechsel
  static bool lastWifi = false, lastEth = false, lastInternet = false;
  if (hasWifi != lastWifi || hasEth != lastEth || hasInternet != lastInternet) {
    lastWifi = hasWifi; lastEth = hasEth; lastInternet = hasInternet;
    AddLog(LOG_LEVEL_INFO, PSTR("e.LED: wifi=%d eth=%d internet=%d"), hasWifi, hasEth, hasInternet);
  }

  EStationLedState target;
  if      (hasEth  && hasInternet) target = ELED_GREEN;
  else if (hasWifi && hasInternet) target = ELED_BLUE;
  else if (hasEth)                 target = ELED_ORANGE;
  else if (hasWifi)                target = ELED_PURPLE;
  else                             target = ELED_RED;

  // LED nur neu setzen wenn sich Zustand geändert hat oder gerade Flash beendet wurde
  if (target != estationLedState || estationLedFlashEnd > 0) {
    estationLedState  = target;
    estationLedFlashEnd = 0;
    EStationLED_set(target);
  }
}


// Lädt die Imp/kWh-Konfiguration aller 12 Kanäle aus der UFS-JSON-Datei.
// Gibt true zurück, wenn die Datei vorhanden und lesbar war.
bool EStationLoadData(void) {
  char key[] = XDRV_100_KEY;
  char jsonCounter[24];
  String json = UfsJsonSettingsRead(key);
  if (json.length() == 0) { return false; }

  JsonParser parser((char*)json.c_str());
  JsonParserObject root = parser.getRootObject();
  if (!root) { return false; }

  for (uint32_t i = 0; i < 12; i++) {
    snprintf_P(jsonCounter, sizeof(jsonCounter), PSTR(D_ESTATION_JSON_S0 "%d"), i+1);
    s0counter_idx[i] = root.getUInt(jsonCounter, s0counter_idx[i]);
  }

  return true;
}



/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/

#ifdef USE_WEBSERVER

// URL-Pfad der Konfigurationsseite im eingebetteten Webserver
#define WEB_HANDLE_ESTATION "es"

const char S_CONFIGURE_ESTATION[] PROGMEM = D_CONFIGURE_ESTATION;

// HTML-Button im Konfigurationsmenü, der zur e.station-Seite führt
const char HTTP_BTN_MENU_ESTATION[] PROGMEM =
  "<p></p><form action='" WEB_HANDLE_ESTATION "' method='get'><button>" D_CONFIGURE_ESTATION "</button></form>";

// Kopf der Konfigurationstabelle
const char HTTP_FORM_ESTATION1[] PROGMEM =
  "<fieldset><legend><b>&nbsp;" D_ESTATION_PARAMETERS "&nbsp;</b></legend>"
  "<form method='get' action='" WEB_HANDLE_ESTATION "'>"
  "<table>";
// Eine Tabellenzeile pro Kanal: Kanalbezeichnung, Eingabefeld für Imp/kWh-Wert
const char HTTP_FORM_ESTATION_COUNTER[] PROGMEM =
  "<tr><td style='width:260px'><b>" D_COUNTER " %d</b></td><td style='width:70px'><input id='c%d' placeholder='1000' value='%d'></td><td> " D_ESTATION_IMPKWH "</td></tr>";

// URL-Pfad der Statusseite im eingebetteten Webserver
#define WEB_HANDLE_ESTATION_INFO "esi"

// HTML-Button im Konfigurationsmenü, der zur e.station-Statusseite führt
const char HTTP_BTN_MENU_ESTATION_INFO[] PROGMEM =
  "<p></p><form action='" WEB_HANDLE_ESTATION_INFO "' method='get'><button>e.station Status</button></form>";

// Kopf der Statustabelle: Zeitstempel + Spaltenüberschriften
const char HTTP_ESTATION_INFO_HEADER[] PROGMEM =
  "<fieldset><legend><b>&nbsp;" D_CONFIGURE_ESTATION " " D_STATUS "&nbsp;</b></legend>"
  "<p>" D_ESTATION_LAST_SENT ": <b>%s</b></p>"
  "<table>"
  "<tr><th style='width:200px;text-align:left'>" D_COUNTER "</th>"
  "<th style='width:90px;text-align:right'>" D_ESTATION_IMPULSES "</th>"
  "<th style='width:120px;text-align:right'>kWh</th>"
  "<th style='width:100px;text-align:right'>" D_ESTATION_IMPKWH "</th></tr>";

// Eine Tabellenzeile pro Kanal in der Statusansicht
const char HTTP_ESTATION_INFO_ROW[] PROGMEM =
  "<tr><td><b>" D_COUNTER " %d</b></td>"
  "<td style='text-align:right'>%d</td>"
  "<td style='text-align:right'>%s</td>"
  "<td style='text-align:right'>%d</td></tr>";


// HTTP-Handler für GET /pvs: zeigt die Konfigurationsseite an oder speichert Änderungen.
void HandleEStationConfiguration(void)
{
  if (!HttpCheckPriviledgedAccess()) { return; }

  AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_HTTP D_CONFIGURE_ESTATION));

  EStationLoadData();


  if (Webserver->hasArg(F("save"))) {
    EStationSaveSettings();
    WebRestart(1);
    return;
  }

  char str[TOPSZ];

  WSContentStart_P(PSTR(D_CONFIGURE_ESTATION));
  WSContentSendStyle();
  WSContentSend_P(HTTP_FORM_ESTATION1);
  for (uint32_t i = 0; i < 12; i++) {
    WSContentSend_P(HTTP_FORM_ESTATION_COUNTER,
      i +1, i,  s0counter_idx[i]);
  }
  WSContentSend_P(PSTR("</table>"));

  WSContentSend_P(HTTP_FORM_END);
  WSContentSpaceButton(BUTTON_CONFIGURATION);
  WSContentStop();
}

// HTTP-Handler für GET /esi: zeigt die Statusseite mit aktuellen Zählerständen an.
// Lädt sich alle 5 Sekunden automatisch neu (JS-setTimeout).
void HandleEStationInfo(void)
{
  if (!HttpCheckPriviledgedAccess()) { return; }

  AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_HTTP "e.station Status"));

  WSContentStart_P(PSTR("e.station Status"));
  WSContentSendStyle();
  // HTTP_HEADER1 lässt <script> offen – Refresh erst nach </script> (= nach WSContentSendStyle) einfügen
  WSContentSend_P(PSTR("<script>setTimeout(()=>location.reload(),5000);</script>"));

  const char *lastSend = (estationLastSendTime[0] != '\0') ? estationLastSendTime : PSTR(D_ESTATION_NOT_YET_SENT);
  WSContentSend_P(HTTP_ESTATION_INFO_HEADER, lastSend);

  for (uint32_t i = 0; i < 12; i++) {
    uint32_t count = estationInputCount[i];
    float kwh = (s0counter_idx[i] > 0) ? (float)count / (float)s0counter_idx[i] : 0.0f;
    char kwhStr[12];
    dtostrf(kwh, 1, 3, kwhStr);
    WSContentSend_P(HTTP_ESTATION_INFO_ROW, i + 1, count, kwhStr, s0counter_idx[i]);
  }

  WSContentSend_P(PSTR("</table></fieldset>"));
  WSContentSpaceButton(BUTTON_CONFIGURATION);
  WSContentStop();
}

// Liest die Formularwerte (Imp/kWh je Kanal) aus der HTTP-Anfrage aus und speichert
// sie als JSON-Datei im UFS. Gibt true zurück bei erfolgreichem Schreiben.
bool EStationSaveSettings(void) {

  bool result = true;
  String tmpString = "";
  char stringBuffer[22];

  String eStationSaveString = PSTR("{\"" XDRV_100_KEY "\":{");


  for (uint32_t i = 0; i < 12; i++) {
    s0counter_idx[i] = Webserver->arg(String("c") + String(i)).toInt();
    eStationSaveString += String(PSTR("\"" D_ESTATION_JSON_S0 )) + String(i+1) + String(PSTR("\":\"")) + String(s0counter_idx[i]) + String(PSTR("\""));
    if (i < 11) {
      eStationSaveString += String(PSTR(","));
    }
  }

  eStationSaveString += PSTR("}}");

  Response_P(PSTR(eStationSaveString.c_str()));
  result &= UfsJsonSettingsWrite(ResponseData());

  return result;
}

#endif  // USE_WEBSERVER




// Forward declaration: IRAM_ATTR nur an der Definition, nicht hier – sonst Sections-Konflikt
void EStationTimer_intr();

// Gemeinsame ISR-Hilfsfunktion mit Software-Entprellung.
// Wird aus den kanalspezifischen ISRs aufgerufen; muss im IRAM liegen (ARDUINO_ISR_ATTR).
// esp_timer_get_time() liefert Mikrosekunden seit Boot und ist ISR-sicher auf dem ESP32.
// Impulse mit einem Abstand < ESTATION_DEBOUNCE_MS werden als Prellimpulse verworfen.
static void ARDUINO_ISR_ATTR estationHandleISR(uint8_t idx) {
  // Pegelvalidierung: Rauschspitzen auf der fallenden Flanke erzeugen am ESP32 manchmal
  // einen Schein-RISING-Interrupt, obwohl der Pin bereits LOW ist → verwerfen.
  if (digitalRead(estationInputPin[idx]) == LOW) { return; }

  uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);  // µs -> ms
  if (now - estationLastISRTime[idx] >= ESTATION_DEBOUNCE_MS) {
    estationLastISRTime[idx] = now;
    estationInputStatus[idx] = true;
    estationInputCount[idx]++;
    estationLedFlash = true;  // LED-Ausblitz im nächsten Processing-Zyklus auslösen
  }
}

// Kanalspezifische ISRs; delegieren ausschließlich an estationHandleISR()
void ARDUINO_ISR_ATTR input1ISR()  { estationHandleISR(0);  }
void ARDUINO_ISR_ATTR input2ISR()  { estationHandleISR(1);  }
void ARDUINO_ISR_ATTR input3ISR()  { estationHandleISR(2);  }
void ARDUINO_ISR_ATTR input4ISR()  { estationHandleISR(3);  }
void ARDUINO_ISR_ATTR input5ISR()  { estationHandleISR(4);  }
void ARDUINO_ISR_ATTR input6ISR()  { estationHandleISR(5);  }
void ARDUINO_ISR_ATTR input7ISR()  { estationHandleISR(6);  }
void ARDUINO_ISR_ATTR input8ISR()  { estationHandleISR(7);  }
void ARDUINO_ISR_ATTR input9ISR()  { estationHandleISR(8);  }
void ARDUINO_ISR_ATTR input10ISR() { estationHandleISR(9);  }
void ARDUINO_ISR_ATTR input11ISR() { estationHandleISR(10); }
void ARDUINO_ISR_ATTR input12ISR() { estationHandleISR(11); }

// Timer-ISR: feuert jede Sekunde (10 kHz Timer, Alarm nach 10 000 Ticks).
// Prüft, ob eine neue Sendeminute erreicht wurde, und erstellt dann einen
// atomaren Snapshot der Impulszähler für den Hauptloop. Muss im IRAM liegen.
void IRAM_ATTR EStationTimer_intr() {

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
    if (actualMinute % ESTATION_SEND_INTERVALL == 0) {
    // uint32_t tmpTime; // Global
      // tmpTime verhindert, dass innerhalb derselben Minute mehrfach ein Snapshot erstellt wird
      if (actualMinute != tmpTime) {
        // Atomare Kopie der laufenden Zähler; Hauptloop liest estationInputCountToSend
        memcpy((void*)estationInputCountToSend, (const void*)estationInputCount, sizeof(estationInputCount));
        // Laufende Zähler zurücksetzen, damit das nächste Intervall sauber beginnt
        memset((void*)estationInputCount, 0, sizeof(estationInputCount));
        sendEStationValues = true;
        debugSendEStationValues = sendEStationValues;
        actualMinute = RtcTime.minute;
        debugActualMinuteAfter = actualMinute;
        tmpTime = actualMinute;
      }
    }
    // if (actualMinute + ESTATION_SEND_INTERVALL >= 60) {
    //   tmpTime = (actualMinute + ESTATION_SEND_INTERVALL) - 60;
    // } else {
    //   tmpTime = actualMinute + ESTATION_SEND_INTERVALL;
    // }
    debugtmpTime = tmpTime;
    // if (RtcTime.minute >= tmpTime) {
    //   memcpy((void*)estationInputCountToSend, (const void*)estationInputCount, sizeof(estationInputCount));
    //   sendEStationValues = true;
    //   debugSendEStationValues = sendEStationValues;
    //   memset((void*)estationInputCount, 0, sizeof(estationInputCount));
    //   actualMinute = RtcTime.minute;
    //   debugActualMinuteAfter = actualMinute;
    // }
  }

}


// Initialisierung: wird einmalig beim Tasmota-FUNC_INIT aufgerufen.
// Konfiguriert GPIO-Pins, hängt ISRs ein und startet den Hardware-Timer.
void EStationInit()
{

  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("e.station init..."));

  // Serial.begin(115200);

  if (EStationLoadData()) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: EStation loaded from file"));
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
  estation_timer = timerBegin(10000);
  timerAttachInterrupt(estation_timer, &EStationTimer_intr);
  timerAlarm(estation_timer, 10000, true, 0);

  // RGB-LED initialisieren und sofort auf Rot setzen (Startzustand: kein Netzwerk)
  // Pins sofort auf HIGH setzen (= alle LEDs aus bei common-anode), dann erst Output-Mode.
  // Ohne das würde der kurze LOW-Zustand nach pinMode() alle LEDs kurz aufleuchten lassen.
  digitalWrite(ESTATION_LED_PIN_BLUE,   HIGH);
  digitalWrite(ESTATION_LED_PIN_YELLOW, HIGH);
  digitalWrite(ESTATION_LED_PIN_RED,    HIGH);
  pinMode(ESTATION_LED_PIN_BLUE,   OUTPUT);
  pinMode(ESTATION_LED_PIN_YELLOW, OUTPUT);
  pinMode(ESTATION_LED_PIN_RED,    OUTPUT);
  EStationLED_set(ELED_RED);

  // Initialisierung erfolgreich – Verarbeitung im Hauptloop freigeben
  initSuccess = true;

}



// Hauptverarbeitungsroutine; wird alle 50 ms von FUNC_EVERY_50_MSECOND aufgerufen.
// Gibt ausgelöste Eingänge im Debug-Log aus und sendet bei gesetztem Flag den MQTT-Payload.
void EStationProcessing(void)
{
  // LED-Zustand aktualisieren (Netzwerkstatus, Impuls-Flash)
  EStationLED_update();

  // Pulse-Debug-Logging: jeder Kanal meldet sich einmalig nach dem ersten Impuls
    if (debugTimerFired) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 1: %d impulses"), estationInputCount[0]);
        estationInputStatus[0] = false;
    }
    if (estationInputStatus[1]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 2: %d impulses"), estationInputCount[1]);
        estationInputStatus[1] = false;
    }
    if (estationInputStatus[2]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 3: %d impulses"), estationInputCount[2]);
        estationInputStatus[2] = false;
    }
    if (estationInputStatus[3]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 4: %d impulses"), estationInputCount[3]);
        estationInputStatus[3] = false;
    }
    if (estationInputStatus[4]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 5: %d impulses"), estationInputCount[4]);
        estationInputStatus[4] = false;
    }
    if (estationInputStatus[5]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 6: %d impulses"), estationInputCount[5]);
        estationInputStatus[5] = false;
    }
    if (estationInputStatus[6]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 7: %d impulses"), estationInputCount[6]);
        estationInputStatus[6] = false;
    }
    if (estationInputStatus[7]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 8: %d impulses"), estationInputCount[7]);
        estationInputStatus[7] = false;
    }
    if (estationInputStatus[8]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 9: %d impulses"), estationInputCount[8]);
        estationInputStatus[8] = false;
    }
    if (estationInputStatus[9]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 10: %d impulses"), estationInputCount[9]);
        estationInputStatus[9] = false;
    }
    if (estationInputStatus[10]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 11: %d impulses"), estationInputCount[10]);
        estationInputStatus[10] = false;
    }
    if (estationInputStatus[11]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter 12: %d impulses"), estationInputCount[11]);
        estationInputStatus[11] = false;
    }

    // Timer-ISR-Diagnose: Debug-Ausgabe der Zustandswerte zum Zeitpunkt des Timer-Feuerns
    if (debugTimerFired) {
      debugTimerFired = false;
      AddLog(LOG_LEVEL_DEBUG, PSTR("e.station Timer fired..."));
      // AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugValidRtcTime: %d"), debugValidRtcTime);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> RtcMinute: %d"), RtcTime.minute);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugActualMinute before: %d"), debugActualMinute);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugtmpTime: %d"), debugtmpTime);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugActualMinute after: %d"), debugActualMinuteAfter);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> debugSendEStationValues: %d"), debugSendEStationValues);
      AddLog(LOG_LEVEL_DEBUG, PSTR("  --> sendEStationValues: %d"), sendEStationValues);
      AddLog(LOG_LEVEL_DEBUG, NetworkMacAddress().c_str());
      AddLog(LOG_LEVEL_DEBUG, NetworkAddress().toString().c_str());
      AddLog(LOG_LEVEL_DEBUG, NetworkUniqueId().c_str());
    }


    // MQTT-Versand: vom Timer-ISR gesetztes Flag auswerten, kWh berechnen und JSON senden
    if (sendEStationValues) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("e.station: sending values..."));
      sendEStationValues = false;
      // Zeitstempel für die Statusseite sichern
      strlcpy(estationLastSendTime, GetEStationDateAndTime().c_str(), sizeof(estationLastSendTime));
      // Impulszähler in kWh umrechnen: kWh = Impulse / (Imp/kWh * 1000 W/kW)
      for (uint32_t i = 0; i < 12; i++) {
        if ((estationInputCountToSend[i] > 0) && (s0counter_idx[i] > 0)) {
          AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter %d: %d impulses"), i+1, estationInputCountToSend[i]);
          // Serial.println(estationInputCountToSend[i]);
          estationInputWhToSend[i] = (float)estationInputCountToSend[i] / (float)s0counter_idx[i] *1000.0; // assuming Imp/kWh
          AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Counter %d: %d Wh"), i+1, estationInputWhToSend[i]);
          // Serial.println(estationInputWhToSend[i]);
          estationInputCountToSend[i] = 0;
        } else {
          estationInputWhToSend[i] = 0.0;
          estationInputCountToSend[i] = 0;
        }

        // MqttPublishPrefixTopicRulesProcess_P(TELE, subtopic, "Test");
        // Serial.println(PSTR(D_CMND_TOPIC));
      }
      // snprintf_P(bdt, sizeof(bdt), PSTR("%d" D_YEAR_MONTH_SEPARATOR "%02d" D_MONTH_DAY_SEPARATOR "%02d" D_DATE_TIME_SEPARATOR "%s"), year, month, day, PSTR(__TIME__));

      // Response_P(PSTR("{\"Time\":\"\":%d}}"), "25");

      // JSON-Payload zusammenbauen: Zeitstempel, Netzwerkinfos, Energiewerte aller 12 Kanäle
      String estationMQTTstring = "";

      estationMQTTstring = String(PSTR("{\"Time\":\""));
      estationMQTTstring += String(GetEStationDateAndTime());
      estationMQTTstring += String(PSTR("\",\"IpAdress\":\""));
      estationMQTTstring += NetworkAddress().toString();
      estationMQTTstring += String(PSTR("\",\"MacAdress\":\""));
      estationMQTTstring += NetworkMacAddress();
      estationMQTTstring += String(PSTR("\",\"UniqueNetworkId\":\""));
      estationMQTTstring += NetworkUniqueId();
      estationMQTTstring += String(PSTR("\",\"CounterValues\":{"));
      for (uint32_t i = 0; i < 12; i++) {
        estationMQTTstring += String(PSTR("\"Counter"));
        estationMQTTstring += String(i+1);
        estationMQTTstring += String(PSTR("\":"));
        estationMQTTstring += String(estationInputWhToSend[i], 3);
        if (i < 11) {
          estationMQTTstring += String(PSTR(","));
        }
      }
      estationMQTTstring += String(PSTR("}}"));;
      AddLog(LOG_LEVEL_INFO,PSTR(estationMQTTstring.c_str()));

      Response_P(PSTR(estationMQTTstring.c_str()));

      MqttPublishTeleSensor();
      memset((void*)estationInputCountToSend, 0, sizeof(estationInputCountToSend));
      // memset((void*)estationInputWhToSend, 0.0, sizeof(estationInputWhToSend));
    }
}


// Gibt den aktuellen RTC-Zeitstempel als ISO-8601-String zurück (z. B. "2024-06-18T14:30:00").
String GetEStationDateAndTime(void) {
  // "2017-03-07T11:08:02" - ISO8601:2004
  char estationdate[21];

  snprintf_P(estationdate, sizeof(estationdate), PSTR("%d" D_YEAR_MONTH_SEPARATOR "%02d" D_MONTH_DAY_SEPARATOR "%02d" D_DATE_TIME_SEPARATOR "%02d" D_HOUR_MINUTE_SEPARATOR "%02d" D_MINUTE_SECOND_SEPARATOR "%02d"), RtcTime.year, RtcTime.month, RtcTime.day_of_month, RtcTime.hour, RtcTime.minute, RtcTime.second);
  return String(estationdate);  // 2017-03-07T11:08:02
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

// Tasmota-Dispatch-Funktion: leitet Systemereignisse (Init, Zyklus, Webserver) an die
// entsprechenden e.station-Funktionen weiter.
bool Xdrv100(uint32_t function)
{


  bool result = false;


  if (FUNC_INIT == function) {
    EStationInit();
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("e.station init is done..."));
  }
  else if (initSuccess) {

    // Serial.print("Function: ");
    // Serial.println(function);

    //  if (function != 8) {
    //           Serial.println("e.station Web Default...");
    //           Serial.println(function);
    //         }
    switch (function) {

      case FUNC_EVERY_50_MSECOND:
        EStationProcessing();
        break;
#ifdef USE_WEBSERVER
// #ifndef FIRMWARE_MINIMAL    // not needed in minimal/safeboot because of disabled feature and Settings are not saved anyways
          case FUNC_WEB_ADD_MAIN_BUTTON:
              WSContentSend_P(HTTP_BTN_MENU_ESTATION_INFO);
              break;
          case FUNC_WEB_ADD_BUTTON:
              WSContentSend_P(HTTP_BTN_MENU_ESTATION);
              WSContentSend_P(HTTP_BTN_MENU_ESTATION_INFO);
              break;
          case FUNC_WEB_ADD_HANDLER:
              WebServer_on(PSTR("/" WEB_HANDLE_ESTATION), HandleEStationConfiguration);
              WebServer_on(PSTR("/" WEB_HANDLE_ESTATION_INFO), HandleEStationInfo);
              break;
      // #ifdef USE_WEB_STATUS_LINE
      //     case FUNC_WEB_STATUS_RIGHT:
      //         // if (MqttIsConnected()) {
      //         // if (MqttTLSEnabled()) {
      //         //     WSContentStatusSticker(PSTR(D_ESTATION_TLS_ENABLE));
      //         // } else {
      //         //     WSContentStatusSticker(PSTR(D_ESTATION));
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

#endif  // USE_ESTATION
