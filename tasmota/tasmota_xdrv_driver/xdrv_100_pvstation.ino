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




// Forward declaration needed to preserve IRAM_ATTR before auto-prototype generation
void IRAM_ATTR PVStationTimer_intr();

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

  // Initialisierung erfolgreich – Verarbeitung im Hauptloop freigeben
  initSuccess = true;

}



// Hauptverarbeitungsroutine; wird alle 100 ms von FUNC_EVERY_100_MSECOND aufgerufen.
// Gibt ausgelöste Eingänge im Debug-Log aus und sendet bei gesetztem Flag den MQTT-Payload.
void PVStationProcessing(void)
{
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
