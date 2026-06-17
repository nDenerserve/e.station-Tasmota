/*
  xdrv_100_pvstation.ino - My IoT device support for Tasmota
*/


#ifdef USE_PVSTATION
/*********************************************************************************************\
 * My IoT Device bare minimum
 *
 *
\*********************************************************************************************/

#define XDRV_100 100
#define XDRV_100_KEY                      "pvstationdrv100"

#define D_CONFIGURE_PVSTATION "PV-Station"
#define D_PVSTATION_PARAMETERS "PV-Station Parameters"
#define D_PVSTATION_COUNTER "Zähler"
#define D_PVSTATION_IMPKWH "Imp/kWh"

#define D_PVSTATION_JSON_S0 "s0counter"

#define PVSTATION_SEND_INTERVALL        1    


/*********************************************************************************************\
 * Tasmota Functions
\*********************************************************************************************/

// This variable will be set to true after initialization
bool initSuccess = false;
volatile uint32_t s0counter_idx[12] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};

static hw_timer_t *pvstation_timer = nullptr;


volatile uint32_t actualMinute = 0;
// volatile bool validRtcTime = false;
volatile bool sendPVStationValues = false;

volatile uint32_t debugActualMinute = 0;
volatile uint32_t debugActualMinuteAfter = 0;
// volatile bool debugValidRtcTime = false;
volatile bool debugSendPVStationValues = false;
volatile bool debugTimerFired = false;
volatile uint32_t debugtmpTime = 0;
volatile uint32_t tmpTime;

volatile bool pvstationInputStatus[12] = {false,false,false,false,false,false,false,false,false,false,false,false};
volatile uint32_t pvstationInputCount[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
volatile uint32_t pvstationInputCountToSend[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
volatile float pvstationInputKwhToSend[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};



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

#define WEB_HANDLE_PVSTATION "pvs"

const char S_CONFIGURE_PVSTATION[] PROGMEM = D_CONFIGURE_PVSTATION;

const char HTTP_BTN_MENU_PVSTATION[] PROGMEM =
  "<p></p><form action='" WEB_HANDLE_PVSTATION "' method='get'><button>" D_CONFIGURE_PVSTATION "</button></form>";

const char HTTP_FORM_PVSTATION1[] PROGMEM =
  "<fieldset><legend><b>&nbsp;" D_PVSTATION_PARAMETERS "&nbsp;</b></legend>"
  "<form method='get' action='" WEB_HANDLE_PVSTATION "'>"
  "<table>";
const char HTTP_FORM_PVSTATION_COUNTER[] PROGMEM =
  "<tr><td style='width:260px'><b>" D_PVSTATION_COUNTER " %d</b></td><td style='width:70px'><input id='c%d' placeholder='1000' value='%d'></td><td> " D_PVSTATION_IMPKWH "</td></tr>";
  

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

// Interrupt Service Routine (ISR)
void ARDUINO_ISR_ATTR input1ISR() {
  pvstationInputStatus[0] = true;
  pvstationInputCount[0]++;
}
void ARDUINO_ISR_ATTR input2ISR() {
  pvstationInputStatus[1] = true;
  pvstationInputCount[1]++;
}
void ARDUINO_ISR_ATTR input3ISR() {
  pvstationInputStatus[2] = true;
  pvstationInputCount[2]++;
}
void ARDUINO_ISR_ATTR input4ISR() {
  pvstationInputStatus[3] = true;
  pvstationInputCount[3]++;
}
void ARDUINO_ISR_ATTR input5ISR() {
  pvstationInputStatus[4] = true;
  pvstationInputCount[4]++;
}
void ARDUINO_ISR_ATTR input6ISR() {
  pvstationInputStatus[5] = true;
  pvstationInputCount[5]++;
}
void ARDUINO_ISR_ATTR input7ISR() {
  pvstationInputStatus[6] = true;
  pvstationInputCount[6]++;
}
void ARDUINO_ISR_ATTR input8ISR() {
  pvstationInputStatus[7] = true;
  pvstationInputCount[7]++;
}
void ARDUINO_ISR_ATTR input9ISR() {
  pvstationInputStatus[8] = true;
  pvstationInputCount[8]++;
}
void ARDUINO_ISR_ATTR input10ISR() {
  pvstationInputStatus[9] = true;
  pvstationInputCount[9]++;
}
void ARDUINO_ISR_ATTR input11ISR() {
  pvstationInputStatus[10] = true;
  pvstationInputCount[10]++;
}
void ARDUINO_ISR_ATTR input12ISR() {
  pvstationInputStatus[11] = true;
  pvstationInputCount[11]++;
}

void IRAM_ATTR PVStationTimer_intr() {

  debugTimerFired = true;
  debugActualMinute = actualMinute;
  // debugValidRtcTime = validRtcTime;
  

  if (!RtcTime.valid) {
    return;
  } else if (RtcTime.valid ) {
    // validRtcTime = true;
    actualMinute = RtcTime.minute;
    if (actualMinute % PVSTATION_SEND_INTERVALL == 0) {
    // uint32_t tmpTime; // Global
      if (actualMinute != tmpTime) {
        memcpy((void*)pvstationInputCountToSend, (const void*)pvstationInputCount, sizeof(pvstationInputCount));
        sendPVStationValues = true;
        debugSendPVStationValues = sendPVStationValues;
        memset((void*)pvstationInputCount, 0, sizeof(pvstationInputCount));
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


void PVStationInit()
{

  /*
    Here goes My Project setting.
    Usually this part is included into setup() function
  */


  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("PV Station init..."));

  // Serial.begin(115200);

  if (PVStationLoadData()) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("CFG: PVStation loaded from file"));
  }

  

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

  

  pvstation_timer = timerBegin(10000); 
  timerAttachInterrupt(pvstation_timer, &PVStationTimer_intr);
  timerAlarm(pvstation_timer, 10000, true, 0);

  // Set initSuccess at the very end of the init process
  // Init is successful
  initSuccess = true;

}



void PVStationProcessing(void)
{

  /*
    Here goes My Project code.
    Usually this part is included into loop() function
  */
  
    if (pvstationInputStatus[0]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 1 was triggered!"));
        pvstationInputStatus[0] = false;
    }
    if (pvstationInputStatus[1]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 2 was triggered!"));
        pvstationInputStatus[1] = false;
    }
    if (pvstationInputStatus[2]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 3 was triggered!"));
        pvstationInputStatus[2] = false;
    }
    if (pvstationInputStatus[3]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 4 was triggered!"));
        pvstationInputStatus[3] = false;
    }
    if (pvstationInputStatus[4]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 5 was triggered!"));
        pvstationInputStatus[4] = false;
    }
    if (pvstationInputStatus[5]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 6 was triggered!"));
        pvstationInputStatus[5] = false;
    }
    if (pvstationInputStatus[6]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 7 was triggered!"));
        pvstationInputStatus[6] = false;
    }
    if (pvstationInputStatus[7]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 8 was triggered!"));
        pvstationInputStatus[7] = false;
    }
    if (pvstationInputStatus[8]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 9 was triggered!"));
        pvstationInputStatus[8] = false;
    }
    if (pvstationInputStatus[9]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 10 was triggered!"));
        pvstationInputStatus[9] = false;
    }
    if (pvstationInputStatus[10]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 11 was triggered!"));
        pvstationInputStatus[10] = false;
    }
    if (pvstationInputStatus[11]) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("  --> Input 12 was triggered!"));
        pvstationInputStatus[11] = false;
    }

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


    if (sendPVStationValues) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("PV-Station: sending values..."));
      sendPVStationValues = false;
      for (uint32_t i = 0; i < 12; i++) {
        if ((pvstationInputCountToSend[i] > 0) && (s0counter_idx[i] > 0)) {
          Serial.println(pvstationInputCountToSend[i]);
          pvstationInputKwhToSend[i] = (float)pvstationInputCountToSend[i] / (float)s0counter_idx[i] *1000.0; // assuming Imp/kWh 
          Serial.println(pvstationInputKwhToSend[i]);
          pvstationInputCountToSend[i] = 0;
        } else {
          pvstationInputKwhToSend[i] = 0.0;
          pvstationInputCountToSend[i] = 0;
        }
        
        // MqttPublishPrefixTopicRulesProcess_P(TELE, subtopic, "Test");
        // Serial.println(PSTR(D_CMND_TOPIC));
      }
      // snprintf_P(bdt, sizeof(bdt), PSTR("%d" D_YEAR_MONTH_SEPARATOR "%02d" D_MONTH_DAY_SEPARATOR "%02d" D_DATE_TIME_SEPARATOR "%s"), year, month, day, PSTR(__TIME__));
     
      // Response_P(PSTR("{\"Time\":\"\":%d}}"), "25");
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
        pvstationMQTTstring += String(pvstationInputKwhToSend[i], 3);
        if (i < 11) {
          pvstationMQTTstring += String(PSTR(","));
        }
      }
      pvstationMQTTstring += String(PSTR("}}"));;
      AddLog(LOG_LEVEL_DEBUG,PSTR(pvstationMQTTstring.c_str()));

      Response_P(PSTR(pvstationMQTTstring.c_str()));

      MqttPublishTeleSensor();
      memset((void*)pvstationInputCountToSend, 0, sizeof(pvstationInputCountToSend));
      // memset((void*)pvstationInputKwhToSend, 0.0, sizeof(pvstationInputKwhToSend));
    }
}


String GetPVStationDateAndTime(void) {
  // "2017-03-07T11:08:02" - ISO8601:2004
  char pvstationdate[21];
  
  snprintf_P(pvstationdate, sizeof(pvstationdate), PSTR("%d" D_YEAR_MONTH_SEPARATOR "%02d" D_MONTH_DAY_SEPARATOR "%02d" D_DATE_TIME_SEPARATOR "%02d" D_HOUR_MINUTE_SEPARATOR "%02d" D_MINUTE_SECOND_SEPARATOR "%02d"), RtcTime.year, RtcTime.month, RtcTime.day_of_month, RtcTime.hour, RtcTime.minute, RtcTime.second);
  return String(pvstationdate);  // 2017-03-07T11:08:02
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/
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

      case FUNC_EVERY_100_MSECOND:
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