
/*
  Created by Edwin Kestler, & Dennis Revolorio sept 07 , 2018.
  @flatbox Guatemala Centro America.
  Released into the public domain under GPLv3.
*/

#include <Arduino.h>
// Librerias de ESP // MQTT/ JSON FORMAT data
#include <ESP8266WiFi.h>                                              //Libreira de ESPCORE ARDUINO
#include <PubSubClient.h>                                             //https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h>                                              //https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
//----------------------------------------------------------------------librerias de TIEMPO NTP
#include <TimeLibEsp.h>                                                  //TimeTracking
#include <WiFiUdp.h>                                                  //UDP packet handling for NTP request
//----------------------------------------------------------------------Librerias de manejo de setup de redes 
#include <ESP8266WebServer.h>                                         //Libreira de html para ESP8266
#include <DNSServer.h>                                                //Libreria de DNS para resolucion de Nombres
#include <WiFiManager.h>                                              //https://github.com/tzapu/WiFiManager
//----------------------------------------------------------------------Librerias de Codigo de Lectora RFID
#include <SoftwareSerial.h>
#include "settings.h"
//----------------------------------------------------------------------Libreria locales de clases de botron y LEDS
#include <BlinkRGB.h>
//---------------------------------------------------------------------------------RGB Settings
BlinkRGB Azul (D6);
BlinkRGB Verde (D7);
BlinkRGB Rojo (D8);

BlinkColor Blanco  (D6,D7,D8);
BlinkColor Purpura (D6,D4,D8);

//----------------------------------------------------------------------------------Buzzer Settings
const int beep = D5;
//----------------------------------------------------------------------------------RFID Settings

SoftwareSerial SSDEBUG(D2, D3, false, 256);

//----------------------------------------------------------------------------------json Data
String msg = "";
int WifiSignal;
//----------------------------------------------------------------------Poner el Pin de ADC en modo de sensar el voltaje da la bateria
int AnalogVCCPin = A0;                                              //Se opne el pin A0 en modo de Lectura interna 1.8V
float VBat = 0;
boolean BatWarningSent = false;
boolean flashWarning = false;
//----------------------------------------------------------------------Variables de verificacion de fallas de capa de conexion con servicio
int failed, sent, published;                                          //Variables de conteo de envios 
int BeepBatteryWarning = 0;
int BeepSignalWarning =0;
//------------------------------------------------------------------------------------FSM Settings
#define STATE_IDLE                    0
#define STATE_TRANSMIT_CARD_DATA      1
#define STATE_UPDATE                  2
#define STATE_TRANSMIT_ALARM_UPDATE   3
#define STATE_TRANSMIT_DEVICE_UPDATE  4
#define STATE_UPDATE_TIME             5
int fsm_state;
//----------------------------------------------------------------------Inicio de cliente UDP
WiFiUDP udp;                                                          //Cliente UDP para WIFI
//----------------------------------------------------------------------Codigo para estblecer el protocolo de tiempo en red NTP
const int NTP_PACKET_SIZE = 48;                                       //NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];                                   //Buffer to hold incoming & outgoing packets
boolean NTP = false;                                                  //Bandera que establece el estado inicial del valor de NTP
//----------------------------------------------------------------------Variables del servicio de envio de datos MQTT
const char* cserver = "";
//char authMethod[] = "use-token-auth";                                 //Tipo de Autenticacion para el servicio de Bluemix (la calve es unica por cada nodo)
//char token[] = TOKEN;                                                 //Variable donde se almacena el Token provisto por el servicio (ver Settings.h)
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;             //Variable de Identificacion de Cliente para servicio de MQTT Bluemix 
String  Smacaddrs = "00:00:00:00:00:00";
String  Sipaddrs  = "000.000.000.000";
//----------------------------------------------------------------------Declaracion de Variables Globales (procuar que sean las minimas requeridas.)
unsigned long lastUPDATEMillis;                                       //Variable para llevar conteo del tiempo desde la ultima publicacion 
unsigned long lastwarning;                                         //Variable para llevar conteo del tiempo desde la ultima publicacion 
unsigned long lastNResetMillis;                                       //Variable para llevar conteo del tiempo desde la ultima publicacion 
String ISO8601;                                                       //Variable para almacenar la marca del timepo (timestamp) de acuerdo al formtao ISO8601
int hora = 0;
//----------------------------------------------------------------------definir Parametros de Lector de RFID
unsigned long RetardoLectura;
String OldTagRead = "1";                                                    //VAriable para guardar la ultima tag leida y evitar lecturas consecutivas
String inputString;

//----------------------------------------------------------------------Variables Para el boton de emergencia
int IdEventoT= 0;
//----------------------------------------------------------------------Variables Propias del CORE ESP8266 Para la administracion del Modulo
String NodeID = String(ESP.getChipId());                              //Variable Global que contiene la identidad del nodo (ChipID) o numero unico
//------------------------------------------------------------------------------------denifinir el sonido de bocina
void buzzer() {
  digitalWrite(beep, HIGH);
  delay(300);
  digitalWrite(beep, LOW);
  delay(100);
}
//----------------------------------------------------------------------Funcion remota para administrar las actulizaciones remotas de las variables configurables desde IBMbluemix
void handleUpdate(byte* payload) {                                    //La Funcion recibe lo que obtenga Payload de la Funcion Callback que vigila el Topico de subcripcion (Subscribe TOPIC)
    StaticJsonBuffer<300> jsonBuffer;                                  //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
    JsonObject& root = jsonBuffer.parseObject((char*)payload);          //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
    if (!root.success()) {                                              //Si no se encuentra el objeto Raiz del Json
        Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
        return;                                                           //Nos salimos de la funcion
    }                                                                //se cierra el condicional
    Serial.println(F("handleUpdate payload:"));                         //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
    root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
    Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
}
//----------------------------------------------------------------------Funcion remota para mandar a dormir el esp despues de enviar un RFID
void handleResponse (byte* payloadrsp) {
    StaticJsonBuffer<200> jsonBuffer;                                   //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
    JsonObject& root = jsonBuffer.parseObject((char*)payloadrsp);       //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
    if (!root.success()) {                                                                              //Si no se encuentra el objeto Raiz del Json
        Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
        return;                                                           //Nos salimos de la funcion
    }                                                                   //se cierra el condicional

    Serial.println(F("handleResponse payload:"));                       //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
    root.printTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
    Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
}
//----------------------------------------------------------------------Funcion de vigilancia sobre mensajeria remota desde el servicion de IBM bluemix
void callback(char* topic, byte* payload, unsigned int payloadLength){//Esta Funcion vigila los mensajes que se reciben por medio de los Topicos de respuesta;
    Serial.print(F("callback invoked for topic: "));                    //Imprimir un mensaje seÃ±alando sobre que topico se recibio un mensaje
    Serial.println(topic);                                              //Imprimir el Topico
  
    if (strcmp (responseTopic, topic) == 0) {                            //verificar si el topico conicide con el Topico responseTopic[] definido en el archivo settings.h local
        handleResponse(payload);
        //return; // just print of response for now                         //Hacer algo si conicide (o en este caso hacer nada)
    }
    
    if (strcmp (rebootTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    ESP.reset();                                                    //Emitir comando de reinicio para ESP8266
    }

    if (strcmp (updateTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico updateTopic[] definido en el archivo settings.h local
    handleUpdate(payload);                                            //enviar a la funcion handleUpdate el contenido del mensaje para su parseo.
    }
}
//----------------------------------------------------------------------definicion de Cliente WIFI para ESP8266 y cliente de publicacion y subcripcion
WiFiClient wifiClient;                                                //Se establece el Cliente Wifi
PubSubClient client(MQTTServer, 1883, callback, wifiClient);              //se establece el Cliente para el servicio MQTT
//----------------------------------------------------------------------Funcion de Conexion a Servicio de MQTT
void mqttConnect() {
  if (!!!client.connected()) {                                         //Verificar si el cliente se encunetra conectado al servicio
  Serial.print(F("Reconnecting MQTT client to: "));                    //Si no se encuentra conectado imprimir un mensake de error y de reconexion al servicio
  Serial.println(MQTTServer);                                             //Imprimir la direccion del servidor a donde se esta intentado conectar 
  char charBuf[30];
  String CID (clientId + NodeID); 
  CID.toCharArray(charBuf, 30);  
  #if defined (internetS)
    while (!!!client.connect(charBuf, "flatboxadmin", "FBx_admin2012")) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    Blanco.CFlash();
    }  
  #else
    while (!!!client.connect(charBuf)) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    Blanco.CFlash();
    }  
  #endif  
  Serial.println();                                                   //dejar un espacio en la terminal para diferenciar los mensajes.
 }
}

//----------------------------------------------------------------------Funcion de REConexion a Servicio de MQTT
void MQTTreconnect() {
  int retry = 0;
  // Loop until we're reconnected
  while (!client.connected()) {    
    Serial.print(F("Attempting MQTT connection..."));
    Blanco.CFlash();
    buzzer();
    char charBuf[30];
    String CID (clientId + NodeID);
    CID.toCharArray(charBuf, 30);  
     #if defined (internetS)
     if (client.connect(charBuf, "flatboxadmin", "FBx_admin2012")) {
      Serial.println(F("connected"));
     }
     #else
     if (client.connect(charBuf)) {
      Serial.println(F("connected"));
     }
     #endif
     else {
      Purpura.CFlash();
      buzzer();
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.print(F(" try again in 3 seconds,"));
      Serial.print(F(" retry #:"));
      Serial.println(retry);
      if (retry > 10){
        ESP.restart();
        retry=0;
      }
      retry++;
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
}

//----------------------------------------------------------------------Funcion encargada de subscribir el nodo a los servicio de administracion remota y de notificar los para metros configurables al mismo
void initManagedDevice() {
  if (client.subscribe("iotdm-1/response")) {                         //Subscribir el nodo al servicio de mensajeria de respuesta
    Serial.println(F("subscribe to responses OK"));                   //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to responses FAILED"));               //Si no se logra la subcripcion imprimir un mensaje de error
  }
  
  if (client.subscribe(rebootTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to reboot OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to reboot FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
  }
  
  if (client.subscribe("iotdm-1/device/update")) {                    //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to update OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to update FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error         
  }
  
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["UInterval"] = UInterval;
  metadata["UPDATETIME"] = 60*UInterval;
  metadata["NResetTIME"] = 60*60*UInterval;
  metadata["timeZone"] = timeZone;    
  JsonObject& supports = d.createNestedObject("supports");
  supports["deviceActions"] = true;  
  JsonObject& deviceInfo = d.createNestedObject("deviceInfo");
  deviceInfo["ntpServerName"] = ntpServerName;
  deviceInfo["server"] = MQTTServer;
  deviceInfo["MacAddress"] = Smacaddrs;
  deviceInfo["IPAddress"]= Sipaddrs;    
  char buff[500];
  root.printTo(buff, sizeof(buff));
  Serial.println(F("publishing device manageTopic metadata:"));
  Serial.println(buff);
  sent++;
  if (client.publish(manageTopic, buff)) {
    Serial.println(F("device Publish ok"));
  }else {
    Serial.println(F("device Publish failed:"));
  }
}

//----------------------------------------------------------------------send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//----------------------------------------------------------------------Funcion para obtener el paquee de TP y procesasr la fecha hora desde el servidor de NTP
time_t getNtpTime(){
  while (udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      NTP = true;
      udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}


//----------------------------------------------------------------------anager function. Configure the wifi connection if not connect put in mode AP--------//
void wifimanager() {
  WiFiManager wifiManager;
  Serial.println(F("empezando"));
  Purpura.COn();
  if (!  wifiManager.autoConnect("flatwifi")) {
    Purpura.CFlash();
    if (!wifiManager.startConfigPortal("flatwifi")) {
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5 * UInterval);
    }
  }
}

//----------------------------------------------------------------------anager function. Configure the wifi connection if not connect put in mode AP--------//
void OnDemandWifimanager() {
  WiFiManager wifiManager;
  Serial.println(F("Empezando Configuracion de WIFI Bajo Demanda"));
  Purpura.COn();
  if (!wifiManager.startConfigPortal("flatwifi")) {
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5 * UInterval);
  }
}
//-----------------------------------------------------------------------------------Setting up ESP8266 scketch
void setup() {
  pinMode(beep, OUTPUT);
  digitalWrite(beep, LOW);
  Blanco.COff();
  Serial.begin(115200);
  SSDEBUG.begin(9600);
  Serial.println(F("")); 
  Serial.println(F("Inicializacion de programa de boton con identificacion RFID;"));
  Serial.println(F("Parametros de ambiente de funcionamiento:"));
  Serial.print(F("            CHIPID: "));
  Serial.println(NodeID);
  Serial.print(F("            HARDWARE: "));
  Serial.println(HardwareVersion);
  Serial.print(F("            FIRMWARE: "));
  Serial.println(FirmwareVersion);
  Serial.print(F("            Servidor de NTP: "));
  Serial.println(ntpServerName);
  Serial.print(F("            Servidor de MQTT: "));
  Serial.println(MQTTServer);
  Serial.print(F("            Client ID: "));
  Serial.println(clientId); 
  delay(UInterval); 
  //--------------------------------------------------------------------------Configuracion Automatica de Wifi   
  while (WiFi.status() != WL_CONNECTED) {                                   //conectamos al wifi si no hay la rutina iniciara una pagina web de configuracion en la direccion 192.168.4.1 
    wifimanager();
    delay(UInterval);
  }
  Serial.print(F("Wifi conectado, Direccion de IP Asignado: "));
  Serial.println(WiFi.localIP());
  Sipaddrs = WiFi.localIP().toString();
  Serial.print(F("Direccion de MAC Asignado: "));
  Serial.println(WiFi.macAddress());
  Smacaddrs = String(WiFi.macAddress());
  Serial.println(F(""));                                                         //dejamos una linea en blanco en la terminal 
  //una vez contados al Wifi nos aseguramos tener la hora correcta simepre
  Serial.println(F("Connected to WiFi, sincronizando con el NTP;"));                    //mensaje de depuracion para saber que se intentara obtner la hora
  //--------------------------------------------------------------------------Configuracion de NTP
  Serial.print(F("servidor de NTP:"));
  Serial.println(ntpServerName);
  //--------------------------------------------------------------------------Configuracion de UDP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  while (NTP == false) {
    setSyncProvider(getNtpTime);                                                          //iniciamos la mensajeria de UDP para consultar la hora en el servicio de NTP remoto (el servidor se configura en 
    delay(UInterval);
  }
  NTP = false;
  //--------------------------------------------------------------------------Connectando a servicio de MQTT
  Serial.println(F("Time Sync, Connecting to mqtt sevrer"));
  mqttConnect();                                                            //Conectamos al servicio de Mqtt con las credenciales provistas en el archivo "settings.h"
  Serial.println(F("Mqtt Connection Done!, sending Device Data"));
  //--------------------------------------------------------------------------Enviando datos de primera conexion
  initManagedDevice();                                                      //inciamos la administracion remota desde Bluemix
  Serial.println(F("Finalizing Setup"));                                    //enviamos un mensaje de depuracion
  Blanco.COff();
  fsm_state = STATE_IDLE; //inciar el estado del la maquina de stado finito
  yield();
}

//-------- Data de Manejo RF_ID_Manejo. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//
void publishRF_ID_Manejo (String IDModulo,String MSG,float vValue,int RSSIV, int env, int fail,String Tstamp, String SMacAd, String SIpAd){
  StaticJsonBuffer<300> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& Ddata = d.createNestedObject("Ddata");
  Ddata["ChipID"] = IDModulo;
  Ddata["Msg"] = MSG;
  Ddata["batt"] = vValue;
  Ddata["RSSI"] = RSSIV;
  Ddata["publicados"] = env;
  Ddata["enviados"] = sent;
  Ddata["fallidos"] = fail;
  Ddata["Tstamp"] = Tstamp;
  Ddata["Mac"] = SMacAd;
  Ddata["Ip"] = SIpAd;
  char MqttDevicedata[300];
  root.printTo(MqttDevicedata, sizeof(MqttDevicedata));
  Serial.println(F("publishing device data to manageTopic:"));
  Serial.println(MqttDevicedata);
  sent++;
  if (client.publish(manageTopic, MqttDevicedata)) {
     Serial.println(F("enviado data de dispositivo:OK"));
     published ++;
     failed = 0; 
  }else {
    Serial.print(F("enviado data de dispositivo:FAILED"));
    failed ++;
  }
}
//------------------------------------------------------------------------------------Leer la tarjeta que se presenta
void readTag() {
  
  return;
}

//--------------------------------------------------------------------------Funcion de Verificacion de bateria------------------------------------------------------------------------------
float Bateria(){
 //int sensorValue = 4.2; //analogRead(AnalogVCCPin);
 //float volt = sensorValue;
 float volt = 821.14;
 volt = volt / 221.93;
 return volt;
}
//--------------------------------------------------------------------------------------Parsear la informacion de la tartjeta leida. (opcional)

//------------------------------------------------------------------------------------------------Funcion de reseteo normal
void NormalReset(){
  if (millis()- lastNResetMillis > 60 * 60 * UInterval){
    hora++;
    WifiSignal = WiFi.RSSI();
    if (hora > 24){
      msg = ("24h NReset");  
      VBat = 4.2; //Bateria();
      publishRF_ID_Manejo(NodeID, msg, VBat, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);        //publishRF_ID_Manejo (String IDModulo,String MSG,float vValue, int fail,String Tstamp)
      void disconnect ();
      hora = 0;
      ESP.restart();
    }
    lastNResetMillis = millis(); //Actulizar la ultima hora de envio
  }
}
//--------------------------------------------------------------------------Funcion de checkear alarmas.!!!------------------------------------------------------------------------------
void checkalarms (){
      if (WiFi.RSSI() < -85){
        if(BeepSignalWarning < 4){
          buzzer();
          BeepSignalWarning++;
        }
        Blanco.CFlash();
      }
      BeepSignalWarning = 0;
  }

//--------------------------------------------------------------------------Funcion dealarmas locales Flash luces y bocina!!!------------------------------------------------------------------------------
void LocalWarning (){
     if (millis()- lastwarning > UInterval){
      lastwarning =millis();
      if (flashWarning == true){
        Rojo.Flash();
        if(BeepBatteryWarning < 4){
        buzzer();
        BeepBatteryWarning ++;
        }
        }else{
          if (Bateria() > BATTRESHHOLD ){
            BatWarningSent = true;
            flashWarning = false;
            BeepBatteryWarning = 0;
          }
        }
     }
  }      

//--------------------------------------------------------------------------Funcion de publicar los datos de estado si ha pasado el tiempo establecido entonces*!!------------------------------------------------------------------------------
void updateDeviceInfo(){
  msg = ("on");
  VBat = Bateria();
  WifiSignal = WiFi.RSSI();
  if (WiFi.RSSI() < -75){
    msg = ("LOWiFi");
    Rojo.Flash();
    buzzer();
    Serial.print(WiFi.SSID());
    Serial.print(" ");
    Serial.println(WiFi.RSSI());
    fsm_state = STATE_TRANSMIT_ALARM_UPDATE; //publishRF_ID_Manejo(NodeID, msg, VBat, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);        //publishRF_ID_Manejo (String IDModulo,String MSG,float vValue, int fail,String Tstamp)
    return;
  }
  if (Bateria() < BATTRESHHOLD ){
    flashWarning = true;
    buzzer();
    msg = ("LowBat");
    if (BatWarningSent == false){
      fsm_state = STATE_TRANSMIT_ALARM_UPDATE; //publishRF_ID_Manejo(NodeID, msg, VBat, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);
      BatWarningSent = true;
    }
    return;
  }
  if (Bateria() > BATTRESHHOLD ){
    BatWarningSent = true;
    flashWarning = false;
  }
 }

//----------------------------------------------------------------------------funcion que procesa como desplegar y transmitir la hora de acuerdo al formato del ISO8601
void CheckTime(){ //digital clock display of the time
  time_t prevDisplay = 0; 
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) {                                             //update the display only if time has changed
      prevDisplay = now();
      ISO8601 = String (year(), DEC);
      ISO8601 += "-";
      ISO8601 += month();
      ISO8601 += "-";
      ISO8601 += day();
      ISO8601 +="T";
      if ((hour() >= 0)&& (hour() < 10)){
        //Serial.print(F("+0:"));
        //Serial.println(hour());
        ISO8601 +="0";
        ISO8601 += hour();
      }else{
        //Serial.print(F("hora:"));
        //Serial.println(hour());
        ISO8601 += hour();
      }
      ISO8601 += ":";
      ISO8601 += minute();
      ISO8601 += ":";
      ISO8601 += second();
    }
  }
}

//-------- funcion datos Lectura Tag RF_ID_LECTURA. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//
void publishRF_ID_Lectura(String IDModulo, String Tstamp, String tagread) {
  if (OldTagRead != tagread){
    OldTagRead = tagread;
    IdEventoT ++;
    String IDEventoT = String (NodeID + IdEventoT);
    StaticJsonBuffer<250> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& tagdata = d.createNestedObject("tagdata");
    tagdata["ChipID"] = IDModulo;
    tagdata["IDeventoTag"]= IDEventoT;
    tagdata["Tstamp"] = Tstamp;
    tagdata["Tag"] = tagread;
    char MqttTagdata[250];
    root.printTo(MqttTagdata, sizeof(MqttTagdata));
    Serial.println(F("publishing Tag data to publishTopic:")); 
    Serial.println(MqttTagdata);
    sent ++;
    if (client.publish(publishTopic, MqttTagdata)){
      Serial.println(F("enviado data de RFID: OK"));
      Verde.Flash();
      buzzer();
      published ++;
      inputString = "";
      failed = 0; 
      }else {
        Serial.println(F("enviado data de RFID: FAILED"));
        Rojo.Flash();
        failed ++;
        OldTagRead = "1";
        inputString = "";
      }
  }else{
    Serial.println("Este es una lectura consecutiva");
  }
}

//*******************************************************************************************************VOID LOOP*******************************************************
void loop() {
    switch(fsm_state){                                                                                  // inciar el casw switch
    case STATE_IDLE: // hacer cuando el estado sea IDLE
    readTag(); //leer su hay alguna tarjeta
    NormalReset();
    checkalarms();
    LocalWarning ();
    if(millis() - lastUPDATEMillis > 30*60*UInterval) {
        lastUPDATEMillis = millis(); //Actulizar la ultima hora de envio
        fsm_state = STATE_UPDATE;
    }
    
    if(millis() - lastUPDATEMillis > 60*60*UInterval) {
        lastUPDATEMillis = millis(); //Actulizar la ultima hora de envio
        fsm_state = STATE_UPDATE_TIME;
    }
    
    if ( millis() - RetardoLectura > 5 * UInterval){
        OldTagRead = "1";
        RetardoLectura = millis(); //Actulizar la ultima hora de envio
    }
    // VERIFICAMOS CUANTAS VECES NO SE HAN ENVIOADO PAQUETES (ERRORES)
    if (failed >= FAILTRESHOLD){
        failed =0;
        published =0;
        sent=0;    
        ESP.restart();
    }
    //verificar que el cliente de Conexion al servicio se encuentre conectado
    if (!client.connected()) {
        MQTTreconnect();
    }
    client.loop();
    break;
    //**************************************************************************************************STATE_TRANSMIT_CARD_DATA*****************************************
    case STATE_TRANSMIT_CARD_DATA:
    //Build the Json
    //check connection
    //Send the card data
    Serial.println(F("CARD DATA SENT"));
    CheckTime();
    publishRF_ID_Lectura(NodeID,ISO8601,inputString);
    fsm_state = STATE_IDLE; 
    break;
    //**************************************************************************************************STATE_UPDATE*****************************************************
    case STATE_UPDATE:
    Serial.println(F("STATE_UPDATE"));
    updateDeviceInfo();
    fsm_state = STATE_TRANSMIT_DEVICE_UPDATE;
    break;
    //**************************************************************************************************STATE_TRANSMIT_DEVICE_UPDATE*************************************
    case STATE_TRANSMIT_DEVICE_UPDATE:
    Serial.println(F("STATE_TRANSMIT_DEVICE_UPDATE"));
    //verificar que el cliente de Conexion al servicio se encuentre conectado
    if (!client.connected()) {
        MQTTreconnect();
    }
    //verificar la hora
    CheckTime();
    publishRF_ID_Manejo(NodeID, msg, VBat, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);
    fsm_state = STATE_IDLE;
    break;
    //**************************************************************************************************STATE_TRANSMIT_ALARM_UPDATE**************************************
    case STATE_TRANSMIT_ALARM_UPDATE:
    Serial.println(F("STATE_TRANSMIT_ALARM_UPDATE"));
    //verificar que el cliente de Conexion al servicio se encuentre conectado
    if (!client.connected()) {
        MQTTreconnect();
    }
    // Verificar la hora
    CheckTime();
    publishRF_ID_Manejo(NodeID,msg, VBat, WifiSignal, published, failed, ISO8601, Smacaddrs, Sipaddrs);
    break;
    //**************************************************************************************************STATE_UPDATE_TIME************************************************
    case STATE_UPDATE_TIME:
    Serial.println(F("Starting UDP"));
    udp.begin(localPort);
    Serial.print(("Local port: "));
    Serial.println(udp.localPort());
    while (NTP == false) {
        setSyncProvider(getNtpTime);                                                                    //iniciamos la mensajeria de UDP para consultar la hora en el servicio de NTP remoto (el servidor se configura en 
        delay(UInterval);
    }                                                                                                   //Cuando fue actualizada la hora del reloj
    NTP = false;
    fsm_state = STATE_IDLE; 
    break;
    }
    yield();
}