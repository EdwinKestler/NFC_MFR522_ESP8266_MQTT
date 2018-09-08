
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
String Analisis = "";
String CARD_ID = "";
String LAST_ID = "";
bool RECORD=LOW;
SoftwareSerial SSDEBUG(D1, D2, false, 256); // Rx, Tx
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
unsigned long Check_connection_mqtt;                                                                    //Variable donde se define cada cuanto se chequea conecion conel servidor de mqtt 
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
        SSDEBUG.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
        return;                                                           //Nos salimos de la funcion
    }                                                                //se cierra el condicional
    SSDEBUG.println(F("handleUpdate payload:"));                         //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
    root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
    SSDEBUG.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
}
//----------------------------------------------------------------------Funcion remota para mandar a dormir el esp despues de enviar un RFID
void handleResponse (byte* payloadrsp) {
    StaticJsonBuffer<200> jsonBuffer;                                   //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
    JsonObject& root = jsonBuffer.parseObject((char*)payloadrsp);       //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
    if (!root.success()) {                                                                              //Si no se encuentra el objeto Raiz del Json
        SSDEBUG.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
        return;                                                           //Nos salimos de la funcion
    }                                                                   //se cierra el condicional

    SSDEBUG.println(F("handleResponse payload:"));                       //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
    root.printTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
    SSDEBUG.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
}
//----------------------------------------------------------------------Funcion de vigilancia sobre mensajeria remota desde el servicion de IBM bluemix
void callback(char* topic, byte* payload, unsigned int payloadLength){//Esta Funcion vigila los mensajes que se reciben por medio de los Topicos de respuesta;
    SSDEBUG.print(F("callback invoked for topic: "));                    //Imprimir un mensaje seÃ±alando sobre que topico se recibio un mensaje
    SSDEBUG.println(topic);                                              //Imprimir el Topico
  
    if (strcmp (responseTopic, topic) == 0) {                            //verificar si el topico conicide con el Topico responseTopic[] definido en el archivo settings.h local
        handleResponse(payload);
        //return; // just print of response for now                         //Hacer algo si conicide (o en este caso hacer nada)
    }
    
    if (strcmp (rebootTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    SSDEBUG.println(F("Rebooting..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
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
  SSDEBUG.print(F("Reconnecting MQTT client to: "));                    //Si no se encuentra conectado imprimir un mensake de error y de reconexion al servicio
  SSDEBUG.println(MQTTServer);                                             //Imprimir la direccion del servidor a donde se esta intentado conectar 
  char charBuf[30];
  String CID (clientId + NodeID); 
  CID.toCharArray(charBuf, 30);  
  #if defined (internetS)
    while (!!!client.connect(charBuf, "flatboxadmin", "FBx_admin2012")) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    SSDEBUG.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    Blanco.CFlash();
    }  
  #else
    while (!!!client.connect(charBuf)) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    SSDEBUG.print(F("."));                                             //imprimir una serie de puntos mientras se da la conexion al servicio
    Blanco.CFlash();
    }  
  #endif  
  SSDEBUG.println();                                                   //dejar un espacio en la terminal para diferenciar los mensajes.
 }
}

//----------------------------------------------------------------------Funcion de REConexion a Servicio de MQTT
void MQTTreconnect() {
  int retry = 0;
  // Loop until we're reconnected
  while (!client.connected()) {    
    SSDEBUG.print(F("Attempting MQTT connection..."));
    Blanco.CFlash();
    buzzer();
    char charBuf[30];
    String CID (clientId + NodeID);
    CID.toCharArray(charBuf, 30);  
     #if defined (internetS)
     if (client.connect(charBuf, "flatboxadmin", "FBx_admin2012")) {
      SSDEBUG.println(F("connected"));
     }
     #else
     if (client.connect(charBuf)) {
      SSDEBUG.println(F("connected"));
     }
     #endif
     else {
      Purpura.CFlash();
      buzzer();
      SSDEBUG.print(F("failed, rc="));
      SSDEBUG.print(client.state());
      SSDEBUG.print(F(" try again in 3 seconds,"));
      SSDEBUG.print(F(" retry #:"));
      SSDEBUG.println(retry);
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
    SSDEBUG.println(F("subscribe to responses OK"));                   //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    SSDEBUG.println(F("subscribe to responses FAILED"));               //Si no se logra la subcripcion imprimir un mensaje de error
  }
  
  if (client.subscribe(rebootTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    SSDEBUG.println(F("subscribe to reboot OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    SSDEBUG.println(F("subscribe to reboot FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
  }
  
  if (client.subscribe("iotdm-1/device/update")) {                    //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    SSDEBUG.println(F("subscribe to update OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    SSDEBUG.println(F("subscribe to update FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error         
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
  SSDEBUG.println(F("publishing device manageTopic metadata:"));
  SSDEBUG.println(buff);
  sent++;
  if (client.publish(manageTopic, buff)) {
    SSDEBUG.println(F("device Publish ok"));
  }else {
    SSDEBUG.println(F("device Publish failed:"));
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
  SSDEBUG.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      SSDEBUG.println(F("Receive NTP Response"));
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
  SSDEBUG.println(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}


//----------------------------------------------------------------------anager function. Configure the wifi connection if not connect put in mode AP--------//
void wifimanager() {
  WiFiManager wifiManager;
  SSDEBUG.println(F("empezando"));
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
  SSDEBUG.println(F("Empezando Configuracion de WIFI Bajo Demanda"));
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
  SSDEBUG.begin(9600);
  Serial.begin(115200);
  SSDEBUG.println(F("")); 
  SSDEBUG.println(F("Inicializacion de programa de boton con identificacion RFID;"));
  SSDEBUG.println(F("Parametros de ambiente de funcionamiento:"));
  SSDEBUG.print(F("            CHIPID: "));
  SSDEBUG.println(NodeID);
  SSDEBUG.print(F("            HARDWARE: "));
  SSDEBUG.println(HardwareVersion);
  SSDEBUG.print(F("            FIRMWARE: "));
  SSDEBUG.println(FirmwareVersion);
  SSDEBUG.print(F("            Servidor de NTP: "));
  SSDEBUG.println(ntpServerName);
  SSDEBUG.print(F("            Servidor de MQTT: "));
  SSDEBUG.println(MQTTServer);
  SSDEBUG.print(F("            Client ID: "));
  SSDEBUG.println(clientId); 
  delay(UInterval); 
  //--------------------------------------------------------------------------Configuracion Automatica de Wifi   
  while (WiFi.status() != WL_CONNECTED) {                                   //conectamos al wifi si no hay la rutina iniciara una pagina web de configuracion en la direccion 192.168.4.1 
    wifimanager();
    delay(UInterval);
  }
  SSDEBUG.print(F("Wifi conectado, Direccion de IP Asignado: "));
  SSDEBUG.println(WiFi.localIP());
  Sipaddrs = WiFi.localIP().toString();
  SSDEBUG.print(F("Direccion de MAC Asignado: "));
  SSDEBUG.println(WiFi.macAddress());
  Smacaddrs = String(WiFi.macAddress());
  SSDEBUG.println(F(""));                                                         //dejamos una linea en blanco en la terminal 
  //una vez contados al Wifi nos aseguramos tener la hora correcta simepre
  SSDEBUG.println(F("Connected to WiFi, sincronizando con el NTP;"));                    //mensaje de depuracion para saber que se intentara obtner la hora
  //--------------------------------------------------------------------------Configuracion de NTP
  SSDEBUG.print(F("servidor de NTP:"));
  SSDEBUG.println(ntpServerName);
  //--------------------------------------------------------------------------Configuracion de UDP
  SSDEBUG.println("Starting UDP");
  udp.begin(localPort);
  SSDEBUG.print("Local port: ");
  SSDEBUG.println(udp.localPort());
  while (NTP == false) {
    setSyncProvider(getNtpTime);                                                          //iniciamos la mensajeria de UDP para consultar la hora en el servicio de NTP remoto (el servidor se configura en 
    delay(UInterval);
  }
  NTP = false;
  //--------------------------------------------------------------------------Connectando a servicio de MQTT
  SSDEBUG.println(F("Time Sync, Connecting to mqtt sevrer"));
  mqttConnect();                                                            //Conectamos al servicio de Mqtt con las credenciales provistas en el archivo "settings.h"
  SSDEBUG.println(F("Mqtt Connection Done!, sending Device Data"));
  //--------------------------------------------------------------------------Enviando datos de primera conexion
  initManagedDevice();                                                      //inciamos la administracion remota desde Bluemix
  SSDEBUG.println(F("Finalizing Setup"));                                    //enviamos un mensaje de depuracion
  Blanco.COff();
  fsm_state = STATE_IDLE; //inciar el estado del la maquina de stado finito
  //*****************************************************************************************************Iniciar la Lectora en modo de TEST******************************
   Serial.write(0x02);
   //****************************************************************************************************Yield para que el ESP haga lo que tiene en cola de hacer********
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
  SSDEBUG.println(F("publishing device data to manageTopic:"));
  SSDEBUG.println(MqttDevicedata);
  sent++;
  if (client.publish(manageTopic, MqttDevicedata)) {
     SSDEBUG.println(F("enviado data de dispositivo:OK"));
     published ++;
     failed = 0; 
  }else {
    SSDEBUG.print(F("enviado data de dispositivo:FAILED"));
    failed ++;
  }
}
//------------------------------------------------------------------------------------Leer la tarjeta que se presenta
void readTag() {
  if( Serial.available()){
    Serial.write(0x02);
    char T = Serial.read();
    if ((T != '\n') && (RECORD)) {
      CARD_ID += T;
    }
    if ((T == '\n') && (RECORD)) {
      RECORD = LOW;
      if(LAST_ID!=CARD_ID){
        SSDEBUG.println(CARD_ID);
      }
      Serial.write(0x7f);
      delay(50);
      Serial.write(0x0c);
      delay(50);
      Serial.write(0xf7);
      delay(100);
      inputString = CARD_ID;
      Azul.Flash();
      LAST_ID = CARD_ID;
      CARD_ID = "";
      fsm_state = STATE_TRANSMIT_CARD_DATA;
    }

    Analisis += T;
    int ID = Analisis.lastIndexOf("Series Number:");
    if(ID!=-1){
      Analisis = "";
      RECORD = HIGH;
    }
  }
  
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
    SSDEBUG.print(WiFi.SSID());
    SSDEBUG.print(" ");
    SSDEBUG.println(WiFi.RSSI());
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
        //SSDEBUG.print(F("+0:"));
        //SSDEBUG.println(hour());
        ISO8601 +="0";
        ISO8601 += hour();
      }else{
        //SSDEBUG.print(F("hora:"));
        //SSDEBUG.println(hour());
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
    SSDEBUG.println(F("publishing Tag data to publishTopic:")); 
    SSDEBUG.println(MqttTagdata);
    sent ++;
    if (client.publish(publishTopic, MqttTagdata)){
      SSDEBUG.println(F("enviado data de RFID: OK"));
      Verde.Flash();
      buzzer();
      published ++;
      inputString = "";
      failed = 0; 
      }else {
        SSDEBUG.println(F("enviado data de RFID: FAILED"));
        Rojo.Flash();
        failed ++;
        OldTagRead = "1";
        inputString = "";
      }
  }else{
    SSDEBUG.println("Este es una lectura consecutiva");
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

     if ( millis() - Check_connection_mqtt > 5 * UInterval){
       Check_connection_mqtt = millis();
       //verificar que el cliente de Conexion al servicio se encuentre conectado
       if (!client.connected()) {
         MQTTreconnect();
       }
       client.loop();
     }
     
    break;
    //**************************************************************************************************STATE_TRANSMIT_CARD_DATA*****************************************
    case STATE_TRANSMIT_CARD_DATA:
    //Build the Json
    //check connection
    //Send the card data
    if (!client.connected()) {
        MQTTreconnect();
    }

    SSDEBUG.println(F("CARD DATA SENT"));
    CheckTime();
    publishRF_ID_Lectura(NodeID,ISO8601,inputString);
    fsm_state = STATE_IDLE; 
    break;
    //**************************************************************************************************STATE_UPDATE*****************************************************
    case STATE_UPDATE:
    SSDEBUG.println(F("STATE_UPDATE"));
    updateDeviceInfo();
    fsm_state = STATE_TRANSMIT_DEVICE_UPDATE;
    break;
    //**************************************************************************************************STATE_TRANSMIT_DEVICE_UPDATE*************************************
    case STATE_TRANSMIT_DEVICE_UPDATE:
    SSDEBUG.println(F("STATE_TRANSMIT_DEVICE_UPDATE"));
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
    SSDEBUG.println(F("STATE_TRANSMIT_ALARM_UPDATE"));
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
    SSDEBUG.println(F("Starting UDP"));
    udp.begin(localPort);
    SSDEBUG.print(("Local port: "));
    SSDEBUG.println(udp.localPort());
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