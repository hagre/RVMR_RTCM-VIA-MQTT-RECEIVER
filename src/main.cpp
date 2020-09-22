/*
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file!

RVMR (RTCM VIA MQTT RECEIVER) is using the MQTT protocol (as a secure and opensource alternative to NTRIP) to get RTK correction data for my rover GPS units.

Basic selection via #define in the top of this code

by hagre 
2020 
*/
#define VERSION 0
#define SUB_VERSION 2

//Basic Settings
#define USE_MY_SECRETS //self explaining (more or less just for me to use)

#define DEBUG_UART_ENABLED //enable or disable with // in front of #define     //self explaining (more or less just for me to use)
//Features
#define MQTT_VIA_SECURE_WIFI_NODE
//or
//#define MQTT_VIA_NOT_SECURE_WIFI_NODE

#define MQTT_BROKER_VIA_HOSTNAME
//or
//#define MQTT_BROKER_VIA_IP

#define RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD 

//#define MSM4 //subscribung to RTCM-MSM4 TOPICS
#define MSM7 //subscribung to RTCM-MSM7 TOPICS


//More Settings
#ifdef USE_MY_SECRETS
  #include "secrets/secrets_client.h"
#endif
//the folowing is rewritten if USE_MY_SECRETS is activated
#ifndef LAN_IP_RANGE
  #define LAN_IP_RANGE 20 //43 or 20 config as required
#endif  
#define YOUR_WIFI_HOSTNAME "RTCM_MQTT_RECEIVER"
#define IP_1_THIS_NODE 192
#define IP_2_THIS_NODE 168
#define IP_3_THIS_NODE LAN_IP_RANGE
#define IP_4_THIS_NODE 127

#ifndef YOUR_WIFI_SSID
  #define YOUR_WIFI_SSID "WLAN"
#endif
#ifndef YOUR_WIFI_PASSWORD
  #define YOUR_WIFI_PASSWORD "Password"
#endif
#ifndef YOUR_WIFI_HOSTNAME
  #define YOUR_WIFI_HOSTNAME "RTCM_MQTT_RELAY"
#endif 

#ifdef RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD
  #ifndef MQTT_BROKER_USERNAME
    #define MQTT_BROKER_USERNAME "MQTTuser" //config as required 
  #endif
  #ifndef MQTT_BROKER_PASSWORD
    #define MQTT_BROKER_PASSWORD "MQTTpassword" //config as required 
  #endif
#endif

#ifndef MQTT_CLIENT_ID_FOR_BROKER
  #define MQTT_CLIENT_ID_FOR_BROKER "TestClient" //config as required 
#endif

#define MQTT_VERSION 4 //4 = MQTT_VERSION_3_1 // 3 = MQTT_VERSION_3_1_1 

#ifdef MQTT_BROKER_VIA_IP
  #ifndef MQTT_BROKER_1_IP 
    #define MQTT_BROKER_1_IP 10 //config as required //for unsecure LAN connection
  #endif
  #ifndef MQTT_BROKER_2_IP 
    #define MQTT_BROKER_2_IP 0 //config as required //for unsecure LAN connection
  #endif
  #ifndef MQTT_BROKER_3_IP 
    #define MQTT_BROKER_3_IP 0 //config as required //for unsecure LAN connection
  #endif
  #ifndef MQTT_BROKER_4_IP 
    #define MQTT_BROKER_4_IP 1 //config as required //for unsecure LAN connection
  #endif
#endif  
#ifdef MQTT_BROKER_VIA_HOSTNAME
  #ifndef HOSTNAME_OF_MQTT_BROKER
    #define HOSTNAME_OF_MQTT_BROKER "test.mosquitto.com" //config as required  //for secure INTERNET connection
  #endif
#endif

#ifdef MQTT_VIA_SECURE_WIFI_NODE
  #define MQTT_CONNECTION_PORT 8883 // 8883 TLS //config as required
#endif  
#ifdef MQTT_VIA_NOT_SECURE_WIFI_NODE
  #define MQTT_CONNECTION_PORT 8882 //8882 = not secure on LAN only //config as required
#endif

#define MQTT_SET_KEEPALIVE 30 //15 = 15sec
#define MQTT_SET_SOCKET_TIMEOUT 40 //20sec

#define MQTT_WAIT_FOR_SERVER_CONNECTION 10000 //ms time - default is 30000 ms, change only if required
#define MQTT_WAIT_FOR_SERVER_RECONNECTION 5000 //ms time - default is 15000 ms, change only if required

#define MQTT_RTCM_TOPIC_INIT "RTK/Base/" 

#ifndef MQTT_RTCM_BASE_NAME
  #define MQTT_RTCM_BASE_NAME "TEST01" //config as required 
#endif

#define MQTT_MAX_PACKET_SIZE 1024 //1024 config as required
#define RTCM_MSG_BUFFER_SIZE 1024 //1024 config as required
#define RTCM_RELAY_BUFFER_SIZE 10 //10 config as required // •  RTCM 1005 Stationary RTK reference station ARP•  RTCM 1074 and 1077 GPS •  RTCM 1084 and 1087 GLONASS •  RTCM 1094 ans 1097 Galileo •  RTCM 1124 and 1127 BeiDou•  RTCM 1230 GLONASS code-phase biases

#define NTRIP_TCP_PORT 2101 //2101 config as required


#ifdef DEBUG_UART_ENABLED 
  #define DEBUG_UART_HARDWARE_PORT 0 //0 config as required // USB == 0
  #define DEBUG_UART_BOUD 115200 //115200 config as required 
#endif
// -------------------------------- End of Config -----------------------------------------------

// -------------------------------- Begin of Program --------------------------------------------
//include libraries
#include <Arduino.h>
#include "verysimpletimer.h"

#define UART0RX 3 //ESP32 GPIOs
#define UART0TX 1
#define UART1RX 16 //26 //ESP32 look like UNO - modified for Ardusimpleconnection 
#define UART1TX 17 //12 problem SPI internal //ESP32 look like UNO - modified for Ardusimpleconnection 
#define UART2RX 13 //16 //ESP32 look like UNO - modified for Ardusimpleconnection 
#define UART2TX 26 //17 //ESP32 look like UNO - modified for Ardusimpleconnection 

#ifdef DEBUG_UART_ENABLED
  #include <HardwareSerial.h>
  uint32_t serialDebugPortBoud = DEBUG_UART_BOUD;
  #if DEBUG_UART_HARDWARE_PORT == 0  
    #define DEBUG_UART_RX UART0RX
    #define DEBUG_UART_TX UART0TX
  #elif DEBUG_UART_HARDWARE_PORT == 1  
    #define DEBUG_UART_RX UART1RX
    #define DEBUG_UART_TX UART1TX
  #elif DEBUG_UART_HARDWARE_PORT == 2 
    #define DEBUG_UART_RX UART2RX
    #define DEBUG_UART_TX UART2TX 
  #endif
  HardwareSerial SerialDebug(DEBUG_UART_HARDWARE_PORT);
#endif

typedef struct {
  bool readyToSend = false;
  byte RTCMMsg [RTCM_MSG_BUFFER_SIZE]; 
  uint16_t msgLength = 0;
} RTCMRelayBuffer_t;

#define CIRCULAR_BUFFER_INT_SAFE
#include "CircularBuffer.h"
CircularBuffer<RTCMRelayBuffer_t, RTCM_RELAY_BUFFER_SIZE> relayBuffer;

int8_t LANStatus = -5; //Connected to Network //WIFI or ETHERNET //-5 = init, -2 = just disconnected, -1 = wait to reconnect, 0 = disconnected, 1 = connecting, 2 = just connected,  3 = still connected

#ifdef MQTT_BROKER_VIA_HOSTNAME
  const char* mQTTBrokerHostName = HOSTNAME_OF_MQTT_BROKER;
#endif

#ifdef MQTT_BROKER_VIA_IP
  const IPAddress mQTTBrokerIP (MQTT_BROKER_1_IP, MQTT_BROKER_2_IP, MQTT_BROKER_3_IP, MQTT_BROKER_4_IP); 
#endif  

#ifdef RTCM_VIA_MQTT_TRANSMITTING_WITH_PASSWORD
  const char * mqttUsername = MQTT_BROKER_USERNAME;
  const char * mqttPassword = MQTT_BROKER_PASSWORD;
#endif

const IPAddress Node_IP (IP_1_THIS_NODE, IP_2_THIS_NODE, IP_3_THIS_NODE, IP_4_THIS_NODE);

#include <WiFi.h>
#include <SyncWifiConnectionESP32.h>
SyncWifiConnectionESP32 SyncWifiConnection;
const char* ssid     = YOUR_WIFI_SSID; 
const char* password = YOUR_WIFI_PASSWORD;

//#include <WiFiClient.h>
WiFiServer tCPServer(NTRIP_TCP_PORT);
WiFiClient incomingTCPClient;

#ifdef MQTT_VIA_SECURE_WIFI_NODE
  #include <WiFiClientSecure.h>
  WiFiClientSecure mqttLANClient;
  #ifndef MQTT_BROKER_CA_CERT 
    //This is an unvalide root_CA_cert just to show you the format
    #include "root_ca.h"
  #endif
#endif  

#ifdef MQTT_VIA_NOT_SECURE_WIFI_NODE
  #include <WiFiClient.h>
  WiFiClient mqttLANClient;
#endif

#include <SyncMQTTConnectionESP32.h>
SyncMQTTConnectionESP32 syncMQTTConnection; 
const char* mqttPubSubClientId = MQTT_CLIENT_ID_FOR_BROKER;

int8_t MQTTStatus = -5; //Connected to MQTT broker // -5 init, -3 LAN just disconnected, -2 just disconnected, -1 wait to reconnect, 0 disconnected, 1 connecting, 2 just connected, 3 subscribing, 4 subscribed and connected   


void receivedMQTTCallback(char* topic, byte* payload, unsigned int length) {
 //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++check function+++++++++++++++++++++++++++++++++++++++++++
  #ifdef DEBUG_UART_ENABLED 
    SerialDebug.println (" Callback ");
  #endif

  RTCMRelayBuffer_t bridgeBuffer;

  for (int x = 0; x < length; x++) {
    bridgeBuffer.RTCMMsg[x] = payload[x];
  }
  bridgeBuffer.msgLength = length;
  bridgeBuffer.readyToSend = true;

  relayBuffer.push (bridgeBuffer);

  
  #ifdef DEBUG_UART_ENABLED 
    SerialDebug.print("Message received: ");
    SerialDebug.println(topic);
    SerialDebug.print("payload: ");
    for (int i = 0; i < length; i++) {
      SerialDebug.print((char)payload[i]);
      SerialDebug.print(" ");
    }
    SerialDebug.print(" // ");
    for (int i = 0; i < length; i++) {
      SerialDebug.print(payload[i]);
      SerialDebug.print(" ");
    }
    SerialDebug.println();
  #endif
}



void setup() { // -------------------------------- S E T U P --------------------------------------------

  #ifdef DEBUG_UART_ENABLED
    SerialDebug.begin (serialDebugPortBoud, SERIAL_8N1, DEBUG_UART_RX, DEBUG_UART_TX); //Debug output, usually USB Port
    SerialDebug.println ("Setup");
  #endif

  #ifdef DEBUG_UART_ENABLED
    SerialDebug.println ("Starting WIFI");
    SyncWifiConnection.setWifiDebugSerial (&SerialDebug);
  #endif
  SyncWifiConnection.init (WIFI_STA, Node_IP, YOUR_WIFI_HOSTNAME, YOUR_WIFI_SSID, YOUR_WIFI_PASSWORD); 

  #ifdef MQTT_VIA_SECURE_WIFI_NODE
    // set TLS certificate
    mqttLANClient.setCACert(root_CA_Cert);
  #endif

  #ifdef MQTT_VIA_NOT_SECURE_WIFI_NODE
    //Nothing todo
  #endif

  #ifdef DEBUG_UART_ENABLED
    SerialDebug.println ("Starting MQTT");
    syncMQTTConnection.setMQTTDebugSerial (&SerialDebug);
  #endif

  #ifdef MQTT_BROKER_VIA_HOSTNAME
    syncMQTTConnection.setMQTTConnection (mqttPubSubClientId, mqttUsername, mqttPassword, true, mqttLANClient, mQTTBrokerHostName, MQTT_CONNECTION_PORT, MQTT_MAX_PACKET_SIZE, MQTT_SET_KEEPALIVE, MQTT_SET_SOCKET_TIMEOUT);
  #endif

  #ifdef MQTT_BROKER_VIA_IP
      syncMQTTConnection.setMQTTConnection (mqttPubSubClientId, mqttUsername, mqttPassword, true, mqttLANClient, mQTTBrokerIP, MQTT_CONNECTION_PORT, MQTT_MAX_PACKET_SIZE, MQTT_SET_KEEPALIVE, MQTT_SET_SOCKET_TIMEOUT);
  #endif

  syncMQTTConnection.setMQTTCallback (receivedMQTTCallback);

  String subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
  subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
  subscriptionTopic = subscriptionTopic + "/1005"; 
  syncMQTTConnection.addSubscriptionToTable (0, subscriptionTopic.c_str(), subscriptionTopic.length ());

  subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
  subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
  subscriptionTopic = subscriptionTopic + "/1230"; 
  syncMQTTConnection.addSubscriptionToTable (1, subscriptionTopic.c_str(), subscriptionTopic.length ());

  #ifdef MSM4
    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1074"; 
    syncMQTTConnection.addSubscriptionToTable (2, subscriptionTopic.c_str(), subscriptionTopic.length ());

    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1084"; 
    syncMQTTConnection.addSubscriptionToTable (3, subscriptionTopic.c_str(), subscriptionTopic.length ());

    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1094"; 
    syncMQTTConnection.addSubscriptionToTable (4, subscriptionTopic.c_str(), subscriptionTopic.length ());

    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1124"; 
    syncMQTTConnection.addSubscriptionToTable (5, subscriptionTopic.c_str(), subscriptionTopic.length ());
  #endif

  #ifdef MSM7
    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1077"; 
    syncMQTTConnection.addSubscriptionToTable (2, subscriptionTopic.c_str(), subscriptionTopic.length ());

    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1087"; 
    syncMQTTConnection.addSubscriptionToTable (3, subscriptionTopic.c_str(), subscriptionTopic.length ());

    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1097"; 
    syncMQTTConnection.addSubscriptionToTable (4, subscriptionTopic.c_str(), subscriptionTopic.length ());

    subscriptionTopic = MQTT_RTCM_TOPIC_INIT;
    subscriptionTopic = subscriptionTopic + MQTT_RTCM_BASE_NAME;
    subscriptionTopic = subscriptionTopic + "/1127"; 
    syncMQTTConnection.addSubscriptionToTable (5, subscriptionTopic.c_str(), subscriptionTopic.length ());
  #endif
}

void loop() { // -------------------------------- L O O P --------------------------------------------

  #ifdef DEBUG_UART_ENABLED
    //SerialDebug.println ("LOOP");
  #endif


  //WIFI CONNECTION MANAGER
  LANStatus = SyncWifiConnection.loop(millis());

  if (LANStatus == 2){
    tCPServer.begin();
    tCPServer.setNoDelay(true);
    #ifdef DEBUG_UART_ENABLED
      SerialDebug.println ("Starting TCPServer again ");
    #endif
  }
  else if (LANStatus == 3){
    if (tCPServer.hasClient ()){ // LAN is connected, client found        
      #ifdef DEBUG_UART_ENABLED 
        SerialDebug.println ("Server has new client requesting ");
      #endif
      if (incomingTCPClient.connected()){ // LAN is connected, client found, but still connected to old client
        #ifdef DEBUG_UART_ENABLED 
        SerialDebug.println ("Old Client disconnecting");
        #endif
        incomingTCPClient.stop(); // Stop old client to free connection
      }  
      if (!incomingTCPClient.connected()){  // LAN is connected, client found and not already connected, 
        incomingTCPClient = tCPServer.available(); //  => connect to new client
        incomingTCPClient.setNoDelay(true);
        #ifdef DEBUG_UART_ENABLED 
          SerialDebug.println ("connecting Server to NEW Client ");
        #endif
      }
    }
    else {
      #ifdef DEBUG_UART_ENABLED 
        //SerialDebug.println ("Server has NO requesting client now ");
      #endif
    }

    if (incomingTCPClient.connected()){
      #ifdef DEBUG_UART_ENABLED 
        //SerialDebug.print ("Connected - ");
      #endif

      uint16_t nrReceivedTCP = incomingTCPClient.available(); //TCP connection and incomming data avaliable, => read all avaliable date 
      #ifdef DEBUG_UART_ENABLED 
        if (nrReceivedTCP > 0){
          SerialDebug.print ("TCP data receive ... ");
          SerialDebug.println (nrReceivedTCP);
        }    
      #endif

      for (uint16_t i = 0; i < nrReceivedTCP; i++){ 
        incomingTCPClient.read(); //read TCP incomming data and count number of bytes
      }

      if (!relayBuffer.isEmpty()){
        if (relayBuffer.first().readyToSend){
          RTCMRelayBuffer_t bridgeBuffer;
          bridgeBuffer = relayBuffer.shift(); //copy and to delete from buffer
          incomingTCPClient.write(bridgeBuffer.RTCMMsg, bridgeBuffer.msgLength); //write just readed data to TCP port
        }
      }
    }
  }

  //RTCM_VIA_MQTT_NODE
  syncMQTTConnection.loop (millis(), LANStatus);

}