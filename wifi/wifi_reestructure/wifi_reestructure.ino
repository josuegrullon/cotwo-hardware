/*
This example does a test of the TCP client capability:
  * Initialization
  * Optional: SSID scan
  * AP connection
  * DHCP printout
  * DNS lookup
  * Optional: Ping
  * Connect to website and print out webpage contents
  * Disconnect
SmartConfig is still beta and kind of works but is not fully vetted!
It might not work on all networks!
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
#include "SoftwareSerial.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

//#define WLAN_SSID       "FoxFi51"           // cannot be longer than 32 characters!
#define WLAN_SSID       "FoxFi55"           // cannot be longer than 32 characters!
//#define WLAN_PASS       "12345678a"
#define WLAN_PASS       ""
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  900      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
//#define WEBSITE      "172.20.10.7"
#define WEBSITE      "192.168.43.38"
//#define WEBPAGE      "/v1/test-wifi"
String route = "/v1/measurements";
char* WEBPAGE = "/v1/measurements";

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

String id = "0004";

unsigned long valCO2;

String w_dir = "";
int sensorPin = A8;
int direccion = 0;

String w_vel;

// Create aREST instance
// aREST rest = aREST();

SoftwareSerial K_30_Serial(12,13);  //Sets up a virtual serial port
                                    //Using pin 12 for Rx and pin 13 for Tx
                                    
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response

//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

uint32_t ip;

int i=0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************************************
* VOID SETUP: Inicializamos la lectura serial, el sensor de CO2 y la conexion wifi
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(void)
{
  //pinMode(direccion,INPUT);
  w_vel = "50";
  Serial.begin(9600);
  K_30_Serial.begin(9600);    //Opens the virtual serial port with a baud of 9600
  wifInit();  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************************************
* VOID LOOP: Leemos la variable de CO2 en la funion lecCO2. Creamos un String llamado request en el
*            cual pasamos una URL incluyendo el valor de CO2 medido, para el HTTP Request
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(void)
{
  lecCO2();
  String request;
//  
//  int iR0 = analogRead(A0);
//  int iR1 = analogRead(A1);
//  int iR2 = analogRead(A2);
//  int iR3 = analogRead(A3);
//  int iR4 = analogRead(A4);
//  int iR5 = analogRead(A5);
//  int iR6 = analogRead(A6);
//  int iR7 = analogRead(A7);
//  
//  if (iR0 == 0){
//    w_dir="N";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    Serial.println(request);
//    send_request(request);
//  } 
//  if (iR1 == 0){
//    w_dir="NE";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    Serial.println(request);
//    send_request(request);
//  } 
//  if (iR2 == 0){
//    w_dir="E";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    Serial.println(request);
//    send_request(request);
//  } 
//  if (iR3 == 0){
//    w_dir="SE";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    //Serial.println(request);
//    //send_request(request);
//  } 
//  if (iR4 == 0){
//    w_dir="S";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    Serial.println(request);
//    send_request(request);
//  } 
//  if (iR5 == 0){
//    w_dir="SO";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    Serial.println(request);
//    send_request(request);
//  } 
//  if (iR6 == 0){
//    w_dir="O";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    Serial.println(request);
//    send_request(request);
//  } 
//  if (iR7 == 0) {
//    w_dir="NO";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    send_request(request);
//  }
//  else {
//    w_dir="NO";
//    request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + w_dir + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
//    Serial.println(request);
//    send_request(request);
//  }
  
  Serial.print("Lectura analoga: ");Serial.println(direccion);
  request = "GET " + route + "?id=" + id + "&ppm=" + valCO2 + "&w_dir=" + "n" + "&w_vel=" + w_vel + " HTTP/1.0\r\n";
  Serial.println(request);
  send_request(request);
  
  // connection();
  //WPTEMP = "";
  
  delay(100);
//  if (false){
//    /* You need to make sure to clean up after yourself or the CC3000 can freak out */
//    /* the next time your try to connect ... */
//    Serial.println(F("\n\nDisconnecting"));
//    cc3000.disconnect();
//  }
//  i++;
  
  
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************************************
* VOID lecCO2: En esta funcion leemos el valor de CO2 y luego lo imprimimos en el cuerpo serial
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void lecCO2(){
  sendRequest(readCO2);
  valCO2 = getValue(response);
//  Serial.print("Co2 ppm = ");
//  Serial.println(valCO2);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************************************
* VOID wifInit: Inicializamos el sensor CC3000 y probamos la conexion. Nos conectamos a la red WiFi y 
*            seleccionamos la IP por DHCP.
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void wifInit(){
  Serial.println(F("Hello, CC3000!\n")); 

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  // Optional SSID scan
  // listSSIDResults();
  
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************************************
* VOID send_request: Nos conectamos al servidor web via TCP y enviamos el String request, para luego
*                    imprimir los resultados en el puerto serial
********************************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void send_request (String request){
  
  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); 
  Serial.print(F(" -> "));
  while (ip == 0) {
        Serial.println(F("Looking for ip!"));
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
  
  // Connect to server 
  Serial.println("Connecting to server...");
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80); 

  // Send the request 
  if (www.connected()) {
      www.println(request);
      www.println(F("User-agent: CompostMonitor/1.0\r\n"));      
      www.println(F("\r\n"));
      Serial.println("Connected & data sent successfully...");
    } 
    else {
      Serial.println(F("Connection failed."));    
    }

    while (www.connected()) {
      while (www.available()) {

      // Read answer
      char c = www.read();
      Serial.print(c); 
      }
    }

  // Disconnect 
  Serial.println("Closing connection...");
  Serial.println(""); 
  www.close(); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************************************
*    @brief  Begins an SSID scan and prints out all the visible networks
*******************************************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////

void listSSIDResults(void)
{
  uint32_t index;
  uint8_t valid, rssi, sec;
  char ssidname[33]; 

  if (!cc3000.startSSIDscan(&index)) {
    Serial.println(F("SSID scan failed!"));
    return;
  }

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);
    
    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************************************
*    @brief  Tries to read the IP address and other connection details
*******************************************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////

bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendRequest(byte packet[])
{
  while(!K_30_Serial.available())  //keep sending request until we start to get a response
  {
    K_30_Serial.write(readCO2,7);
    delay(50);
  }
  
  int timeout=0;  //set a timeoute counter
  while(K_30_Serial.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;  
    if(timeout > 10)    //if it takes to long there was probably an error
      {
        while(K_30_Serial.available())  //flush whatever we have
          K_30_Serial.read();
          
          break;                        //exit and try again
      }
      delay(50);
  }
  
  for (int i=0; i < 7; i++)
  {
    response[i] = K_30_Serial.read();
  }  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long getValue(byte packet[])
{
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
    int low = packet[4];                         //low byte for value is 5th byte in the packet

  
    unsigned long val = high*256 + low;                //Combine high byte and low byte with this formula to get value
    return val* valMultiplier;
}
