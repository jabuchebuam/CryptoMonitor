/*
      AUTOR:     Jonathan
      Projeto:    Monitor criptomoedas
      DATA:      20/10/22
*/

#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <Arduino_JSON.h>

// DEFINIÇÕES
#define DELAY_10_segundos 10000
#define DELAY_1_segundo   1000

// DECLARAÇÃO DE VARIÁVEIS
// WiFi
const char* ssid = "PeopleTelecom_Eduarda";
const char* password = "91289777";
//const char* ssid = "iPhone de Jonathan";
//const char* password = "11223311";
const char* host = "maker.ifttt.com";

// NTP
WiFiUDP Udp;
time_t prevDisplay = 0; // when the digital clock was displayed
time_t getNtpTime();
static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = -3;     // Horário de Brasilia
unsigned int localPort = 8888;  // local port to listen for UDP packets
int dia, mes, ano;
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);
unsigned long Delay_update_date = 0;

// API
const char* serverName_BTC = "https://www.mercadobitcoin.net/api/BTC/ticker/";
const char* serverName_BTC_daily = "https://www.mercadobitcoin.net/api/BTC/day-summary/2022/";
const char* serverName_ETH = "https://www.mercadobitcoin.net/api/ETH/ticker/";
const char* serverName_ETH_daily = "https://www.mercadobitcoin.net/api/ETH/day-summary/2022/";
const char* serverName_LTC = "https://www.mercadobitcoin.net/api/LTC/ticker/";
const char* serverName_LTC_daily = "https://www.mercadobitcoin.net/api/LTC/day-summary/2022/";
const char* serverName_BCH = "https://www.mercadobitcoin.net/api/BCH/ticker/";
const char* serverName_BCH_daily = "https://www.mercadobitcoin.net/api/BCH/day-summary/2022/";
const char* serverName_SOL = "https://www.mercadobitcoin.net/api/SOL/ticker/";
const char* serverName_SOL_daily = "https://www.mercadobitcoin.net/api/SOL/day-summary/2022/";
const char* serverName_ADA = "https://www.mercadobitcoin.net/api/ADA/ticker/";
const char* serverName_ADA_daily = "https://www.mercadobitcoin.net/api/ADA/day-summary/2022/";
const char* serverName_LINK = "https://www.mercadobitcoin.net/api/LINK/ticker/";
const char* serverName_LINK_daily = "https://www.mercadobitcoin.net/api/LINK/day-summary/2022/";
const char* serverName_GNO = "https://www.mercadobitcoin.net/api/GNO/ticker/";
const char* serverName_GNO_daily = "https://www.mercadobitcoin.net/api/GNO/day-summary/2022/";


//const uint8_t fingerprint[20] = {0x52, 0x21, 0xd4, 0x03, 0x76, 0x71, 0x62, 0x45, 0x46, 0x6b, 0x5d, 0xb5, 0x68, 0x27, 0xdd, 0x39, 0x6d, 0xe4, 0x93, 0xa6};
const uint8_t fingerprint[20] = {0x10, 0x8A, 0x07, 0xF5, 0x7C, 0x53, 0x57, 0xF4, 0xCE, 0x48, 0xAD, 0xA0, 0x05, 0x51, 0xFB, 0xAB, 0xDF, 0x31, 0x78, 0x13};
unsigned long Delay_update_cryptos = 0;

void UpdateCryptoCurrency(const char* Crypto, const char* serverName, const char* serverName_daily, int qtd_casas_dec);

void setup() 
{
  Serial.begin(9600);
  
  WiFi.begin(ssid, password);
  Serial.println("Conectando");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Conectado na rede WiFi com o endereco IP: ");
  Serial.println(WiFi.localIP());
  Udp.begin(localPort);
  Serial.println(Udp.localPort());
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  delay(500);

  while (dia==0)
  {
    /* Atualiza os valores de data, hora */
    if((millis() - Delay_update_date) > DELAY_1_segundo)
    {
      /* Atualização da data e hora */
      if (timeStatus() != timeNotSet) 
      {
        if (now() != prevDisplay) { //update the display only if time has changed
          prevDisplay = now();
          digitalClockDisplay();
        }
      }
      // Marca o momento da ultima atualização para esperar a próxima leitura
      Delay_update_date = millis();
    }
  }
}

void loop()
{
  String resposta, myString, last, high, low;
  char servername[30];
  int i, day_temp, mes_temp;

  /* Atualização dos valores das cryptomoedas */
  if((millis() - Delay_update_cryptos) > DELAY_10_segundos)
  {
    //Verifica o status da conexao Wifi
    if(WiFi.status() == WL_CONNECTED) 
    {
      UpdateCryptoCurrency("BTC ",serverName_BTC, serverName_BTC_daily, 5);
      delay(100);
      UpdateCryptoCurrency("ETH ",serverName_ETH, serverName_ETH_daily, 4);
      delay(100);
      UpdateCryptoCurrency("LTC ",serverName_LTC, serverName_LTC_daily, 3);
      delay(100);
    //UpdateCryptoCurrency("BCH ",serverName_BCH, serverName_BCH_daily, 4);
     //delay(100);
      //UpdateCryptoCurrency("SOL ",serverName_SOL, serverName_SOL_daily, 4);
      //delay(100);
      //UpdateCryptoCurrency("ADA ",serverName_ADA, serverName_ADA_daily, 4);
      //delay(100);
      //UpdateCryptoCurrency("LINK",serverName_LINK, serverName_LINK_daily, 4);
      //delay(100);
      //UpdateCryptoCurrency("GNO ",serverName_GNO, serverName_GNO_daily, 4);
      //delay(100);
      
    }  
    // Marca o momento da ultima atualização para esperar a próxima leitura
    Delay_update_cryptos = millis();
  }

  /* Envio do e-mail de alerta */
  WiFiClient client;
  const int httpPort = 80;
  if(!client.connect(host, httpPort))
  {
    Serial.println("Falha");
    return;
  }
  if(Serial.available())
  {
    char a = Serial.read();
    if(a=='a')
    {
      delay(500);
      String url = "/trigger/crypto_alarm/json/with/key/IF6FXkORhQFnR43Bnc6wuHj7WZ8ZqHtqT15g9qoZmI";
      client.print(String("GET ") + url + " HTTP/1.1\r\n"+"Host: " + host + "\r\n" + "Connection: close \r\n\r\n");
      Serial.println("E-mail enviado");
      delay(500);
    }
  }

  /* Atualiza os valores de data, hora */
  if((millis() - Delay_update_date) > DELAY_10_segundos)
  {

  }
}

String httpsGETRequest(const char* serverName)
{
  HTTPClient https;

//  //Instância do Cliente WiFi
  WiFiClientSecure wifi;
  wifi.setFingerprint(fingerprint); // Verficiar o certificado no site fonte

  https.begin(wifi, serverName);

  int httpsResponseCode = https.GET();
  String payload = "{}";

  if (httpsResponseCode > 0) 
  {
    payload = https.getString();
  }

  https.end();
  return payload;
}

void UpdateCryptoCurrency(const char* Crypto, const char* serverName, const char* serverName_daily, int qtd_casas_dec)
{
  String resposta, myString, last, high, low;
  char servername[30];
  int i, day_temp, mes_temp, avg[11];
    
  // Faz um GET e retornar a String do JSON recebido
  resposta = httpsGETRequest(serverName);
  // Cria um JSON a partir da resposta
  JSONVar myObject = JSON.parse(resposta);
  //Extrai o valor da ultima cotacao do Json e armazena na variavel valor
  last = ((const char*) (myObject["ticker"]["last"]));
  last.remove(qtd_casas_dec);
  high = ((const char*) (myObject["ticker"]["high"]));
  high.remove(qtd_casas_dec);
  low = ((const char*) (myObject["ticker"]["low"]));
  low.remove(qtd_casas_dec);
  Serial.println(Crypto+last+"/"+high+"/"+low);
  delay(50);
  // Faz um GET e retornar a String do JSON recebido
  for(i=1;i<11;i++)
  {
     day_temp = dia-i;
     if(day_temp<1)
     {
       day_temp = day_temp+30;
       mes_temp = mes-1;
     }
     else
     {
       mes_temp = mes;
     }
     String d = String(day_temp);
     String m = String(mes_temp);
     String ic = String(i-1);
     myString = serverName_daily+m+"/"+d;
     const char *c = myString.c_str();
     resposta = httpsGETRequest(c);
     // Cria um JSON a partir da resposta
     myObject = JSON.parse(resposta);
     //Extrai o valor da ultima cotacao do Json e armazena na variavel valor
     avg[i] = int(myObject["avg_price"]);
     myString = String(avg[i]);
     Serial.println(ic+Crypto+myString);
     delay(50);
  }
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  dia = day();
  Serial.print(dia);
  Serial.print("/");
  mes = month();
  Serial.print(mes);
  Serial.print("/");
  ano = year();
  Serial.print(ano);
  Serial.println();
}


void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
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
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  //Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.beginPacket("pool.ntp.br", 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
