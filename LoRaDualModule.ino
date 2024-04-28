/// Simon Janelle-Bombardier
/// simon.janelle-bombardier@usherbrooke.ca
/// Code de contrôle pourle module de communication LoRa
/// ***** LIRE LES COMMENTAIRE EST TRÈS IMPORTANT *****
/*
######## IMPORTANT ########
## 904 MHz (legal min): 2A
## 915 MHz (wanted): 35
## 928 MHz (legal max): 42
######## IMPORTANT ########

### Voir "E32-900T30D_UserManual_EN_v1.0+.pdf" pour plus de détails sur les configuration
### default conf    : C0 00 00 1A 17 44
### Model       Frequency   Address     Channel     Air data rate   Baud rate   Parity      Transmitting power
### E32-900T30D 868MHz      0x0000      0x06        2.4kbps         9600        8N1         1W
#
### conf drone send    : C0 01 01 1A 30 C4
### Model       Frequency   Address     Channel     Air data rate   Baud rate   Parity      Transmitting power
### E32-900T30D 910MHz      0x0101      0x30        2.4kbps         57600       8N1         1W
#
### conf drone receive : C0 01 01 1A 3A C4
### Model       Frequency   Address     Channel     Air data rate   Baud rate   Parity      Transmitting power
### E32-900T30D 920MHz      0x0101      0x3A        2.4kbps         57600       8N1         1W
#
### conf gcs send      : C0 02 02 1A 30 C4
### Model       Frequency   Address     Channel     Air data rate   Baud rate   Parity      Transmitting power
### E32-900T30D 920MHz      0x0202      0x3A        2.4kbps         57600       8N1         1W
#
### conf gcs receive   : C0 02 02 1A 3A C4
### Model       Frequency   Address     Channel     Air data rate   Baud rate   Parity      Transmitting power
### E32-900T30D 910MHz      0x0202      0x30        2.4kbps         57600       8N1         1W
*/
#include "Arduino.h"
#define FREQUENCY_900       //needed to define base frequency of lora radios
#include "LoRa_E32.h"
#include <HardwareSerial.h>

////// configuration

///activer le define pour la bonne plateforme
//#define gcs 
#define drone


#ifdef drone
  #define SENDADDH 0x00
  #define SENDADDL 0x01
  #define SENDDESH 0x00
  #define SENDDESL 0x02
  #define SENDCHAN 0x30
  #define REICADDH 0x00
  #define REICADDL 0x01
  #define REICCHAN 0x3A
#endif

#ifdef gcs
  #define SENDADDH 0x00
  #define SENDADDL 0x02
  #define SENDDESH 0x00
  #define SENDDESL 0x01
  #define SENDCHAN 0x3A
  #define REICADDH 0x00
  #define REICADDL 0x02
  #define REICCHAN 0x30
#endif

//serial:

//USB
//description : module connection
//pins reservé : PA11, PA12

//USART 1 :
//description : programming port
//ftdi:   bluepill
//TX  :   PA9
//RX  :   PA10

//USART 2 : 
//description : LoRa input
//E32 :   bluepill
//TX  :   PA3
//RX  :   PA2 
//AUX :   PA5
//M0  :   PA6 
//M1  :   PA7

//USART 3 
//description : LoRa output
//E32 :   bluepill
//TX  :   PB11
//RX  :   PB10
//AUX :   PB7
//M0  :   PB6 
//M1  :   PB5

//LoRa_E32 radio(&HWserial, aux, m0, m1, baud)

//#define SerialCOM SerialUSB
HardwareSerial SerialCOM(USART1);


HardwareSerial serialRX(USART2);
LoRa_E32 radioRX(&serialRX, PA5, PA6, PA7, UART_BPS_RATE_9600);
HardwareSerial serialTX(USART3);
LoRa_E32 radioTX(&serialTX, PB7, PB6, PB5, UART_BPS_RATE_9600);


void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
void configureSender();
void configureReceiver();
void blink(unsigned int num);


// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(PB2, OUTPUT);
  blink(1);

  SerialCOM.setTimeout(10);
  SerialCOM.begin(19200);
  SerialCOM.println("Serial connection initialised");  

  blink(2);

  configureSender();
  configureReceiver();
  
  //blink 3 times fast to show connexion initialised
  digitalWrite(PB2, HIGH);
  delay(100);
  digitalWrite(PB2, LOW); 
  delay(100); 
  digitalWrite(PB2, HIGH);
  delay(100);
  digitalWrite(PB2, LOW); 
  delay(100); 
  digitalWrite(PB2, HIGH);
  delay(100);
  digitalWrite(PB2, LOW); 
}

ResponseStatus rsSend;
ResponseContainer rsReic;
char InputBuffer[512];
int numBytes = 0;
//double timerNow = 0;
//double timerFlush = 0;
// the loop function runs over and over again forever
void loop() {
  //timerNow = millis();
  if(SerialCOM)
  {
    if (SerialCOM.available()>0){
      digitalWrite(PB2, HIGH); 

      numBytes = SerialCOM.readBytes(InputBuffer, 512); 
      rsSend = radioTX.sendFixedMessage(SENDDESH,SENDDESL,SENDCHAN, &InputBuffer, numBytes);
      
      digitalWrite(PB2, LOW);
    }

    if(radioRX.available()>0){
      digitalWrite(PB2, HIGH);

      rsReic = radioRX.receiveMessage();
      SerialCOM.print(rsReic.data);

      digitalWrite(PB2, LOW);
    }
  }
  //flush every second to not overflow the buffer if the upstream is down
  //else if (!SerialCOM && timerNow - timerFlush >= 1000)
  //{
    //timerFlush = timerNow;
    //radioRX.cleanUARTBuffer();
    //radioTX.cleanUARTBuffer();
  //}
}

void configureSender(){
	// Startup all pins and UART
	radioTX.begin();

	ResponseStructContainer c;
	c = radioTX.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	SerialCOM.println(c.status.getResponseDescription());
	SerialCOM.println(c.status.code);

	configuration.ADDL = SENDADDL;
	configuration.ADDH = SENDADDH;
	configuration.CHAN = SENDCHAN;

	configuration.OPTION.fec = FEC_1_ON;
	configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
	configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
	configuration.OPTION.transmissionPower = POWER_20;
	configuration.OPTION.wirelessWakeupTime = WAKE_UP_1250;

	configuration.SPED.airDataRate = AIR_DATA_RATE_101_192;
	configuration.SPED.uartBaudRate = UART_BPS_19200;
	configuration.SPED.uartParity = MODE_00_8N1;

	// Set configuration changed and set to not hold the configuration
	ResponseStatus rs = radioTX.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
	SerialCOM.println(rs.getResponseDescription());
	SerialCOM.println(rs.code);
	printParameters(configuration);
	c.close();
  
  //needed to set device in different baudrate
  radioTX.resetModule();
  delay(500);
  radioTX = LoRa_E32(&serialTX, PB7, PB6, PB5, UART_BPS_RATE_19200);
  radioTX.begin();
  serialTX.setTimeout(100);
}

void configureReceiver(){
// Startup all pins and UART
	radioRX.begin();

	ResponseStructContainer c;
	c = radioRX.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	SerialCOM.println(c.status.getResponseDescription());
	SerialCOM.println(c.status.code);

	configuration.ADDL = REICADDL;
	configuration.ADDH = REICADDH;
	configuration.CHAN = REICCHAN;//TODO : change for REICCHAN after test

	configuration.OPTION.fec = FEC_1_ON;
	configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
	configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
	configuration.OPTION.transmissionPower = POWER_20;
	configuration.OPTION.wirelessWakeupTime = WAKE_UP_1250;

	configuration.SPED.airDataRate = AIR_DATA_RATE_101_192;
	configuration.SPED.uartBaudRate = UART_BPS_19200;
	configuration.SPED.uartParity = MODE_00_8N1;

	// Set configuration changed and set to not hold the configuration
	ResponseStatus rs = radioRX.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
	SerialCOM.println(rs.getResponseDescription());
	SerialCOM.println(rs.code);
	printParameters(configuration);
	c.close();

  //needed to set device in different baudrate
  radioRX.resetModule();
  delay(500);
  radioRX = LoRa_E32(&serialRX, PA5, PA6, PA7, UART_BPS_RATE_19200);
  radioRX.begin();
  serialRX.setTimeout(100);
}

void printParameters(struct Configuration configuration) {
	SerialCOM.println("----------------------------------------");

	SerialCOM.print(F("HEAD : "));  SerialCOM.print(configuration.HEAD, BIN);SerialCOM.print(" ");SerialCOM.print(configuration.HEAD, DEC);SerialCOM.print(" ");SerialCOM.println(configuration.HEAD, HEX);
	SerialCOM.println(F(" "));
	SerialCOM.print(F("AddH : "));  SerialCOM.println(configuration.ADDH, BIN);
	SerialCOM.print(F("AddL : "));  SerialCOM.println(configuration.ADDL, BIN);
	SerialCOM.print(F("Chan : "));  SerialCOM.print(configuration.CHAN, DEC); SerialCOM.print(" -> "); SerialCOM.println(configuration.getChannelDescription());
	SerialCOM.println(F(" "));
	SerialCOM.print(F("SpeedParityBit     : "));  SerialCOM.print(configuration.SPED.uartParity, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.SPED.getUARTParityDescription());
	SerialCOM.print(F("SpeedUARTDatte  : "));  SerialCOM.print(configuration.SPED.uartBaudRate, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.SPED.getUARTBaudRate());
	SerialCOM.print(F("SpeedAirDataRate   : "));  SerialCOM.print(configuration.SPED.airDataRate, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.SPED.getAirDataRate());

	SerialCOM.print(F("OptionTrans        : "));  SerialCOM.print(configuration.OPTION.fixedTransmission, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.OPTION.getFixedTransmissionDescription());
	SerialCOM.print(F("OptionPullup       : "));  SerialCOM.print(configuration.OPTION.ioDriveMode, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.OPTION.getIODroveModeDescription());
	SerialCOM.print(F("OptionWakeup       : "));  SerialCOM.print(configuration.OPTION.wirelessWakeupTime, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	SerialCOM.print(F("OptionFEC          : "));  SerialCOM.print(configuration.OPTION.fec, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.OPTION.getFECDescription());
	SerialCOM.print(F("OptionPower        : "));  SerialCOM.print(configuration.OPTION.transmissionPower, BIN);SerialCOM.print(" -> "); SerialCOM.println(configuration.OPTION.getTransmissionPowerDescription());

	SerialCOM.println("----------------------------------------");

}

//blocking blink
void blink(unsigned int num){
  for (int i=0; i<num;i++){
    digitalWrite(PB2, HIGH);
    delay(500);
    digitalWrite(PB2, LOW);
    delay(200);
  }
}


