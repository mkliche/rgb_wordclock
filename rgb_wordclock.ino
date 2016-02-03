/* 

(c) 2014 - Markus Backes - https://backes-markus.de/blog/

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Dieses Programm ist Freie Software: Sie k�nnen es unter den Bedingungen
der GNU General Public License, wie von der Free Software Foundation,
Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
ver�ffentlichten Version, weiterverbreiten und/oder modifizieren.

Dieses Programm wird in der Hoffnung, dass es n�tzlich sein wird, aber
OHNE JEDE GEW�HRLEISTUNG, bereitgestellt; sogar ohne die implizite
Gew�hrleistung der MARKTF�HIGKEIT oder EIGNUNG F�R EINEN BESTIMMTEN ZWECK.
Siehe die GNU General Public License f�r weitere Details.

Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. 
*/

//Library includes
#include <DS1307RTC.h>
#include <FastLED.h>
#include <Wire.h>
#include <Time.h>
#include <SoftwareSerial.h>
//#include <DCF77.h>
//#include <IRremote.h>
#include "alt_layout3.h" //#include "alt_layout1.h"

// IR defines
#define ONOFF 0xFF02FD
#define AUTO 0xFFF00F
#define BLUE_DOWN 0xFF48B7
#define BLUE_UP 0xFF6897
#define BRIGHTER 0xFF3AC5
#define DIM 0xFFBA45
#define DIY1 0xFF30CF
#define DIY2 0xFFB04F
#define DIY3 0xFF708F
#define DIY4 0xFF10EF
#define DIY5 0xFF906F
#define DIY6 0xFF50AF
#define FLASH 0xFFD02F
#define QUICK 0xFFE817
#define SLOW 0xFFC837

//LED defines
#define NUM_LEDS 114

//PIN defines
#define STRIP_DATA_PIN 12
//#define IR_RECV_PIN 11
#define ARDUINO_LED 13 //Default Arduino LED
//#define DCF_PIN 2	         // Connection pin to DCF 77 device
#define DCF_INTERRUPT 0		 // Interrupt number associated with pin
#define LDR_PIN 0

//WIFI defines
#define WIFI_SSID         "KA"
#define WIFI_PASS         "2905Lena2410"
#define WIFI_RX_PIN       10
#define WIFI_TX_PIN       11

// For NTP sync
unsigned long time = 0;
const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];

//dcf variables
/*
time_t time;
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT);
bool timeInSync = false;
*/

//Wifi variables
SoftwareSerial wifiSerial(WIFI_RX_PIN, WIFI_TX_PIN); // RX, TX

uint8_t strip[NUM_LEDS];
uint8_t stackptr = 0;
tmElements_t tm;

CRGB leds[NUM_LEDS];
/*
IRrecv irrecv = IRrecv(IR_RECV_PIN);
decode_results irDecodeResults;
*/
uint8_t selectedLanguageMode = 0;
const uint8_t RHEIN_RUHR_MODE = 0; //Define?
const uint8_t WESSI_MODE = 1;

boolean autoBrightnessEnabled = true;
boolean listening = false;

int displayMode = DIY1;

CRGB defaultColor = CRGB::Red;
uint8_t colorIndex = 0;

int currentHours = 0;
int currentMinutes = 0;

//multitasking helper

const long oneSecondDelay = 1000;
const long halfSecondDelay = 500;

long waitUntilRtc = 0;
long waitUntilParty = 0;
long waitUntilOff = 0;
long waitUntilFastTest = 0;
long waitUntilHeart = 0;
long waitUntilDCF = 0;
long waitUntilLDR = 0;

//forward declaration
void fastTest();
void clockLogic();
//void doIRLogic();
//void doLDRLogic();
void makeParty();
void off();
void showHeart();
void pushToStrip(int ledId);
void resetAndBlack();
void resetStrip();
void displayStripRandomColor();
void displayStrip();
void displayStrip(CRGB colorCode);
void timeToStrip(uint8_t hours,uint8_t minutes);
//void doDCFLogic();

#define DEBUG

#ifdef DEBUG
	#define DEBUG_PRINT(str)  Serial.println (str)
#else
	#define DEBUG_PRINT(str)
#endif

void setup() {
	
  delay(2000);
  
	#ifdef DEBUG
		Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for Leonardo only
    }   
	#endif

  setupWifi();  
	
	pinMode(ARDUINO_LED, OUTPUT);
	
	//setup leds incl. fastled
	for(int i = 0; i<NUM_LEDS; i++) {
		strip[i] = 0;
	}
	FastLED.addLeds<WS2812B, STRIP_DATA_PIN, GRB>(leds, NUM_LEDS);
	resetAndBlack();
	//displayStrip();
	
	//setup dcf
	//DCF.Start();
	setSyncInterval(3600); //every hour
	setSyncProvider(getNtpTime);
	DEBUG_PRINT("Waiting for NTP time ... ");
	DEBUG_PRINT("It will take at least 2 minutes until a first update can be processed.");

	while(timeStatus()== timeNotSet) {
		// wait until the time is set by the sync provider
		DEBUG_PRINT(".");
		delay(2000);
	}
	
	//setup ir
	//irrecv.enableIRIn();

  startHttpServer();
  //startListening();  
}

void setupWifi() {
  // set the data rate for the SoftwareSerial port
  wifiSerial.begin(9600);
  delay (1000);
  connectWiFi();  // Start the WiFi module
}

void loop() {
	//doIRLogic();
  doHttpLogic();
	doLDRLogic();
	switch(displayMode) {
		case ONOFF:
			off();
			break;
		case DIY1:
			clockLogic();
			break;
		case DIY2:
			makeParty();
			break;
		case DIY3:
			showHeart();
			break;
		case DIY4:
			fastTest();
			break;
		default:
			clockLogic();
			break;
	}
}

int start_save=0;
int ndx=0;

time_t getNtpTime()
{
  stopListening();
  DEBUG_PRINT("getNtpTime");
  unsigned long epoch = 0;
  connectNTP();
  setPacket();
  time = millis();
  while ((time + 10000) >= millis()){
    if (wifiSerial.available()) {
        
      unsigned char c=wifiSerial.read();
  
      if( start_save && ndx < 48 ) 
        packetBuffer[ndx++]=c;
      else if( c == ':' )  // get the next 48 bytes
        start_save=1;
  
      if(  start_save && ndx == 48 ) {  // convert
        epoch = dump_ntp_packet();
        ndx=start_save=0;
        startListening();
        return epoch;
      }
      // Serial.write(c);
    }
  }
  DEBUG_PRINT("NTP failed - no wifi"); 
  startListening();
  return epoch;
}

/*
unsigned long getDCFTime() {
	time_t DCFtime = DCF.getTime();
	// Indicator that a time check is done
	if (DCFtime!=0) {
		DEBUG_PRINT("sync");
	}
	return DCFtime;
}
*/
void doLDRLogic() {
	if(millis() >= waitUntilLDR && autoBrightnessEnabled) {
		DEBUG_PRINT("doing LDR logic");
		waitUntilLDR = millis();
		int ldrVal = map(analogRead(LDR_PIN), 0, 1023, 0, 150);
		FastLED.setBrightness(105+ldrVal);
		FastLED.show();
		DEBUG_PRINT(ldrVal);
		waitUntilLDR += oneSecondDelay;
	}
}
/*
void doIRLogic() {
	uint8_t brightness = 0;
	if (irrecv.decode(&irDecodeResults)) {
		DEBUG_PRINT("Received IR code");
		delay(50);
		switch(irDecodeResults.value) {
			case ONOFF:
				displayMode = ONOFF;
				break;
			case AUTO:
				autoBrightnessEnabled = !autoBrightnessEnabled;
				break;
			case BLUE_DOWN:
				//TODO
				break;
			case BLUE_UP:
				//TODO
				break;
			case BRIGHTER:
				autoBrightnessEnabled = false;
				brightness = FastLED.getBrightness();
				if(brightness <= 255 - 50) {
					FastLED.setBrightness(brightness + 50);
				} else {
					FastLED.setBrightness(255);
				}
				FastLED.show();
				break;
			case DIM:
				autoBrightnessEnabled = false;
				brightness = FastLED.getBrightness();
				if(brightness >= 50) {
					FastLED.setBrightness(brightness - 50);
				} else {
					FastLED.setBrightness(0);
				}
				FastLED.show();
				break;
			case DIY1:
				displayMode = DIY1;
				autoBrightnessEnabled = true;
				//to force display update
				currentMinutes = -1;
				currentHours = -1;
				break;
			case DIY2:
				displayMode = DIY2;
				break;
			case DIY3:
				displayMode = DIY3;
				break;
			case DIY4:
				displayMode = DIY4;
				break;
			case DIY5:
				displayMode = DIY5;
				break;
			case DIY6:
				displayMode = DIY6;
				break;
			case FLASH:
				displayMode = FLASH;
				break;
			case QUICK:
				defaultColor = nextColor();
				displayStrip();
				break;
			case SLOW:
				defaultColor = prevColor();
				displayStrip();
				break;
			default:
				DEBUG_PRINT("IR DEFAULT");
				break;
		}
		irrecv.resume();
	}
}
*/
///////////////////////
//DISPLAY MODES
///////////////////////
void clockLogic() {
	if(millis() >= waitUntilRtc) {
		DEBUG_PRINT("doing clock logic");
		waitUntilRtc = millis();
		if(currentMinutes != minute() || currentHours != hour()) {
			currentMinutes = minute();
			currentHours = hour();
			resetAndBlack();
			timeToStrip(currentHours, currentMinutes);
			displayStrip(defaultColor);
		}
		waitUntilRtc += oneSecondDelay;
	}
}

void off() {
	if(millis() >= waitUntilOff) {
		DEBUG_PRINT("switching off");
		waitUntilOff = millis();
		resetAndBlack();
		displayStrip(CRGB::Black);
		waitUntilOff += halfSecondDelay;
	}
}

void makeParty() {
	if(millis() >= waitUntilParty) {
		autoBrightnessEnabled = false;
		DEBUG_PRINT("YEAH party party");
		waitUntilParty = millis();
		resetAndBlack();
		for(int i = 0; i<NUM_LEDS;i++) {
			leds[i] = CHSV(random(0, 255), 255, 255);
		}
		FastLED.show();
		waitUntilParty += halfSecondDelay;
	}
}

void showHeart() {
	if(millis() >= waitUntilHeart) {
		autoBrightnessEnabled = false;
		DEBUG_PRINT("showing heart");
		waitUntilHeart = millis();
		resetAndBlack();
		pushToStrip(L29); pushToStrip(L30); pushToStrip(L70); pushToStrip(L89);
		pushToStrip(L11); pushToStrip(L48); pushToStrip(L68); pushToStrip(L91);
		pushToStrip(L7); pushToStrip(L52); pushToStrip(L107);
		pushToStrip(L6); pushToStrip(L106);
		pushToStrip(L5); pushToStrip(L105);
		pushToStrip(L15); pushToStrip(L95);
		pushToStrip(L23); pushToStrip(L83);
		pushToStrip(L37); pushToStrip(L77);
		pushToStrip(L41); pushToStrip(L61);
		pushToStrip(L59);
		displayStrip(CRGB::Red);
		waitUntilHeart += oneSecondDelay;
	}
}

void fastTest() {
	if(millis() >= waitUntilFastTest) {
		autoBrightnessEnabled = false;
		DEBUG_PRINT("showing heart");
		waitUntilFastTest = millis();
		if(currentMinutes >= 60) {
			currentMinutes = 0;
			currentHours++;
		}
		if(currentHours >= 24) {
			currentHours = 0;
		}
		
		//Array leeren
		resetAndBlack();
		timeToStrip(currentHours, currentMinutes);
		displayStripRandomColor();
		currentMinutes++;
		waitUntilFastTest += oneSecondDelay;
	}
}
///////////////////////

CRGB prevColor() {
	if(colorIndex > 0) {
		colorIndex--;
	}
	return getColorForIndex();
}
CRGB nextColor() {
	if(colorIndex < 9) {
		colorIndex++;
	}
	return getColorForIndex();
}

CRGB getColorForIndex() {
	switch(colorIndex) {
		case 0:
			return CRGB::White;
		case 1:
			return CRGB::Blue;
		case 2:
			return CRGB::Aqua;
		case 3:
			return CRGB::Green;
		case 4:
			return CRGB::Lime;
		case 5:
			return CRGB::Red;
		case 6:
			return CRGB::Magenta;
		case 7:
			return CRGB::Olive;
		case 8:
			return CRGB::Yellow;
		case 9:
			return CRGB::Silver;
		default:
			colorIndex = 0;
			return CRGB::White;
	}
}

void pushToStrip(int ledId) {
	strip[stackptr] = ledId;
	stackptr++;
}

void resetAndBlack() {
	resetStrip();
	for(int i = 0; i<NUM_LEDS; i++) {
		leds[i] = CRGB::Black;
	}
}

void resetStrip() {
	stackptr = 0;
	for(int i = 0; i<NUM_LEDS; i++) {
		strip[i] = 0;
	}
}

void displayStripRandomColor() {
	for(int i = 0; i<stackptr; i++) {
		leds[strip[i]] = CHSV(random(0, 255), 255, 255);
	}
	FastLED.show();
}

void displayStrip() {
	displayStrip(defaultColor);
}

void displayStrip(CRGB colorCode) {
	for(int i = 0; i<stackptr; i++) {
		leds[strip[i]] = colorCode;
	}
	FastLED.show();
}

void timeToStrip(uint8_t hours,uint8_t minutes)
{
	pushES_IST();

	//show minutes
	if(minutes >= 5 && minutes < 10) {
		pushFUENF1();
		pushNACH();
	} else if(minutes >= 10 && minutes < 15) {
		pushZEHN1();
		pushNACH();
	} else if(minutes >= 15 && minutes < 20) {
		pushVIERTEL();
		pushNACH();
	} else if(minutes >= 20 && minutes < 25) {
		if(selectedLanguageMode == RHEIN_RUHR_MODE) {
			pushZWANZIG();
			pushNACH();
		} else if(selectedLanguageMode == WESSI_MODE) {
			pushZEHN1();
			pushVOR();
			pushHALB();
		}
	} else if(minutes >= 25 && minutes < 30) {
		pushFUENF1();
		pushVOR();
		pushHALB();
	} else if(minutes >= 30 && minutes < 35) {
		pushHALB();
	} else if(minutes >= 35 && minutes < 40) {
		pushFUENF1();
		pushNACH();
		pushHALB();
	} else if(minutes >= 40 && minutes < 45) {
		if(selectedLanguageMode == RHEIN_RUHR_MODE) {
			pushZWANZIG();
			pushVOR();
		} else if(selectedLanguageMode == WESSI_MODE) {
			pushZEHN1();
			pushNACH();
			pushHALB();
		}
	} else if(minutes >= 45 && minutes < 50) {
		pushVIERTEL();
		pushVOR();
	} else if(minutes >= 50 && minutes < 55) {
		pushZEHN1();
		pushVOR();
	} else if(minutes >= 55 && minutes < 60) {
		pushFUENF1();
		pushVOR();
	}
	
	int singleMinutes = minutes % 5;
	switch(singleMinutes) {
		case 1:
			pushONE();
			break;
		case 2:
			pushONE();
			pushTWO();
			break;
		case 3:
			pushONE();
			pushTWO();
			pushTHREE();
			break;
		case 4:
			pushONE();
			pushTWO();
			pushTHREE();
			pushFOUR();
		break;
	}

	if(hours >= 12) {
		hours -= 12;
	}

	if(selectedLanguageMode == RHEIN_RUHR_MODE) {
		if(minutes >= 25) {
			hours++;
		}
	} else if(selectedLanguageMode == WESSI_MODE) {
		if(minutes >= 20) {
			hours++;
		}
	}

	if(hours == 12) {
		hours = 0;
	}

	//show hours
	switch(hours) {
		case 0:
			pushZWOELF();
			break;
		case 1:
			if(minutes > 4) {
				pushEINS(true);
			} else {
				pushEINS(false);
			}
			break;
		case 2:
			pushZWEI();
			break;
		case 3:
			pushDREI();
			break;
		case 4:
			pushVIER();
			break;
		case 5:
			pushFUENF2();
			break;
		case 6:
			pushSECHS();
			break;
		case 7:
			pushSIEBEN();
			break;
		case 8:
			pushACHT();
			break;
		case 9:
			pushNEUN();
			break;
		case 10:
			pushZEHN();
			break;
		case 11:
			pushELF();
			break;
	}
	
	//show uhr
	if(minutes < 5) {
		pushUHR();
	}
}

///////////////////////
//PUSH WORD HELPER
///////////////////////
void pushES_IST()  {
	pushToStrip(L9);
	pushToStrip(L10);
	pushToStrip(L30);
	pushToStrip(L49);
	pushToStrip(L50);
}

void pushFUENF1() {
	pushToStrip(L70);
	pushToStrip(L89);
	pushToStrip(L90);
	pushToStrip(L109);
}

void pushFUENF2() {
	pushToStrip(L74);
	pushToStrip(L85);
	pushToStrip(L94);
	pushToStrip(L105);
}

void pushNACH() {
	pushToStrip(L73);
	pushToStrip(L86);
	pushToStrip(L93);
	pushToStrip(L106);
}

void pushZEHN1() {
	pushToStrip(L8);
	pushToStrip(L11);
	pushToStrip(L28);
	pushToStrip(L31);
}

void pushVIERTEL() {
	pushToStrip(L47);
	pushToStrip(L52);
	pushToStrip(L67);
	pushToStrip(L72);
	pushToStrip(L87);
	pushToStrip(L92);
	pushToStrip(L107);
}

void pushVOR() {
	pushToStrip(L6);
	pushToStrip(L13);
	pushToStrip(L26);
}

void pushHALB() {
	pushToStrip(L5);
	pushToStrip(L14);
	pushToStrip(L25);
	pushToStrip(L34);
}

void pushONE() {
	pushToStrip(L113);
}

void pushTWO() {
	pushToStrip(L110);
}

void pushTHREE() {
	pushToStrip(L111);
}

void pushFOUR() {
	pushToStrip(L112);
}

void pushZWANZIG() {
	pushToStrip(L48);
	pushToStrip(L51);
	pushToStrip(L68);
	pushToStrip(L71);
	pushToStrip(L88);
	pushToStrip(L91);
	pushToStrip(L108);
}

void pushZWOELF() {
	pushToStrip(L61);
	pushToStrip(L78);
	pushToStrip(L81);
	pushToStrip(L98);
	pushToStrip(L101);
}

void pushEINS(bool s) {
	pushToStrip(L4);
	pushToStrip(L15);
	pushToStrip(L24);
	if(s) {
		pushToStrip(L35);
	}
}

void pushZWEI() {
	pushToStrip(L75);
	pushToStrip(L84);
	pushToStrip(L95);
	pushToStrip(L104);
}

void pushDREI() {
	pushToStrip(L3);
	pushToStrip(L16);
	pushToStrip(L23);
	pushToStrip(L36);
}

void pushVIER() {
	pushToStrip(L76);
	pushToStrip(L83);
	pushToStrip(L96);
	pushToStrip(L103);
}

void pushSECHS() {
	pushToStrip(L2);
	pushToStrip(L17);
	pushToStrip(L22);
	pushToStrip(L37);
	pushToStrip(L42);
}

void pushSIEBEN() {
	pushToStrip(L1);
	pushToStrip(L18);
	pushToStrip(L21);
	pushToStrip(L38);
	pushToStrip(L41);
	pushToStrip(L58);
}

void pushACHT() {
	pushToStrip(L77);
	pushToStrip(L82);
	pushToStrip(L97);
	pushToStrip(L102);
}

void pushNEUN() {
	pushToStrip(L39);
	pushToStrip(L40);
	pushToStrip(L59);
	pushToStrip(L60);
}

void pushZEHN() {
	pushToStrip(L0);
	pushToStrip(L19);
	pushToStrip(L20);
	pushToStrip(L39);
}

void pushELF() {
	pushToStrip(L54);
	pushToStrip(L65);
	pushToStrip(L74);
}

void pushUHR() {
	pushToStrip(L80);
	pushToStrip(L99);
	pushToStrip(L100);
}
///////////////////////

// WIFI and NTP functions down here
void connectWiFi()
{

  //Reset the module.
  espCommand("AT+RST", 2000);
  delay(20);

  //Set the wireless mode
  espCommand("AT+CWMODE=1", 500);
  delay(20);

  //disconnect  - it shouldn't be but just to make sure
  espCommand("AT+CWQAP", 1000);

  // connect to your wireless router 
  String cmd="AT+CWJAP=\"";
  cmd+=WIFI_SSID;
  cmd+="\",\"";
  cmd+=WIFI_PASS;
  cmd+="\"";
  espCommand(cmd, 15000);

  //print the ip addr
  espCommand("AT+CIFSR", 5000);

  //set the single connection mode
  espCommand("AT+CIPMUX=1", 1000);
}

String espCommand(String cmd, int timeout) {
  DEBUG_PRINT(cmd);
  wifiSerial.println(cmd);
  String response = "";
  time = millis();
  while ((time + timeout) >= millis()){
    if (wifiSerial.available() > 0) {
      char c = wifiSerial.read();
      response += c;
    }
  }
  DEBUG_PRINT(response);
  return response;
}

void connectNTP() {
  //Connect to the NTP server
  espCommand("AT+CIPSTART=0,\"UDP\",\"129.6.15.28\",123", 5000);
}

void cipsend() {
  espCommand("AT+CIPSEND=0,48", 5000);
}

void setPacket(){

  cipsend();
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
  wifiSerial.write(packetBuffer,NTP_PACKET_SIZE);
}

unsigned long dump_ntp_packet() {
  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;  
  Serial.print("Seconds since Jan 1 1900 = " );
  Serial.println(secsSince1900);               

  // now convert NTP time into everyday time:
  Serial.print("Unix time = ");
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;     
  // subtract seventy years:
  unsigned long epoch = secsSince1900 - seventyYears;  
  // print Unix time:
  Serial.println(epoch);   
                              


  // print the hour, minute and second:
  Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
  int h=(epoch  % 86400L) / 3600;
  Serial.print(h); // print the hour (86400 equals secs per day)
  Serial.print(':');  
  if ( ((epoch % 3600) / 60) < 10 ) {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    Serial.print('0');
  }
  Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
  Serial.print(':'); 
  if ( (epoch % 60) < 10 ) {
    // In the first 10 seconds of each minute, we'll want a leading '0'
    Serial.print('0');
  }
  Serial.println(epoch %60); // print the second

  return epoch;
}

void startHttpServer() {
  espCommand("AT+CIPSERVER=1,80", 1000);
}

void clearBuffer(void) {
  for (int i =0;i<BUFFER_SIZE;i++ ) {
    buffer[i]=0;
  }
}

void clearSerialBuffer(void) {
  while ( wifiSerial.available() > 0 ) {
    wifiSerial.read();
  }
}

void homepage(int ch_id) {
  String Header;

  Header =  "HTTP/1.1 200 OK\r\n";
  Header += "Content-Type: text/html\r\n";
  Header += "Connection: close\r\n";  
  //Header += "Refresh: 5\r\n";
  
  String Content;
  Content = "D";
  
  Header += "Content-Length: ";
  Header += (int)(Content.length());
  Header += "\r\n\r\n";
  
  
  wifiSerial.print("AT+CIPSEND=");
  wifiSerial.print(ch_id);
  wifiSerial.print(",");
  wifiSerial.println(Header.length()+Content.length());
  delay(10);
  
  // for debug buffer serial error
  //while (espSerial.available() >0 )  {
  //  char c = espSerial.read();
  //  dbgTerminal.write(c);
  //  if (c == '>') {
  //      espSerial.print(Header);
  //      espSerial.print(Content);
  //  }
  //}
  
  if (wifiSerial.find(">")) {
      wifiSerial.print(Header);
      wifiSerial.print(Content);
      delay(10);
   }
 
//  Serial1.print("AT+CIPCLOSE=");
//  Serial1.println(ch_id);


}

void doHttpLogic() {
  /*
  int ch_id, packet_len;
  char *pb;
  */

  DEBUG_PRINT("doHttpLogic");
  DEBUG_PRINT("Listening:");
  DEBUG_PRINT(listening);
  
  
  if(false) {  
  
    if(wifiSerial.available()) {
      /*
      wifiSerial.readBytesUntil('\n', buffer, BUFFER_SIZE);
      
      if(strncmp(buffer, "+IPD,", 5)==0) {
        // request: +IPD,ch,len:data
        sscanf(buffer+5, "%d,%d", &ch_id, &packet_len);
  
        if (packet_len > 0) {
          // read serial until packet_len character received
          // start from :
          pb = buffer+5;
          while(*pb!=':') pb++;
          pb++;
          if (strncmp(pb, "GET / ", 6) == 0) {
            DEBUG_PRINT(buffer);
            delay(100);
            clearSerialBuffer();
    
            homepage(ch_id);
          }
        }
        
      }
      */
    }
    
    clearBuffer();
  }

}


void startListening() {
  listening = true;
}

void stopListening() {
  listening = false;
}

