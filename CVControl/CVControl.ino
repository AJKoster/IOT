/*
Name:		CVmasterControl.ino
Created:	8/9/2017 1:31:50 PM
Author:	Addy
*/
/**************************************************************
* timeinput.ino Demonstrate interaction of Time library with
* Blynk's TimeInput widget.
* App project setup:
* RTC widget (no pin required!!!)
* V1 : Manual/Auto button
* V2: On-off button
* Terminal on V3 // Label will be the clock + wifi signal!!!!
*
* Time Input widget on V4 (Monday-Friday)
* Button selection for Time Input (Monday-Friday) on V5
*
* Time Input widget on V6 (Saturday-Sunday)
* Button selection for Time Input (Saturday-Sunday on V7
*
* Time Input widget on V8 (All days)
* Button selection for Time Input (All days) on V9
*
* Time Input widget on V10 (Up to you)
* Button selection for Time Input (Up to you) on V11
*
**************************************************************/

// Version
String version = "1.60";
String program = "CVControl";
#define Satellite 1 // 1 = CVmasterControl, 2 = .., 3 = ...

// Directives
#define DEBUG_THERMOSTAT // enable debugging mode
//#define DEBUG_BlynkTerminal // enable debugging on blynk terminal
#define dbSerialEM Serial


#ifdef DEBUG_BlynkTerminal
#define dbSerialEMPrint(a)    terminal1.print(a)
#define dbSerialEMPrintln(a)  terminal1.println(a)
#define dbSerialEMBegin(a)    do{}while(0)
#else
#ifdef DEBUG_THERMOSTAT
#define dbSerialEMPrint(a)    dbSerialEM.print(a)
#define dbSerialEMPrintln(a)  dbSerialEM.println(a)
#define dbSerialEMBegin(a)    dbSerialEM.begin(a)
#else
#define dbSerialEMPrint(a)    do{}while(0)
#define dbSerialEMPrintln(a)  do{}while(0)
#define dbSerialEMBegin(a)    do{}while(0)
#endif
#endif

//Start timer
unsigned long startMillis = millis();

//Ntp time server
#define DEBUG_NTPCLIENT
#include <Time.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <Wire.h>

// Include RTC library
// -------------------
#include "RTClib.h"
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
bool RTCUsed = false;

void CheckRTC() {
	DateTime now = rtc.now();
	if (rtc.lostPower()) {
		dbSerialEMPrintln(ConvertMillis() + " RTC lost power, lets set the time!");

		// At start-up time is set to 2013 wait until new date value from NTP server. If so, update
		if (year() != 2013) {
			rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
		}
	}
	// check NTP time with system time. Update on difference and year <> 2013
	if (now.minute() != minute()) {
		if (year() != 2013) {
			rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
		}
		else {
			setTime(now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
		}
	}
}

// Display the time from the RTC
//------------------------------
void GetRTCTime() {
	CheckRTC();
	DateTime now = rtc.now();
	dbSerialEMPrint(ConvertMillis() + " " + String(now.year(), DEC));
	dbSerialEMPrint('/');
	dbSerialEMPrint(String(now.month(), DEC));
	dbSerialEMPrint('/');
	dbSerialEMPrint(String(now.day(), DEC));
	dbSerialEMPrint(" (");
	dbSerialEMPrint(daysOfTheWeek[now.dayOfTheWeek()]);
	dbSerialEMPrint(") ");
	dbSerialEMPrint(String(now.hour(), DEC));
	dbSerialEMPrint(':');
	dbSerialEMPrint(String(now.minute(), DEC));
	dbSerialEMPrint(':');
	dbSerialEMPrint(String(now.second(), DEC));
	dbSerialEMPrintln();
}

// convert millis to hr:min:sec.mil
// --------------------------------
String ConvertMillis()
{
	unsigned long PickTime = millis();
	unsigned long milliseconds = (unsigned long)(PickTime) % 1000;
	unsigned int seconds = (((unsigned long)(PickTime)-milliseconds) / 1000) % 60;
	unsigned int minutes = (((((unsigned long)(PickTime)-milliseconds) / 1000) - seconds) / 60) % 60;
	unsigned int hours = ((((((unsigned long)(PickTime)-milliseconds) / 1000) - seconds) / 60) - minutes) / 60;
	String Result = "";
	if (hours < 10) {
		Result += " ";
	}
	Result += String(hours) + ":";

	if (minutes < 10) {
		Result += " ";
	}
	Result += String(minutes) + ":";
	if (seconds < 10) {
		Result += " ";
	}
	Result += String(seconds) + ".";

	if (milliseconds < 10) {
		Result += "00";
	}
	else if (milliseconds < 100) {
		Result += "0";
	}
	Result += String(milliseconds);
	return Result;
}

// Include WiFi handler
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

// OTA update
// https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266httpUpdate
// http://www.switchdoc.com/2016/12/iot-esp8266-tutorial-ota-software-updates-arduino-ide/
// http://esp8266.github.io/Arduino/versions/2.3.0/
// http://arduino.esp8266.com/stable/package_esp8266com_index.json
// http://esp8266.github.io/Arduino/versions/2.3.0/doc/changes.html

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <ESP8266WiFiMulti.h>

// TaskSchedular definitions
#include <TaskScheduler.h>		  //https://github.com/arkhipenko/TaskScheduler
Scheduler runner;

// Callback methods prototypes (tasks)
void postDataDomoticz();
void DoIt();
void performOTA();
void watchDog();
void SendDataToBlynk();
void controlHeating();
void activetoday();
void reconnectBlynk();
void clockvalue();
void sendWifi();
void ResetMinMax();
void InitThermostaat();

//Variables
String tsDataDomoticz;
String tsDateTime;
int WDstatus;
String FunctionName;
String TimeString;
int OldDay = -1;
float lastTemp_out, lastTemp_in;

// output pins
const byte WdLED = 2; // GPIO02 / pin D4 
const byte HeatingMaster = 15; // GPIO15 / pin D8
const byte HeatingFloor = 13; // GPIO13 / pin D7

//Tasks
Task t1(3600000L, TASK_FOREVER, &performOTA, &runner);	// 60 minutes
Task t2(30000L, TASK_FOREVER, &DoIt, &runner);	// 30 seconds
Task t3(300000L, TASK_FOREVER, &postDataDomoticz, &runner);	// 5 minutes
Task t4(1000L, TASK_FOREVER, &watchDog, &runner);	// 1 seconds beat
Task t5(30000L, TASK_FOREVER, &SendDataToBlynk, &runner);	// 30 seconds beat
Task t6(10000L, TASK_FOREVER, &controlHeating, &runner);	// 10 seconds beat
Task t7(10000L, TASK_FOREVER, &activetoday, &runner);	// 10 seconds beat
Task t8(30000L, TASK_FOREVER, &reconnectBlynk, &runner);	// 30 seconds beat
Task t9(5000L, TASK_FOREVER, &clockvalue, &runner);	// 5 seconds beat
Task t10(5000L, TASK_FOREVER, &sendWifi, &runner);	// 5 seconds beat
Task t11(10000L, TASK_FOREVER, &ResetMinMax, &runner);	// 10 seconds beat
Task t12(0, 1, &InitThermostaat, &runner);	// runs once after startup

#include <Ticker.h>
Ticker tickerOSWatch;
#define OSWATCH_RESET_TIME 60
static unsigned long last_loop;

// Time Schedular
#include <SimpleTimer.h>
//#include <TimeLib.h>
SimpleTimer timer;

#define TestLED 2                 // on board LED pin assignment
char Date[16];
char Time[16];

long startsecondswd;            // weekday start time in seconds
long stopsecondswd;             // weekday stop  time in seconds
long nowseconds;                // time now in seconds
bool isFirstConnect = true;

String displaycurrenttimepluswifi;
int wifisignal;
int manual = 0;
int oldstatus;

int mondayfriday;
int saturdaysunday;
int alldays;
int uptoyou;
int lastStatus = 0;
int newStatus = 0;

bool restore_completed = false;

// Blynk
#define BLYNK_PRINT dbSerialEM
#include <BlynkSimpleEsp8266.h>
#include <WidgetRTC.h>
bool BlynkServerUsed = false;
const char* BlynkAuth; // Blynk token
const char* BlynkServer; // IP adress Blynk server
WidgetTerminal terminal(V3); // text window
WidgetTerminal terminal1(V20); // text window

#define BLYNK_GREEN     "#23C48E"
#define BLYNK_BLUE      "#04C0F8"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"
#define BLYNK_DARK_BLUE "#5F7CD8"

float temp_in_max;
float temp_in_min;
float temp_out_max;
float temp_out_min;

//void configModeCallback (WiFiManager *myWiFiManager) {
//  
//    dbSerialEMPrintln("Entered config mode");
//    dbSerialEMPrintln(WiFi.softAPIP());
//    //if you used auto generated SSID, print it
//    dbSerialEMPrintln(myWiFiManager->getConfigPortalSSID());
//  
//}

// variables for sensor definition
const char* satellite_name;
const char* satellite_network_pw;

String writeAPIKey;
WiFiClient client;

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);

// Domoticz connection
bool DomoticzServerUsed = false;
const char* DomoticzServer; // IP address Domotics server
const char* DomoticzUserPass; // Token
const char* DomoticzId;


// Function to check I2C devices
// -----------------------------
uint8_t portArray[] = { 16, 5, 4, 0, 2, 14, 12, 13 };
//String portMap[] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7"}; //for Wemos
String portMap[] = { "GPIO16", "GPIO5", "GPIO4", "GPIO0", "GPIO2", "GPIO14", "GPIO12", "GPIO13" };

void scanPorts() {
	for (uint8_t i = 0; i < sizeof(portArray); i++) {
		for (uint8_t j = 0; j < sizeof(portArray); j++) {
			if (i != j) {
				dbSerialEMPrint("Scanning (SDA : SCL) - " + portMap[i] + " : " + portMap[j] + " - ");
				Wire.begin(portArray[i], portArray[j]);
				check_if_exist_I2C();
				// Break long sequence
				yield();
			}
		}
	}
}

void check_if_exist_I2C() {
	byte error, address;
	int nDevices;
	nDevices = 0;

	for (address = 1; address < 127; address++) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			dbSerialEMPrintln();
			dbSerialEMPrint(ConvertMillis() + " I2C device found at address 0x");
			if (address < 16)
				dbSerialEMPrint("0");
			dbSerialEMPrint(String(address, HEX));
			dbSerialEMPrint("  !");
			nDevices++;
		}
		else if (error == 4) {
			dbSerialEMPrintln();
			dbSerialEMPrint(ConvertMillis() + "Unknow error at address 0x");
			if (address < 16)
				dbSerialEMPrint("0");
			dbSerialEMPrint(String(address, HEX));
		}
	} //for loop
	if (nDevices == 0) {
		dbSerialEMPrintln();
		dbSerialEMPrintln(ConvertMillis() + " No I2C devices found");
	}
	else {
		dbSerialEMPrintln();
		dbSerialEMPrintln(ConvertMillis() + " **********************************");
	}
}

// Print Wifi status
// -----------------
void printWifiStatus() {
	// print the SSID of the network you're attached to
	dbSerialEMPrint(ConvertMillis() + " Connected to SSID: ");
	dbSerialEMPrintln(WiFi.SSID());

	// print your WiFi shield's IP address
	IPAddress ip = WiFi.localIP();
	dbSerialEMPrint(ConvertMillis() + " IP Address: ");
	dbSerialEMPrintln(ip);

	// print the received signal strength
	long rssi = WiFi.RSSI();
	dbSerialEMPrint(ConvertMillis() + " Signal strength (RSSI):");
	dbSerialEMPrint(rssi);
	dbSerialEMPrintln(" dBm");
}

//WatchDog
void watchDog()
{
	WDstatus = (WDstatus == 0);
	// set watchdog in right state
	//digitalWrite(WdLED, WDstatus);
	// send data to Blynk
	if (BlynkServerUsed) {
		Blynk.virtualWrite(V43, 155 * WDstatus);
	}
}


// This method makes a HTTP connection to the Domoticz server and transfer data
// ----------------------------------------------------------------------------
void postDataDomoticz()
{
	// Combine yourdatacolumn header (yourdata=) with the data recorded from your arduino
	// (yourarduinodata) and package them into the String yourdata which is what will be
	// sent in your POST request

	if (tsDataDomoticz == "" || !DomoticzServerUsed)
	{
		dbSerialEMPrintln(ConvertMillis() + " No valid data for Domoticz...");
		return;
	}

	// If there's data and a successful connection, send the HTTP POST request

	if (BlynkServerUsed) {
		terminal1.println(TimeString + " Domoticz connect");
	}
	FunctionName = __FUNCTION__;
	dbSerialEMPrintln(ConvertMillis() + " Connecting to Domoticz...");
	dbSerialEMPrint(String("GET ") + tsDataDomoticz + " HTTP/1.1\r\n" +
		"Host: " + DomoticzServer + " \r\n" +
		"Connection: close\r\n" +
		"Authorization: Basic " + DomoticzUserPass + " \r\n" +
		"Content-Length: 0\r\n" +
		"\r\n"
	);

	if (!client.connect(DomoticzServer, 8080)) {

		dbSerialEMPrintln(ConvertMillis() + " Connection to Domoticz failed (1)...");
		if (BlynkServerUsed) {
			terminal1.println(TimeString + " Domoticz failed");
		}
		return;
	}

	dbSerialEMPrintln(ConvertMillis() + " Connected to Domoticz ...");

	// This will send the request to the server 
	client.print(String("GET ") + tsDataDomoticz + " HTTP/1.1\r\n" +
		"Host: " + DomoticzServer + " \r\n" +
		"Connection: close\r\n" +
		"Authorization: Basic " + DomoticzUserPass + " \r\n" +
		"Content-Length: 0\r\n" +
		"\r\n"
	);

	if (client.connected()) {
		//delay(200);
		client.setTimeout(500);
		while (client.available()) {
			String line = client.readStringUntil('\r');
			dbSerialEMPrint(line);
		}

		// Close connection
		dbSerialEMPrintln("\n" + ConvertMillis() + " Disconnecting Domoticz...");
		client.stop();
		if (BlynkServerUsed) {
			terminal1.println(TimeString + " Domoticz success");
		}
	}
	else {
		// If you couldn't make a connection:
		dbSerialEMPrintln(ConvertMillis() + " Connection Domoticz failed (2)");
		if (BlynkServerUsed) {
			terminal1.println(TimeString + " Domoticz failed");
		}
	}
	FunctionName = "idle";
}

// This method gets the data and prepares the global strings
// ---------------------------------------------------------
void DoIt() {

	FunctionName = __FUNCTION__;

	dbSerialEMPrintln(ConvertMillis() + " DoIt started... ");

	// make date stamp
	if (RTCUsed) {
		DateTime now = rtc.now();
		TimeString = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
	}
	else {
		TimeString = String(year()) + "-" + String(month()) + "-" + String(day()) + " " + String(hour()) + ":" + String(minute()) + ":" + String(second());
	}

	// Prepare strings if valid values are available
	// ---------------------------------------------
	// Send data to Domoticz
	if (DomoticzServerUsed) {

		//tsDataDomoticz = "/json.htm?type=command&param=udevice&idx=" + String(DomoticzId) + "&nvalue=0&svalue=" + String(DHT_temp) + ";" + String(DHT_humidity) + ";" + String(DHT_humidity_stat);


	}
	else {
		tsDataDomoticz = "";
	}

	//Display RTC time
	if (RTCUsed) {
		GetRTCTime();
	}

	dbSerialEMPrintln(ConvertMillis() + " " + tsDataDomoticz);
	dbSerialEMPrintln(ConvertMillis() + " DoIt stopped... ");
}

void SendDataToBlynk() {
	if (BlynkServerUsed) {
		// Send data to Blynk
		FunctionName = __FUNCTION__;
		dbSerialEMPrintln(ConvertMillis() + " Start Blynk communication");
		//terminal1.println(TimeString + " Start Blynk communication");

		// Software version
		Blynk.virtualWrite(V45, version);

		//High and low temperatures

		Blynk.virtualWrite(V61, temp_in_max);
		Blynk.virtualWrite(V62, temp_in_min);
		Blynk.virtualWrite(V51, temp_out_max);
		Blynk.virtualWrite(V52, temp_out_min);

		dbSerialEMPrintln(ConvertMillis() + " Stop Blynk communication");
		//terminal1.println(TimeString + " Stop Blynk communication");

		FunctionName = "idle";
	}
}

// Init Ntp server
//----------------
void initNtp()
{
	dbSerialEMPrintln(ConvertMillis() + " Connect Ntp server");
	setTime(1357041600); // Jan 1 2013
	NTP.begin("nl.pool.ntp.org", 1, true); // Netherlands
										   // Break long sequence
	yield();
	NTP.setInterval(15, 24 * 3600); // OPTIONAL. Set sync interval: once a day after first match
	dbSerialEMPrintln(ConvertMillis() + " Ready Ntp server");
}

void printDigits(int digits) {
	// utility function for digital clock display: prints preceding colon and leading 0

	dbSerialEMPrint(":");
	if (digits < 10)
		dbSerialEMPrint('0');
	dbSerialEMPrint(digits);
}

void CheckFlashConfig() {
	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	FlashMode_t ideMode = ESP.getFlashChipMode();

	dbSerialEMPrintln("Flash real id:   " + String(ESP.getFlashChipId()));
	dbSerialEMPrintln("Flash real size: " + String(realSize));
	dbSerialEMPrintln("Flash ide  size: " + String(ideSize));
	dbSerialEMPrintln("Flash ide speed: " + String(ESP.getFlashChipSpeed()));
	dbSerialEMPrintln("Flash ide mode:  " + String((ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN")));

	if (ideSize != realSize) {
		dbSerialEMPrintln("Flash Chip configuration wrong!\n");
	}
	else {
		dbSerialEMPrintln("Flash Chip configuration ok.\n");
	}
}

void setup_serial_debug(void)
{
	/* Set the baudrate which is for debug and communicate with Nextion screen. */

	dbSerialEMBegin(115200);
	delay(2000);
	dbSerialEMPrintln(ConvertMillis() + " Serial Setup done");
}

void setup() {

	//Define sensor
	switch (Satellite) {
	case 1:
		satellite_name = "CVcontrol";
		satellite_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = true;

		// Domoticz
		DomoticzServerUsed = true;
		DomoticzServer = "192.168.2.15"; // adress Domotics server
		DomoticzUserPass = "YWRtaW46IyNEb21vdGljeiMj";

		DomoticzId = "17";

		// Blynk
		BlynkServerUsed = true;
		BlynkServer = "192.168.2.15"; // IP address Blynk server
//		BlynkAuth = "06a8e6ec53b34c15a8d3dc7a45f8ed6b"; // Blynk Token at house@my-net.nl account
		BlynkAuth = "af26c5fce7bf4d979bb3da1f1e6f7a34"; // Blynk Token at aj.koster@my-net.nl account

		break;
	case 2:
		satellite_name = "Satellite2";
		satellite_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = false;

		// Domoticz
		DomoticzServerUsed = false;
		DomoticzServer = "192.168.2.15"; // IP address Domotics server
		DomoticzUserPass = "";
		DomoticzId = "21";

		// Blynk
		BlynkServerUsed = false;
		BlynkServer = "192.168.2.15"; // IP address Blynk server
		BlynkAuth = ""; // Blynk Token

		break;
	case 3:
		satellite_name = "Satellite3";
		satellite_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = false;

		// Domoticz
		DomoticzServerUsed = false;
		DomoticzServer = ""; // IP address Domotics server
		DomoticzUserPass = "";
		DomoticzId = "";

		//Blynk
		BlynkServerUsed = false;
		BlynkServer = "192.168.2.15"; // IP address Blynk server
		BlynkAuth = ""; // Blynk Token

		break;
	default:
		satellite_name = "SatelliteDefault";
		satellite_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = false;

		// Domoticz
		DomoticzServerUsed = false;
		DomoticzServer = ""; // IP address Domotics server
		DomoticzUserPass = "";
		DomoticzId = "";

		//Blynk
		BlynkServerUsed = false;
		BlynkServer = ""; // IP address Blynk server		
		BlynkAuth = ""; // Blynk Token

		break;
	}
	setup_serial_debug();

	// Hello message
	//--------------
	dbSerialEMPrintln(ConvertMillis() + " Program: " + program);
	dbSerialEMPrintln(ConvertMillis() + " Version: " + version);
	dbSerialEMPrintln(ConvertMillis() + " Device: " + satellite_name);

	// prepare GPIO1 (flag start initialization (actual value will be restored via Blynk)
	pinMode(HeatingMaster, OUTPUT);
	pinMode(HeatingFloor, OUTPUT);

	//WiFiManager
	//-----------
	//Local intialization. Once its business is done, there is no need to keep it around
	WiFiManager wifiManager;
	//reset settings - for testing
	//wifiManager.resetSettings();

#ifdef DEBUG_THERMOSTAT

#else
	wifiManager.setDebugOutput(false);
#endif
	// We start by connecting to a WiFi network
	//set a timeout so the ESP doesn't hang waiting to be configured, for instance after a power failure
	wifiManager.setConfigPortalTimeout(120);

	//tell WiFiManager not to show networks below an arbitrary quality %;
	wifiManager.setMinimumSignalQuality(10);

	//set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
	//wifiManager.setAPCallback(configModeCallback);

	//fetches ssid and pass and tries to connect
	//if it does not connect it starts an access point with the specified name
	//here: sensor_name and goes into a blocking loop awaiting configuration
	yield();

	if (!wifiManager.autoConnect(satellite_name, satellite_network_pw)) {
		dbSerialEMPrintln(ConvertMillis() + " Failed to connect and hit timeout");
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(1000);
	}

	//if you get here you have connected to the WiFi
	dbSerialEMPrintln(ConvertMillis() + " Connected...yeey :)");
	printWifiStatus();

	// init Blynk
	if (BlynkServerUsed) {
		Blynk.config(BlynkAuth, BlynkServer);
		while (Blynk.connect() == false); // should be before rtc.begin(); to starte drectly with synchronization!
		terminal1.println(TimeString + " Blynk connected");
		terminal1.flush();
	}
	// reset all
	deactivateHeating();
	deactivateFloorHeating();

	// Time clock
	//timer.setInterval(10000L, activetoday);  // check every 10 SECONDS if schedule should run today 
	//timer.setInterval(30000L, reconnectBlynk);  // check every 30s if still connected to server 
	//timer.setInterval(5000L, clockvalue);  // check value for time
	//timer.setInterval(5000L, sendWifi);    // Wi-Fi singal

	// watch dog
	last_loop = millis();
	tickerOSWatch.attach_ms(((OSWATCH_RESET_TIME / 3) * 1000), osWatch);

	// prepare Watch LED
	pinMode(WdLED, OUTPUT);
	digitalWrite(WdLED, 1);

	// Check ESP8266 flash
	CheckFlashConfig();

	// initialize I2C bus (do not use it anymore in other libraries)
	int SDA = 4; //GPIO4=D2
	int SCL = 5; //GPIO5=D1
	Wire.begin(SDA, SCL);
	Wire.setClock(400000);
	delay(1000);

	// Optional scan I2C ports
	check_if_exist_I2C();

	// init Server
	//---------
	initServer();

	// init Ntp
	//---------
	initNtp();

	// get RTC time
	GetRTCTime();

	// Init TaskSkeduler and add/enable tasks
	//---------------------------------------
	runner.init();
	dbSerialEMPrintln(ConvertMillis() + " Initialized scheduler");
	runner.addTask(t1);
	runner.addTask(t2);
	runner.addTask(t3);
	runner.addTask(t4);
	runner.addTask(t5);
	runner.addTask(t6);
	runner.addTask(t7);
	runner.addTask(t8);
	runner.addTask(t9);
	runner.addTask(t10);
	runner.addTask(t11);
	runner.addTask(t12);

	dbSerialEMPrintln(ConvertMillis() + " Tasks added to TaskScheduler");

	delay(1000);

	t1.enableDelayed(500);  // performOTA - 60 min
	t2.enableDelayed(1000); // DoIt - 30 sec
	t3.enableDelayed(1500); // postDataDomoticz - 5 minuten
	t4.enableDelayed(500); // watchDog - 1 sec
	t5.enableDelayed(2500); // SendDataToBlyn - 30 sec
	t6.enableDelayed(4000); // controlHeating - 10 sec
	t7.enableDelayed(3000); // activetoday - 10 sec
	t8.enableDelayed(4500); // reconnectBlynk - 30 sec
	t9.enableDelayed(4500); // clockvalue - 5 sec
	t10.enableDelayed(5000); // sendWifi - 5 sec
	t11.enableDelayed(60000); // Reset Min Max - 10 sec
	t12.enableDelayed(60000); // RInitThermostaat - runs 1 time after startup

	dbSerialEMPrintln(ConvertMillis() + " Tasks enabled in TaskScheduler");

	if (BlynkServerUsed) {
		terminal1.println(TimeString + " Setup finished");
		terminal1.flush();
	}
}

void initServer() {

	// Start the server to receive messagesvia Domoticz
	// ------------------------------------------------
	server.begin();
	dbSerialEMPrintln(ConvertMillis() + " Server started");
	// Print the IP address
	dbSerialEMPrint(ConvertMillis() + " IP address: ");
	dbSerialEMPrintln(WiFi.localIP());
}

// Handle server request from Domoticz
//------------------------------------
void handle_server()
{
	WiFiClient client = server.available();
	int ConnectionWatchDog = 0;

	if (!client) {
		return;
	}

	// Wait until the client sends some data

	dbSerialEMPrintln(ConvertMillis() + " New client");

	ConnectionWatchDog = 0;

	while ((!client.available()) & (ConnectionWatchDog < 25000)) {
		delay(1);
		ConnectionWatchDog += 1;
	}
	if (!client.available()) {
		client.stop();
		return;
	}
	// Read the first line of the request
	client.setTimeout(500);
	String req = client.readStringUntil('\r');

	dbSerialEMPrintln(req);

	client.flush();

	// Match the request
	int val;
	String StatusAddy;

	if (req.indexOf("/gpio/0") != -1)
	{
		val = 0;
		StatusAddy = "Addy weg";
	}
	else if (req.indexOf("/gpio/1") != -1)
	{
		val = 1;
		StatusAddy = "Addy thuis";
	}
	else {
		dbSerialEMPrintln("invalid request");
		client.stop();
		return;
	}

	// Set GPIO15 according to the request
	//digitalWrite(HeatingMaster, val); // pin D8 / GPIO15 

									  // Synchronize Blynk
	if (BlynkServerUsed) {
		Blynk.virtualWrite(V46, StatusAddy);
		terminal1.println(TimeString + StatusAddy);
		terminal1.flush();

	}
	client.flush();

	// Prepare the response
	String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\nGPIO is now ";
	s += (val) ? "high" : "low";
	s += "</html>\n";

	// Send the response to the client
	client.print(s);
	delay(1);

	dbSerialEMPrintln(ConvertMillis() + " Client disonnected");
}

void performOTA() {
	FunctionName = __FUNCTION__;

	dbSerialEMPrintln(ConvertMillis() + " Start OTA");
	t_httpUpdate_return ret = ESPhttpUpdate.update("ota.my-net.nl", 80, "/ota_check.php", program + "_" + version);
	dbSerialEMPrint("Ret code: "); dbSerialEMPrintln(ret);
	switch (ret) {
	case HTTP_UPDATE_FAILED:
		dbSerialEMPrintln("HTTP_UPDATE_FAILD Error : " + String(ESPhttpUpdate.getLastError()) + String(ESPhttpUpdate.getLastErrorString().c_str()));
		break;
	case HTTP_UPDATE_NO_UPDATES:
		dbSerialEMPrintln("HTTP_UPDATE_NO_UPDATES");
		break;
	case HTTP_UPDATE_OK:
		dbSerialEMPrintln("HTTP_UPDATE_OK");
		break;
	}
	dbSerialEMPrintln("\n" + ConvertMillis() + " Stop OTA");
	FunctionName = "idle";
}

// Blynk Calls

// This function will be called every time button is pressed
// in Blynk app writes values to the Virtual Pin 10

BLYNK_WRITE(V44)
{
	int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
								  // You can also use:
								  // String i = param.asStr();
								  // double d = param.asDouble();
	dbSerialEMPrintln(ConvertMillis() + " V44 value is: ");
	dbSerialEMPrintln(pinValue);

	if (pinValue == 0)
	{
		// Not used

	}
	else
	{
		if (BlynkServerUsed) {
			terminal1.println(TimeString + " Forced restart " + FunctionName);
			terminal1.flush();
		}
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(1000);
	}
}

void InitThermostaat() {

	if (V2 > 0) { // heating is on

		newStatus = 0;
	}
}


void activateHeating() {
	// Set MasterHeating on
	digitalWrite(HeatingMaster, 0); // pin D8 / GPIO15
	lastStatus = 0;
	newStatus = 0;
	Blynk.virtualWrite(V40, 1);
	Blynk.setProperty(V40, "color", BLYNK_RED);
	terminal1.println(TimeString + " Master Heating On ");
	terminal1.flush();
}

void deactivateHeating() {
	// Set MasterHeating off
	digitalWrite(HeatingMaster, 1); // pin D8 / GPIO15
	lastStatus = 1;
	newStatus = 1;
	Blynk.virtualWrite(V40, 0);
	Blynk.setProperty(V40, "color", BLYNK_GREEN);
	terminal1.println(TimeString + " Master Heating Off ");
	terminal1.flush();
}

void activateFloorHeating() {
	// Set FloorHeating on
	digitalWrite(HeatingFloor, 0); // pin D8 / GPIO15

	Blynk.virtualWrite(V41, 1);
	Blynk.setProperty(V41, "color", BLYNK_RED);
	terminal1.println(TimeString + " Floor Heating On ");
	terminal1.flush();
}

void deactivateFloorHeating() {
	// Set FloorrHeating off
	digitalWrite(HeatingFloor, 1); // pin D8 / GPIO15

	Blynk.virtualWrite(V41, 0);
	Blynk.setProperty(V41, "color", BLYNK_GREEN);
	terminal1.println(TimeString + " Floor Heating Off ");
	terminal1.flush();
}
// This function will be called every time button is pressed
// in Blynk app writes values to the Virtual Pin 40

BLYNK_WRITE(V40)
{
	int pinValue = param.asInt(); // assigning incoming value from pin V40 to a variable
								  // You can also use:
								  // String i = param.asStr();
								  // double d = param.asDouble();
	dbSerialEMPrint(ConvertMillis() + " V40 value is: ");
	dbSerialEMPrintln(pinValue);

	if (pinValue == 1)
	{
		activateHeating();
	}
	else
	{
		deactivateHeating();
	}
}

BLYNK_WRITE(V41)
{
	int pinValue = param.asInt(); // assigning incoming value from pin V40 to a variable
								  // You can also use:
								  // String i = param.asStr();
								  // double d = param.asDouble();
	dbSerialEMPrint(ConvertMillis() + " V41 value is: ");
	dbSerialEMPrintln(pinValue);

	if (pinValue == 1)
	{
		// Set GPIO15 according to the request
		activateFloorHeating();
	}
	else
	{
		deactivateFloorHeating();
	}
}
BLYNK_READ(V42) // Widget in the app READs Virtal Pin V42 with the certain frequency
{
	int hours, min, sec;
	String timeString;

	// This command writes Arduino's uptime to Virtual Pin V11
	sec = millis() / 1000;
	min = sec / 60;
	hours = min / 60;
	timeString = (String(hours) + ":" + String(min % 60) + ":" + String(sec % 60)).c_str();
	Blynk.virtualWrite(V42, timeString);
}

void ICACHE_RAM_ATTR osWatch(void) // watchdog
{
	unsigned long t = millis();
	unsigned long last_run = abs(t - last_loop);
	if (last_run >= (OSWATCH_RESET_TIME * 1000)) {
		// save the hit here to eeprom or to rtc memory if needed
		if (BlynkServerUsed) {
			terminal1.println(TimeString + " TimeOut from " + FunctionName);
			terminal1.flush();
		}
		ESP.restart();  // normal reboot 
						//ESP.reset();  // hard reset
		delay(1000);
	}
}

// START Time Clock

BLYNK_CONNECTED() {
	if (isFirstConnect) {
		Blynk.syncVirtual(V13); // get last status...
		isFirstConnect = false;
		dbSerialEMPrintln(ConvertMillis() + " Blynk connect has run: ");
	}
}

void sendWifi() {
	wifisignal = map(WiFi.RSSI(), -105, -40, 0, 100);
}

void clockvalue() // Digital clock display of the time
{
	int gmthour = hour();
	if (gmthour == 24) {
		gmthour = 0;
	}
	String displayhour = String(gmthour, DEC);
	int hourdigits = displayhour.length();
	if (hourdigits == 1) {
		displayhour = "0" + displayhour;
	}
	String displayminute = String(minute(), DEC);
	int minutedigits = displayminute.length();
	if (minutedigits == 1) {
		displayminute = "0" + displayminute;
	}
	displaycurrenttimepluswifi = "                                          Clock:  " + displayhour + ":" + displayminute + "               Signal:  " + wifisignal + " %";
	Blynk.setProperty(V3, "label", displaycurrenttimepluswifi);
}

void activetoday() {        // check if schedule should run today
	if ((year() != 2013) && (restore_completed)) {
		dbSerialEMPrintln(ConvertMillis() + " start active today " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
		if (mondayfriday == 1) {
			dbSerialEMPrintln(ConvertMillis() + " start active mondayfriday ");
			Blynk.syncVirtual(V4); // sync timeinput widget  
		}
		if (saturdaysunday == 1) {
			dbSerialEMPrintln(ConvertMillis() + " start active satsun ");
			Blynk.syncVirtual(V6); // sync timeinput widget  
		}
		if (alldays == 1) {
			Blynk.syncVirtual(V8); // sync timeinput widget  
		}
		if (uptoyou == 1) {
			Blynk.syncVirtual(V10); // sync timeinput widget  
		}
		checklastbuttonpressed();
		dbSerialEMPrintln(ConvertMillis() + " stop active today " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
	}
	else {
		terminal1.println(TimeString + " Restore Complete " + restore_completed);
		terminal1.flush();
	}
}

void checklastbuttonpressed() {

	dbSerialEMPrintln(ConvertMillis() + " start last button checked: " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
	if ((mondayfriday == 1) && (saturdaysunday == 0)) { oldstatus = 1; }
	if ((mondayfriday == 0) && (saturdaysunday == 1)) { oldstatus = 2; }
	if ((mondayfriday == 1) && (saturdaysunday == 1)) { oldstatus = 3; }
	if (alldays == 1) { oldstatus = 4; }
	if (uptoyou == 1) { oldstatus = 5; }
	if ((mondayfriday == 0) && (saturdaysunday == 0) && (alldays == 0) && (uptoyou == 0)) { oldstatus = 6; }

	Blynk.virtualWrite(V13, oldstatus); // store old status on server
	dbSerialEMPrintln(ConvertMillis() + " end last button checked: " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
}


void restorelastbuttonpressed() {

	dbSerialEMPrintln(ConvertMillis() + " start restorelastbuttonpressed: " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));

	if (oldstatus == 1) { mondayfriday = 1; Blynk.virtualWrite(V5, 1); }
	if (oldstatus == 2) { saturdaysunday = 1; Blynk.virtualWrite(V7, 1); }
	if (oldstatus == 3) { saturdaysunday = 1; mondayfriday = 1; Blynk.virtualWrite(V5, 1); Blynk.virtualWrite(V7, 1); }
	if (oldstatus == 4) { alldays = 1; Blynk.virtualWrite(V9, 1); }
	if (oldstatus == 5) { uptoyou = 1; Blynk.virtualWrite(V11, 1); }
	if (oldstatus == 6) {
		mondayfriday = 0;
		saturdaysunday = 0;
		alldays = 0;
		uptoyou = 0;
		Blynk.virtualWrite(V5, 0);
		Blynk.virtualWrite(V7, 0);
		Blynk.virtualWrite(V9, 0);
		Blynk.virtualWrite(V11, 0);
	}
	dbSerialEMPrintln(ConvertMillis() + " stop restorelastbuttonpressed: " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
}

// restoring oldstatus from server
BLYNK_WRITE(V13)
{
	dbSerialEMPrintln(ConvertMillis() + " start (WRITE(V13)): " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));


	oldstatus = param[0].asInt();
	restorelastbuttonpressed();
	if (oldstatus != 6) {
		//Blynk.virtualWrite(V1, 0);
		//resetManual();

	}
	restore_completed = true;
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V13)): " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
}

BLYNK_WRITE(V1)  // Manual/Auto selection
{
	dbSerialEMPrintln(ConvertMillis() + " start (WRITE(V1)): " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
	if (param.asInt() == 1) {
		manual = 1;

		terminal.println("Manual MODE is ON");
		terminal.println("Press ON/OFF button if required");
		terminal.flush();

		checklastbuttonpressed();

		alldays = 0;
		uptoyou = 0;
		mondayfriday = 0;
		saturdaysunday = 0;
		Blynk.virtualWrite(V5, 0);
		Blynk.virtualWrite(V7, 0);
		Blynk.virtualWrite(V9, 0);
		Blynk.virtualWrite(V11, 0);

	}
	else {
		restorelastbuttonpressed();
		manual = 0;

		terminal.println("Manual MODE is OFF");
		terminal.println("Auto MODE restored from last status");
		terminal.println("Wait for update (10 seconds as maximum)");
		terminal.flush();
		dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V1)): " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
	}
}

void resetTerminal()
{
	terminal.println("New MODE has been selected");
	terminal.println("Wait for update (10 seconds as maximum)");
	terminal.flush();
}

void resetManual()
{
	Blynk.virtualWrite(V1, 0);   //Turn OFF Manual Mode Widget
	Blynk.virtualWrite(V2, 0);   //Turn OFF Button Widget Device
	digitalWrite(TestLED, LOW); // set LED OFF
}


BLYNK_WRITE(V2)  // ON-OFF Manual 
{
	dbSerialEMPrintln(ConvertMillis() + " start (WRITE(V2)): " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
	if (param.asInt() == 1) {  // boton encendido  

		terminal.println("Manual MODE is ON");
		terminal.println("Press ON/OFF button if required");
		terminal.println("Device is ON");
		terminal.flush();

		if (manual == 0) {  //está en modo automático     
			checklastbuttonpressed();
			manual = 1;
			mondayfriday = 0;
			saturdaysunday = 0;
			alldays = 0;
			uptoyou = 0;
			Blynk.virtualWrite(V1, 1);
			Blynk.virtualWrite(V5, 0);
			Blynk.virtualWrite(V7, 0);
			Blynk.virtualWrite(V9, 0);
			Blynk.virtualWrite(V11, 0);
			digitalWrite(TestLED, HIGH); // set LED ON 
			Blynk.virtualWrite(V2, 1);   //Turn ON Button Widget
			newStatus = 0;
		}
		else {             //está en modo manual 
			mondayfriday = 0;
			saturdaysunday = 0;
			alldays = 0;
			uptoyou = 0;
			Blynk.virtualWrite(V1, 1);
			Blynk.virtualWrite(V5, 0);
			Blynk.virtualWrite(V7, 0);
			Blynk.virtualWrite(V9, 0);
			Blynk.virtualWrite(V11, 0);
			digitalWrite(TestLED, HIGH); // set LED ON 
			Blynk.virtualWrite(V2, 1);   //Turn ON Button Widget
			newStatus = 0;
		}
	}
	else {
		terminal.println("Manual MODE is ON");
		terminal.println("Press ON/OFF button if required");
		terminal.println("Device is OFF");
		terminal.flush();

		if (manual == 0) {      //modo automático
			checklastbuttonpressed();
			manual = 1;
			mondayfriday = 0;
			saturdaysunday = 0;
			alldays = 0;
			uptoyou = 0;
			Blynk.virtualWrite(V1, 1);
			Blynk.virtualWrite(V5, 0);
			Blynk.virtualWrite(V7, 0);
			Blynk.virtualWrite(V9, 0);
			Blynk.virtualWrite(V11, 0);
			digitalWrite(TestLED, LOW); // set LED OFF
			Blynk.virtualWrite(V2, 0);   //Turn OFF Button Widget
			newStatus = 1;
		}
		else {
			mondayfriday = 0;
			saturdaysunday = 0;
			alldays = 0;
			uptoyou = 0;
			Blynk.virtualWrite(V1, 1);
			Blynk.virtualWrite(V5, 0);
			Blynk.virtualWrite(V7, 0);
			Blynk.virtualWrite(V9, 0);
			Blynk.virtualWrite(V11, 0);
			digitalWrite(TestLED, LOW); // set LED OFF
			Blynk.virtualWrite(V2, 0);   //Turn OFF Button Widget
			newStatus = 1;
		}
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V2)): " + String(oldstatus) + " " + String(mondayfriday) + " " + String(saturdaysunday) + " " + String(alldays) + " " + String(uptoyou));
}

BLYNK_WRITE(V5)  // Monday-Friday selected
{
	dbSerialEMPrintln(ConvertMillis() + " workdays (WRITE(V5)): " + String(oldstatus));
	if (param.asInt() == 1 && (V1 == 1)) {
		timer.setTimeout(50, resetTerminal);
		timer.setTimeout(50, resetManual);
		timer.setTimeout(50, checklastbuttonpressed);
		mondayfriday = 1;
		alldays = 0;
		uptoyou = 0;
		Blynk.virtualWrite(V9, 0);
		Blynk.virtualWrite(V11, 0);
	}
	else {
		mondayfriday = 0;

	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V5)): " + String(oldstatus));
}


BLYNK_WRITE(V7)  // Saturday-Sunday selected
{
	dbSerialEMPrintln(ConvertMillis() + " weekend (WRITE(V7)): " + String(oldstatus));
	if (param.asInt() == 1) {
		timer.setTimeout(50, resetTerminal);
		timer.setTimeout(50, resetManual);
		timer.setTimeout(50, checklastbuttonpressed);
		saturdaysunday = 1;
		alldays = 0;
		uptoyou = 0;
		Blynk.virtualWrite(V9, 0);
		Blynk.virtualWrite(V11, 0);
	}
	else {
		saturdaysunday = 0;
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V7)): " + String(oldstatus));
}

BLYNK_WRITE(V9)  // All days selected
{
	dbSerialEMPrintln(ConvertMillis() + " all days (WRITE(V9)): " + String(oldstatus));
	if (param.asInt() == 1) {
		timer.setTimeout(50, resetTerminal);
		timer.setTimeout(50, resetManual);
		timer.setTimeout(50, checklastbuttonpressed);
		alldays = 1;
		mondayfriday = 0;
		saturdaysunday = 0;
		uptoyou = 0;
		Blynk.virtualWrite(V5, 0);
		Blynk.virtualWrite(V7, 0);
		Blynk.virtualWrite(V11, 0);
	}
	else {
		alldays = 0;
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V9)): " + String(oldstatus));
}

BLYNK_WRITE(V11)  // Up to you selected
{
	dbSerialEMPrintln(ConvertMillis() + " uptoyou (WRITE(V11)): " + String(oldstatus));
	if (param.asInt() == 1) {
		timer.setTimeout(50, resetTerminal);
		timer.setTimeout(50, resetManual);
		timer.setTimeout(50, checklastbuttonpressed);
		uptoyou = 1;
		mondayfriday = 0;
		saturdaysunday = 0;
		alldays = 0;
		Blynk.virtualWrite(V5, 0);
		Blynk.virtualWrite(V7, 0);
		Blynk.virtualWrite(V9, 0);
	}
	else {
		uptoyou = 0;
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V11)): " + String(oldstatus));
}

BLYNK_WRITE(V4)//Monday-Friday
{
	dbSerialEMPrintln(ConvertMillis() + " workdays (WRITE(V4)): " + String(oldstatus));
	if (mondayfriday == 1) {
		sprintf(Date, "%02d/%02d/%04d", day(), month(), year());
		sprintf(Time, "%02d:%02d:%02d", hour(), minute(), second());

		TimeInputParam t(param);

		terminal.print("M-F Checked schedule at: ");
		terminal.println(Time);
		terminal.flush();
		int dayadjustment = -1;
		if (weekday() == 1) {
			dayadjustment = 6; // needed for Sunday, Time library is day 1 and Blynk is day 7
		}
		if (t.isWeekdaySelected(weekday() + dayadjustment)) { //Time library starts week on Sunday, Blynk on Monday
			terminal.println("Monday-Friday ACTIVE today");
			terminal.flush();
			if (t.hasStartTime()) // Process start time
			{
				terminal.println(String("Start: ") + t.getStartHour() + ":" + t.getStartMinute());
				terminal.flush();
			}
			if (t.hasStopTime()) // Process stop time
			{
				terminal.println(String("Stop : ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
			}
			// Display timezone details, for information purposes only 
			terminal.println(String("Time zone: ") + t.getTZ()); // Timezone is already added to start/stop time 
																 //  terminal.println(String("Time zone offset: ") + t.getTZ_Offset()); // Get timezone offset (in seconds)
			terminal.flush();

			for (int i = 1; i <= 7; i++) {  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
				if (t.isWeekdaySelected(i)) {
					terminal.println(String("Day ") + i + " is selected");
					terminal.flush();
				}
			}
			nowseconds = ((hour() * 3600) + (minute() * 60) + second());
			startsecondswd = (t.getStartHour() * 3600) + (t.getStartMinute() * 60);
			//Serial.println(startsecondswd);  // used for debugging
			if (nowseconds >= startsecondswd) {
				terminal.print("Monday-Friday STARTED at");
				terminal.println(String(" ") + t.getStartHour() + ":" + t.getStartMinute());
				terminal.flush();
				if (nowseconds <= startsecondswd + 90) {    // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, HIGH); // set LED ON
					Blynk.virtualWrite(V2, 1);
					// code here to switch the relay ON
					newStatus = 0;
				}
			}
			else {
				terminal.println("Monday-Friday Device NOT STARTED today");
				terminal.flush();

			}
			stopsecondswd = (t.getStopHour() * 3600) + (t.getStopMinute() * 60);
			//Serial.println(stopsecondswd);  // used for debugging
			if (nowseconds >= stopsecondswd) {
				digitalWrite(TestLED, LOW); // set LED OFF
				Blynk.virtualWrite(V2, 0);

				terminal.print("Monday-Friday STOPPED at");
				terminal.println(String(" ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
				if (nowseconds <= stopsecondswd + 90) {   // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, LOW); // set LED OFF
					Blynk.virtualWrite(V2, 0);
					// code here to switch the relay OFF
					newStatus = 1;
				}
			}
			else {
				if (nowseconds >= startsecondswd) {
					digitalWrite(TestLED, HIGH); // set LED ON    test
					Blynk.virtualWrite(V2, 1);
					terminal.println("Monday-Friday is ON");
					terminal.flush();
				}
			}
		}
		else {
			terminal.println("Monday-Friday INACTIVE today");
			terminal.flush();
			// nothing to do today, check again in 30 SECONDS time    
		}
		terminal.println();
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V4)): " + String(oldstatus));
}

BLYNK_WRITE(V6) //Saturday-Sunday
{
	dbSerialEMPrintln(ConvertMillis() + " weekend (WRITE(V6)): " + String(oldstatus));
	if (saturdaysunday == 1) {
		sprintf(Date, "%02d/%02d/%04d", day(), month(), year());
		sprintf(Time, "%02d:%02d:%02d", hour(), minute(), second());

		TimeInputParam t(param);

		terminal.print("S-S Checked schedule at: ");
		terminal.println(Time);
		terminal.flush();
		int dayadjustment = -1;
		if (weekday() == 1) {
			dayadjustment = 6; // needed for Sunday, Time library is day 1 and Blynk is day 7
		}
		if (t.isWeekdaySelected(weekday() + dayadjustment)) { //Time library starts week on Sunday, Blynk on Monday
			terminal.println("Saturday-Sunday ACTIVE today");
			terminal.flush();
			if (t.hasStartTime()) // Process start time
			{
				terminal.println(String("Start: ") + t.getStartHour() + ":" + t.getStartMinute());
				terminal.flush();
			}
			if (t.hasStopTime()) // Process stop time
			{
				terminal.println(String("Stop : ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
			}
			// Display timezone details, for information purposes only 
			terminal.println(String("Time zone: ") + t.getTZ()); // Timezone is already added to start/stop time 
			// terminal.println(String("Time zone offset: ") + t.getTZ_Offset()); // Get timezone offset (in seconds)
			terminal.flush();

			for (int i = 1; i <= 7; i++) {  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
				if (t.isWeekdaySelected(i)) {
					terminal.println(String("Day ") + i + " is selected");
					terminal.flush();
				}
			}
			nowseconds = ((hour() * 3600) + (minute() * 60) + second());
			startsecondswd = (t.getStartHour() * 3600) + (t.getStartMinute() * 60);
			//Serial.println(startsecondswd);  // used for debugging
			if (nowseconds >= startsecondswd) {
				terminal.print("Saturday-Sunday STARTED at");
				terminal.println(String(" ") + t.getStartHour() + ":" + t.getStartMinute());

				terminal.flush();
				if (nowseconds <= startsecondswd + 90) {    // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, HIGH); // set LED ON
					Blynk.virtualWrite(V2, 1);
					// code here to switch the relay ON
					newStatus = 0;
				}
			}
			else {
				terminal.println("Saturday-Sunday NOT STARTED today");
				terminal.flush();

			}
			stopsecondswd = (t.getStopHour() * 3600) + (t.getStopMinute() * 60);
			//Serial.println(stopsecondswd);  // used for debugging
			if (nowseconds >= stopsecondswd) {
				digitalWrite(TestLED, LOW); // set LED OFF
				Blynk.virtualWrite(V2, 0);

				terminal.print("Saturday-Sunday STOPPED at");
				terminal.println(String(" ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
				if (nowseconds <= stopsecondswd + 90) {   // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, LOW); // set LED OFF
					Blynk.virtualWrite(V2, 0);
					// code here to switch the relay OFF
					newStatus = 1;
				}
			}
			else {
				if (nowseconds >= startsecondswd) {
					digitalWrite(TestLED, HIGH); // set LED ON  TEST
					Blynk.virtualWrite(V2, 1);
					terminal.println("Saturday-Sunday is ON");
					terminal.flush();
				}
			}
		}
		else {
			terminal.println("Saturday-Sunday INACTIVE today");
			terminal.flush();
			// nothing to do today, check again in 30 SECONDS time    
		}
		terminal.println();
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V6)): " + String(oldstatus));
}


BLYNK_WRITE(V8)//All days
{
	dbSerialEMPrintln(ConvertMillis() + " all days (WRITE(V8)): " + String(oldstatus));
	if (alldays == 1) {
		sprintf(Date, "%02d/%02d/%04d", day(), month(), year());
		sprintf(Time, "%02d:%02d:%02d", hour(), minute(), second());

		TimeInputParam t(param);

		terminal.print("All Days Checked schedule at: ");
		terminal.println(Time);
		terminal.flush();
		int dayadjustment = -1;
		if (weekday() == 1) {
			dayadjustment = 6; // needed for Sunday, Time library is day 1 and Blynk is day 7
		}
		if (t.isWeekdaySelected(weekday() + dayadjustment)) { //Time library starts week on Sunday, Blynk on Monday
			terminal.println("ALL DAYS ACTIVE today");
			terminal.flush();
			if (t.hasStartTime()) // Process start time
			{
				terminal.println(String("Start: ") + t.getStartHour() + ":" + t.getStartMinute());
				terminal.flush();
			}
			if (t.hasStopTime()) // Process stop time
			{
				terminal.println(String("Stop : ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
			}
			// Display timezone details, for information purposes only 
			terminal.println(String("Time zone: ") + t.getTZ()); // Timezone is already added to start/stop time 
																 //  terminal.println(String("Time zone offset: ") + t.getTZ_Offset()); // Get timezone offset (in seconds)
			terminal.flush();

			for (int i = 1; i <= 7; i++) {  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
				if (t.isWeekdaySelected(i)) {
					terminal.println(String("Day ") + i + " is selected");
					terminal.flush();
				}
			}
			nowseconds = ((hour() * 3600) + (minute() * 60) + second());
			startsecondswd = (t.getStartHour() * 3600) + (t.getStartMinute() * 60);
			//Serial.println(startsecondswd);  // used for debugging
			if (nowseconds >= startsecondswd) {
				terminal.print("ALL DAYS STARTED at");
				terminal.println(String(" ") + t.getStartHour() + ":" + t.getStartMinute());
				terminal.flush();
				if (nowseconds <= startsecondswd + 90) {    // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, HIGH); // set LED ON
					Blynk.virtualWrite(V2, 1);
					// code here to switch the relay ON
					newStatus = 0;
				}
			}
			else {
				terminal.println("All Day Device NOT STARTED today");
				terminal.flush();

			}
			stopsecondswd = (t.getStopHour() * 3600) + (t.getStopMinute() * 60);
			//Serial.println(stopsecondswd);  // used for debugging
			if (nowseconds >= stopsecondswd) {
				digitalWrite(TestLED, LOW); // set LED OFF
				Blynk.virtualWrite(V2, 0);

				terminal.print("All day STOPPED at");
				terminal.println(String(" ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
				if (nowseconds <= stopsecondswd + 90) {   // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, LOW); // set LED OFF
					Blynk.virtualWrite(V2, 0);
					// code here to switch the relay OFF
					newStatus = 1;
				}
			}
			else {
				if (nowseconds >= startsecondswd) {
					digitalWrite(TestLED, HIGH); // set LED ON  TEST!!!!!
					Blynk.virtualWrite(V2, 1);

					terminal.println("All day is ON");
					terminal.flush();

				}
			}
		}
		else {
			terminal.println("All day INACTIVE today");
			terminal.flush();
			// nothing to do today, check again in 30 SECONDS time    
		}
		terminal.println();
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V8)): " + String(oldstatus));
}

BLYNK_WRITE(V10)//Up to you 
{
	dbSerialEMPrintln(ConvertMillis() + " uptoyou (WRITE(V10)): " + String(oldstatus));
	if (uptoyou == 1) {
		sprintf(Date, "%02d/%02d/%04d", day(), month(), year());
		sprintf(Time, "%02d:%02d:%02d", hour(), minute(), second());

		TimeInputParam t(param);

		terminal.print("Up to you Checked schedule at: ");
		terminal.println(Time);
		terminal.flush();
		int dayadjustment = -1;
		if (weekday() == 1) {
			dayadjustment = 6; // needed for Sunday, Time library is day 1 and Blynk is day 7
		}
		if (t.isWeekdaySelected(weekday() + dayadjustment)) { //Time library starts week on Sunday, Blynk on Monday
			terminal.println("Up to you ACTIVE today");
			terminal.flush();
			if (t.hasStartTime()) // Process start time
			{
				terminal.println(String("Start: ") + t.getStartHour() + ":" + t.getStartMinute());
				terminal.flush();
			}
			if (t.hasStopTime()) // Process stop time
			{
				terminal.println(String("Stop : ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
			}
			// Display timezone details, for information purposes only 
			terminal.println(String("Time zone: ") + t.getTZ()); // Timezone is already added to start/stop time 
			terminal.println("At least ONE day MUST be selected");
			// terminal.println(String("Time zone offset: ") + t.getTZ_Offset()); // Get timezone offset (in seconds)
			terminal.flush();

			for (int i = 1; i <= 7; i++) {  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
				if (t.isWeekdaySelected(i)) {
					terminal.println(String("Day ") + i + " is selected");
					terminal.flush();
				}
			}
			nowseconds = ((hour() * 3600) + (minute() * 60) + second());
			startsecondswd = (t.getStartHour() * 3600) + (t.getStartMinute() * 60);
			//Serial.println(startsecondswd);  // used for debugging
			if (nowseconds >= startsecondswd) {
				terminal.print("Up to you STARTED at");
				terminal.println(String(" ") + t.getStartHour() + ":" + t.getStartMinute());
				terminal.flush();
				if (nowseconds <= startsecondswd + 90) {    // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, HIGH); // set LED ON
					Blynk.virtualWrite(V2, 1);
					// code here to switch the relay ON
					newStatus = 0;
				}
			}
			else {
				terminal.println("UP to you Device NOT STARTED today");
				terminal.flush();

			}
			stopsecondswd = (t.getStopHour() * 3600) + (t.getStopMinute() * 60);
			//Serial.println(stopsecondswd);  // used for debugging
			if (nowseconds >= stopsecondswd) {
				digitalWrite(TestLED, LOW); // set LED OFF
				Blynk.virtualWrite(V2, 0);

				terminal.print("Up to you STOPPED at");
				terminal.println(String(" ") + t.getStopHour() + ":" + t.getStopMinute());
				terminal.flush();
				if (nowseconds <= stopsecondswd + 90) {   // 90s on 60s timer ensures 1 trigger command is sent
					digitalWrite(TestLED, LOW); // set LED OFF
					Blynk.virtualWrite(V2, 0);
					// code here to switch the relay OFF
					newStatus = 1;
				}
			}
			else {
				if (nowseconds >= startsecondswd) {
					digitalWrite(TestLED, HIGH); // set LED ON 
					Blynk.virtualWrite(V2, 1);
					terminal.println("Up to you is ON");
					terminal.flush();
				}
			}
		}
		else {
			terminal.println("Up to you INACTIVE today");
			terminal.flush();
			// nothing to do today, check again in 30 SECONDS time    
		}
		terminal.println();
	}
	dbSerialEMPrintln(ConvertMillis() + " stop (WRITE(V10)): " + String(oldstatus));
}

// get Bridge data / Garden temperature
BLYNK_WRITE(V50) {
	float pinData = param.asFloat(); //pinData variable will store value that came via Bridge
	Blynk.setProperty(V50, "color", MapTempColor(pinData));
	lastTemp_out = pinData;

	// Determine High and low temperature
	if (pinData > temp_out_max) {
		temp_out_max = pinData;
	}
	else {
		if (pinData < temp_out_min)
			temp_out_min = pinData;
	}
}

// get Bridge data / Home temperature
BLYNK_WRITE(V60) {
	float pinData = param.asFloat(); //pinData variable will store value that came via Bridge
	Blynk.setProperty(V60, "color", MapTempColor(pinData));
	lastTemp_in = pinData;

	// Determine High and low temperature
	if (pinData > temp_in_max) {
		temp_in_max = pinData;
	}
	else {
		if (pinData < temp_in_min)
			temp_in_min = pinData;
	}
}

// once a day and on start, reset min/max values of temperatures
void ResetMinMax() {

	if (day() != OldDay) {
		OldDay = day();
		temp_out_max = lastTemp_out;
		temp_out_min = temp_out_max;
		temp_in_max = lastTemp_in;
		temp_in_min = temp_in_max;
	}
}

// Map a value to a rainbow color.
String MapTempColor(float value)
{
	// Map different color bands.
	if (value > 30.0)
	{
		return "#FF0000";
	}
	else if (value > 25.0)
	{
		return "#FF7F00";
	}
	else if (value > 20.0)
	{
		return "#FFFF00";
	}
	else if (value > 15.0)
	{
		return "#00FF00";
	}
	else if (value > 10.0)
	{
		return "#0000FF";
	}
	else if (value > 5.0)
	{
		return "#4B0082";
	}
	else
		return "#9400D3";
}

// Control heating
void controlHeating() {

	terminal.println("Check if action is needed: " + String(newStatus) + "<->" + String(lastStatus));
	if (newStatus == lastStatus) {
		// confirm color
		if (lastStatus == 0) {
			Blynk.setProperty(V40, "color", BLYNK_RED);
		}
		else {
			Blynk.setProperty(V40, "color", BLYNK_GREEN);
		}
	}
	else {
		terminal.println("Action!!!!!!");
		if (newStatus == 0) { // activate heating
			Blynk.virtualWrite(V40, 1);
			activateHeating();
			terminal.println("Activate heating");

		}
		else { // stop heating
			Blynk.virtualWrite(V40, 0);
			deactivateHeating();
			terminal.println("Stop heating");

		}
	}
	terminal.flush();
}

// Control heating floor
void controlHeatingFloor() {


	terminal.println("Check if action floor heating is needed: " + String(newStatus) + "<->" + String(lastStatus));
	if (newStatus == lastStatus) {
		// confirm color
		if (lastStatus == 0) {
			Blynk.setProperty(V41, "color", BLYNK_RED);
		}
		else {
			Blynk.setProperty(V41, "color", BLYNK_GREEN);
		}
	}
	else {
		terminal.println("Action!!!!!!");
		if (newStatus == 0) // AND OUTSIDE TEMP < -4 for 1 hour then
		{
			Blynk.virtualWrite(V41, 1);
			activateFloorHeating();
			terminal.println("Activate floorheating");
		}
		else {
			Blynk.virtualWrite(V41, 0);
			deactivateFloorHeating();
			terminal.println("Stop floorheating");
		}
		//lastStatus = newStatus;

	}
	terminal.flush();
}

void reconnectBlynk() {
	if (!Blynk.connected()) {
		if (Blynk.connect(5000)) {
			BLYNK_LOG("Reconnected");
		}
		else {
			BLYNK_LOG("Not reconnected");
		}
	}
}

void loop() {
	runner.execute();
	handle_server();

	if (BlynkServerUsed) {
		Blynk.run();
	}
	// reset watchdog
	last_loop = millis();

	// Time schedular
	timer.run();

}