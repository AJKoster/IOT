/*
 Name:		TestUnit.ino
 Created:	8/12/2017 7:46:49 PM
 Author:	Addy
*/

/*
Name:		EnvironmentMonitor.ino
Created:	24/13/2016 1:43:04 PM
Author:	Addy Koster
*/

// Environment Monitor Software
// Uses WiFiManager (https://github.com/tzapu/WiFiManager)
// Uses Thinkspeak connection (Written by: Thomas Tongue)
// Date: December 24th, 2016
// All put together by
// A.J. Koster

// Version
String version = "1.05";
String program = "TestUnit";
#define Satellite 1 // 1 = CV switch, 2 = .., 3 = ...

// Directives
#define DEBUG_THERMOSTAT // enable debugging mode
//#define DEBUG_BlynkTerminal // enable debugging on blynk terminal
#define dbSerialEM Serial


#ifdef DEBUG_BlynkTerminal
#define dbSerialEMPrint(a)    terminal.print(a)
#define dbSerialEMPrintln(a)  terminal.println(a)
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

void CheckRTC () {
	DateTime now = rtc.now();
	if (rtc.lostPower()) {
		dbSerialEMPrintln(ConvertMillis() + " RTC lost power, lets set the time!");

		// At start-up time is set to 2013 wait until new date value from NTP server. If so, update
		if (year() != 2013) {
			rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
		}
	}
	// check NTP time with system time. Update on difference and year <> 2013
	if ((now.minute() != minute()) || (now.hour() != hour())) {
		if (year() != 2013) {
			rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
		}
		else{
			setTime(now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
		}
	}
}

// Display the time from the RTC
//------------------------------
void GetRTCTime() {	
	CheckRTC();
	DateTime now = rtc.now();
	dbSerialEMPrint(ConvertMillis() + " "+ String(now.year(), DEC));
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
void updateThingSpeak();
void postData();
void postDataDomoticz();
void DoIt();
void performOTA();
void watchDog();
void digitalClockDisplay();

void SendDataToBlynk();

//Variables
String tsDataDomoticz;
String tsDataThingSpeak;
String tsDataMyHouse;
String tsDateTime;
int WDstatus;
String FunctionName;

// output pins
const byte WdLED = 2; // GPIO02 / pin D4 

//Tasks
Task t1(3600000L, TASK_FOREVER, &performOTA, &runner);	// 60 minutes
Task t2(30000L, TASK_FOREVER, &DoIt, &runner);	// 30 seconds
Task t3(300000L, TASK_FOREVER, &postDataDomoticz, &runner);	// 5 minutes
Task t4(1000L, TASK_FOREVER, &watchDog, &runner);	// 1 seconds beat
Task t5(30000L, TASK_FOREVER, &SendDataToBlynk, &runner);	// 30 seconds beat
Task t7(60000L,   TASK_FOREVER, &digitalClockDisplay, &runner); // 60 seconds beat

#include <Ticker.h>
Ticker tickerOSWatch;
#define OSWATCH_RESET_TIME 60
static unsigned long last_loop;

// Blynk
#define BLYNK_PRINT dbSerialEM
#include <BlynkSimpleEsp8266.h>
bool BlynkServerUsed = false;
const char* BlynkAuth; // Blynk token
const char* BlynkServer; // IP adress Blynk server
WidgetTerminal terminal(V20); // text window

// sensor definitions
// ------------------
// CCS811

#include <SparkFunCCS811.h>
//#define CCS811_ADDR 0x5B //Default I2C Address (Add = high)
#define CCS811_ADDR 0x5A //Alternate I2C Address (Add = low)

CCS811 mySensor(CCS811_ADDR);
bool ccs811SensorInstalled = false; // default not installed.
float css811_CO2;
float css811_TVOC;


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

//printDriverError decodes the CCS811Core::status type and prints the
//type of error to the serial terminal.
//
//Save the return value of any function of type CCS811Core::status, then pass
//to this function to see what the output was.
void printDriverError(CCS811Core::status errorCode)
{
	switch (errorCode)
	{
	case CCS811Core::SENSOR_SUCCESS:
		dbSerialEMPrintln("ccs811 SUCCESS");
		break;
	case CCS811Core::SENSOR_ID_ERROR:
		dbSerialEMPrintln("ccs811 ID_ERROR");
		break;
	case CCS811Core::SENSOR_I2C_ERROR:
		dbSerialEMPrintln("ccs811 I2C_ERROR");
		break;
	case CCS811Core::SENSOR_INTERNAL_ERROR:
		dbSerialEMPrintln("ccs811 INTERNAL_ERROR");
		break;
	case CCS811Core::SENSOR_GENERIC_ERROR:
		dbSerialEMPrintln("ccs811 GENERIC_ERROR");
		break;
	default:
		dbSerialEMPrintln("ccs811 Unspecified error.");
	}
}

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


// Get values from CCS811 sensor
// ----------------------------

void get_ccs811()
{
	CCS811Core::status returnCode = mySensor.begin();

	if (returnCode != CCS811Core::SENSOR_SUCCESS) {
		//ccs811SensorInstalled = false;
		dbSerialEMPrint(ConvertMillis() + " Could not find a valid ccs811 sensor, error: ");
		printDriverError(returnCode);
		return;
	}
	
	//Check to see if data is ready with .dataAvailable()
	if (ccs811SensorInstalled) {
		if (mySensor.dataAvailable())
		{
			//If so, have the sensor read and calculate the results.
			//Get them later
			mySensor.readAlgorithmResults();
			dbSerialEMPrintln();
			dbSerialEMPrint(ConvertMillis() + "CO2[");
			//Returns calculated CO2 reading
			css811_CO2 = mySensor.getCO2();
			dbSerialEMPrint(css811_CO2);
			dbSerialEMPrint("] tVOC[");
			//Returns calculated TVOC reading
			css811_TVOC = mySensor.getTVOC();
			dbSerialEMPrint(css811_TVOC);
			dbSerialEMPrint("]");
			dbSerialEMPrintln();
		}
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
	digitalWrite(WdLED, WDstatus);
	// send data to Blynk
	if (BlynkServerUsed) {
		Blynk.virtualWrite(V9, 155 * WDstatus);
		terminal.flush();
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
		terminal.println(ConvertMillis() + " Domoticz connect");
		terminal.flush();
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
			terminal.println(ConvertMillis() + " Domoticz failed");
			terminal.flush();
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
			terminal.println(String(ConvertMillis()) + " Domoticz success");
			terminal.flush();
		}
	}
	else {
		// If you couldn't make a connection:
		dbSerialEMPrintln(ConvertMillis() + " Connection Domoticz failed (2)");
		if (BlynkServerUsed) {
			terminal.println(String(ConvertMillis()) + " Domoticz failed");
			terminal.flush();
		}
	}
	FunctionName = "idle";
}

// This method gets the data and prepares the global strings
// ---------------------------------------------------------
void DoIt() {
	FunctionName = __FUNCTION__;
	String TimeString;

	// make date stamp
	if (RTCUsed) {
		DateTime now = rtc.now();
		TimeString = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
	}
	else {
		TimeString = String(year()) + "-" + String(month()) + "-" + String(day()) + " " + String(hour()) + ":" + String(minute()) + ":" + String(second());
	}
	dbSerialEMPrintln(ConvertMillis() + " DoIt started... ");

	// get gas sensor ccs811 (ppm)
	//------------------------------------
	if (ccs811SensorInstalled) {
		// get and display ccs811 data
		get_ccs811();
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


	FunctionName = "idle";

	dbSerialEMPrintln(ConvertMillis() + " " + tsDataDomoticz);
	dbSerialEMPrintln(ConvertMillis() + " DoIt stopped... ");
}

void SendDataToBlynk() {
	if (BlynkServerUsed) {
		// Send data to Blynk
		FunctionName = __FUNCTION__;
		dbSerialEMPrintln(ConvertMillis() + " Start Blynk communication");
		//terminal.println(String(ConvertMillis()) + " Start Blynk communication");
		//terminal.flush();


		if (ccs811SensorInstalled) {
			Blynk.virtualWrite(V15, css811_CO2);
			Blynk.virtualWrite(V16, css811_TVOC);
		}
		// Software version
		Blynk.virtualWrite(V30, version);

		dbSerialEMPrintln(ConvertMillis() + " Stop Blynk communication");
		//terminal.println(String(ConvertMillis()) + " Stop Blynk communication");
		//terminal.flush();

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
	NTP.setInterval(15, 24*3600); // OPTIONAL. Set sync interval: once a day after first match
	dbSerialEMPrintln(ConvertMillis() + " Ready Ntp server");
}

void printDigits(int digits) {
	// utility function for digital clock display: prints preceding colon and leading 0

	dbSerialEMPrint(":");
	if (digits < 10)
		dbSerialEMPrint('0');
	dbSerialEMPrint(digits);
}

void digitalClockDisplay() {
	// digital clock display of the time
	dbSerialEMPrint(ConvertMillis() + " System time: "+ hour());
	printDigits(minute());
	printDigits(second());
	dbSerialEMPrint(" ");
	dbSerialEMPrint(day());
	dbSerialEMPrint(" ");
	dbSerialEMPrint(month());
	dbSerialEMPrint(" ");
	dbSerialEMPrint(year());
	dbSerialEMPrintln();
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

// This function will run every time Blynk connection is established
BLYNK_CONNECTED() {
	//get last data stored from server
	Blynk.syncVirtual(V40);
	Blynk.syncVirtual(V41);
}

void setup() {

	//Define sensor
	switch (Satellite) {
	case 1:
		satellite_name = "Satellite1";
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
		BlynkAuth = "76c706af72964df59752368de306395d"; // Blynk Token

		ccs811SensorInstalled = true;
		break;
	case 2:
		satellite_name = "Satellite2";
		satellite_network_pw = "##Sensor##";

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
	dbSerialEMPrintln(ConvertMillis() + " Satellite name: " + satellite_name);

	// prepare GPIO1 (flag start initialization (actual value will be restored via Blynk)


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
		while (Blynk.connect() == false); // wait until connection is there!
		terminal.println(ConvertMillis() + " Blynk configured");
		terminal.flush();
	}

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
	Wire.begin(SDA,SCL);
	Wire.setClock(400000);
	delay(1000);

	// Optional scan I2C ports
	check_if_exist_I2C();
	yield();
	//ccs811 sensor
	//-------------
	if (ccs811SensorInstalled) {
		dbSerialEMPrintln(ConvertMillis() + " CCS811 init");
		//It is recommended to check return status on .begin(), but it is not required.		
		CCS811Core::status returnCode = mySensor.begin();
		if (returnCode != CCS811Core::SENSOR_SUCCESS) {
			//ccs811SensorInstalled = false;
			dbSerialEMPrint(ConvertMillis() + " Could not find a valid ccs811 sensor, error: ");
			printDriverError(returnCode);
		}
	}

	// Init TaskSkeduler and add/enable tasks
	//---------------------------------------
	runner.init();
	dbSerialEMPrintln(ConvertMillis() + " Initialized scheduler");
	runner.addTask(t1);
	runner.addTask(t2);
	runner.addTask(t3);
	runner.addTask(t4);
	runner.addTask(t5);
	runner.addTask(t7);
	dbSerialEMPrintln(ConvertMillis() + " Tasks added to TaskScheduler");

	delay(1000);

	t1.enable();
	delay(500);
	t2.enable();
	delay(500);
	t3.enable();
	delay(500);
	t4.enable();
	delay(500);
	t5.enable();

	delay(600);
	t7.enable();
	dbSerialEMPrintln(ConvertMillis() + " Tasks enabled in TaskScheduler");

	// init Server
	//---------
	initServer();

	// init Ntp
	//---------
	initNtp();

	// put your setup code here, to run once:
	if (BlynkServerUsed) {
		terminal.println(ConvertMillis() + " Setup finished");
		terminal.flush();
	}
	GetRTCTime();
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

	if (req.indexOf("/gpio/0") != -1)
		val = 0;
	else if (req.indexOf("/gpio/1") != -1)
		val = 1;
	else {
		dbSerialEMPrintln("invalid request");
		client.stop();
		return;
	}



									  // Synchronize Blynk
	if (BlynkServerUsed) {
		Blynk.virtualWrite(V40, val);
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
	t_httpUpdate_return ret = ESPhttpUpdate.update("ota.my-net.nl", 80, "/ota_check.php", program + "_" + Satellite + "_" + version);
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

BLYNK_WRITE(V10)
{
	int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
								  // You can also use:
								  // String i = param.asStr();
								  // double d = param.asDouble();
	dbSerialEMPrintln(ConvertMillis() + " V10 value is: ");
	dbSerialEMPrintln(pinValue);

	if (pinValue == 0)
	{
		// Not used

	}
	else
	{
		if (BlynkServerUsed) {
			terminal.println(ConvertMillis() + " Forced restart " + FunctionName);
			terminal.flush();
		}
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(1000);
	}
}




BLYNK_READ(V11) // Widget in the app READs Virtal Pin V5 with the certain frequency
{
	int hours, min, sec;
	String timeString;

	// This command writes Arduino's uptime to Virtual Pin V11
	sec = millis() / 1000;
	min = sec / 60;
	hours = min / 60;
	timeString = (String(hours) + ":" + String(min % 60) + ":" + String(sec % 60)).c_str();
	Blynk.virtualWrite(V11, timeString);
}

void ICACHE_RAM_ATTR osWatch(void) // watchdog
{
	unsigned long t = millis();
	unsigned long last_run = abs(t - last_loop);
	if (last_run >= (OSWATCH_RESET_TIME * 1000)) {
		// save the hit here to eeprom or to rtc memory if needed
		if (BlynkServerUsed) {
			terminal.println(ConvertMillis() + " TimeOut from " + FunctionName);
			terminal.flush();
		}
		ESP.restart();  // normal reboot 
						//ESP.reset();  // hard reset
		delay(1000);
	}
}

void loop() {

	runner.execute();
	handle_server();
	// Break long sequence
	yield();

	if (BlynkServerUsed) {
		Blynk.run();
	}
	// reset watchdog
	last_loop = millis();
}
