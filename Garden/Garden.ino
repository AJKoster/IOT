/*
Name:		Garden.ino
Created:	12/08/2017 1:43:04 PM
Author:		Addy Koster
*/

// Monitor Software
// Uses WiFiManager (https://github.com/tzapu/WiFiManager)
// Uses Thinkspeak connection (Written by: Thomas Tongue)
// Date: August 12, 2017
// All put together by
// A.J. Koster

// Version
String version = "3.3";
String program = "Garden";
#define Sensor 1 // 1 = Garden, 2 = MyHouse, 3 = Test

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
const byte Heating = 15; // GPIO15 / pin D8
uint32_t old_heating_button_state;

//Tasks
Task t1(3600000L, TASK_FOREVER, &performOTA, &runner);	// 60 minutes
Task t2(30000L, TASK_FOREVER, &DoIt, &runner);	// 30 seconds
Task t3(420000L, TASK_FOREVER, &updateThingSpeak, &runner);	// 7 minutes
Task t4(360000L, TASK_FOREVER, &postData, &runner);	// 6 minutes
Task t5(300000L, TASK_FOREVER, &postDataDomoticz, &runner);	// 5 minutes
Task t6(1000L, TASK_FOREVER, &watchDog, &runner);	// 1 seconds beat
Task t7(60000L, TASK_FOREVER, &digitalClockDisplay, &runner); // 60 seconds beat

Task t9(30000L, TASK_FOREVER, &SendDataToBlynk, &runner);	// 30 seconds beat

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

const char* BridgeToCVAuth = ""; // Auth ID of receiving device
WidgetBridge bridgeCV(V50); //Initiating Bridge Widget on V50 of Device A (Garden)
WidgetBridge bridgeCV2(V60); //Initiating Bridge Widget on V60 of Device A (Home)


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


// DHT22
// Initialize DHT sensor
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01

// DHT library from Rob Tillaart
#include <dht.h> //http://playground.arduino.cc/Main/DHTLib https://forum.arduino.cc/index.php?topic=58531.180


// Default values
bool DHTSensorInstalled = false; // default not installed, in sensor definitions
#define DHT22_PIN 13
dht DHT;

struct
{
	uint32_t total;
	uint32_t ok;
	uint32_t crc_error;
	uint32_t time_out;
	uint32_t connect;
	uint32_t ack_l;
	uint32_t ack_h;
	uint32_t unknown;
} stat = { 0,0,0,0,0,0,0,0 };

float DHTDiscomfortIndex = 0.0;
float DHT_humidity = 0.0;
float DHT_temp = 0.0;
float DHT_dewpoint = 0.0;
float DHT_humidity_stat = 0;

int DHTDInumber = 0;
float DHThif = 0.0;  // Values read from sensor

					 //MQ135
#include <MQ135.h>
MQ135 gasSensor = MQ135(A0);
// Default values
bool AirQualitySensorInstalled = false; // default not AirQualitySensorInstalled but Battery power is measured.
float CO2_value = 0.0;
float RZeroValue = 0.0;
float CO2_value_Corr = 0.0;
float RZeroValue_Corr = 0.0;

// Include BME_280 library
#include <Bme280BoschWrapper.h>
Bme280BoschWrapper bme(true);
// Default values
bool BME280SensorInstalled = false; // default not installed, in sensor definitions
#define SEALEVELPRESSURE_HPA (1013.25)
float BME280_temp = 0.0;
float BME280_pressure = 0.0;
float BME280_humidity = 0.0;
float BME280_dewpoint = 0.0;
float BME280_humidity_stat = 0;
float BME280_pressure_for = 0;
float BME280_alti = 0.0;
int BME280Address;

// GY-30 - BH
#include <BH1750.h>
// Default values
bool BH1750SensorInstalled = false; // default not installed, in sensor definitions
uint16_t BH1750lux;
int BM1750Address;
BH1750 lightMeter;

//void configModeCallback (WiFiManager *myWiFiManager) {
//  
//    dbSerialEMPrintln("Entered config mode");
//    dbSerialEMPrintln(WiFi.softAPIP());
//    //if you used auto generated SSID, print it
//    dbSerialEMPrintln(myWiFiManager->getConfigPortalSSID());
//  
//}

// variables for sensor definition
const char* sensor_name;
const char* sensor_network_pw;

String writeAPIKey;
WiFiClient client;

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);

// EDIT: 'Server' address to match your domain
// MyHouse server
bool MyHouseServerUsed = false;
const char* MyHouseServer;

// Domoticz connection
bool DomoticzServerUsed = false;
const char* DomoticzServer; // IP address Domotics server
const char* DomoticzUserPass; // Token
const char* DomoticzId;

// ThingSpeak
bool ThinkspeakServerUsed = false;
const char* thingSpeakAddress = "api.thingspeak.com";
const int updateThingSpeakInterval = 300 * 1000;  // 300 sec, 300.000 msec.


int DiscomfortIndex(float temperature, float humidity) {

	float temp;

	// DiscomfortIndex
	temp = temperature - 0.55*(1 - humidity / 100)*(temperature - 14.5);
	dbSerialEMPrintln(ConvertMillis() + " DiscomfortIndexTemperature " + String(temp) + " *C");
	if (temp <= 24) {
		return 0;
	}
	else {
		if (temp <= 27) {
			return 1;
		}
		else {
			if (temp <= 29) {
				return 2;
			}
			else {
				return 3;
			}
		}
	}

}

// Domoticz calculation: humidity from above 70% wet, below 30 Dry, between 30 and 45 Normal, and 45 and 70 comfortable.

int HumidityStatus(float humidity) {
	if (humidity <= 30) {
		return 2;
	}
	else {
		if (humidity <= 45) {
			return 0;
		}
		else {
			if (humidity <= 70) {
				return 1;
			}
			else {
				return 3;
			}
		}
	}
}

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

// Get values from BH1750 sensor
// ----------------------------
void get_BH1750() {

	if (BH1750SensorInstalled) {
		BH1750lux = lightMeter.readLightLevel();
		dbSerialEMPrintln(ConvertMillis() + " Collected data from BH1750 sensor");
		dbSerialEMPrintln(ConvertMillis() + " Lux: " + String(BH1750lux) + " *lux");
	}
}

// Get values from DHT22 sensor
// ----------------------------
void get_DHT() {
	// Wait at least 2 seconds seconds between measurements.
	// if the difference between the current time and last time you read
	// the sensor is bigger than the interval you set, read the sensor
	// Works better than delay for things happening elsewhere also
	// Reading temperature for humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
	boolean DHT_succes = false;

	uint32_t start = micros();

	int chk = DHT.read22(DHT22_PIN);
	uint32_t stop = micros();
	stat.total++;

	// Error handling
	switch (chk)
	{
	case DHTLIB_OK:
		stat.ok++;
		dbSerialEMPrint("DHT OK,\t");
		DHT_succes = true;
		DHT_humidity = DHT.humidity;
		DHT_temp = DHT.temperature;
		DHT_dewpoint = dewPoint(DHT_temp, DHT_humidity, 1013.124);

		break;
	case DHTLIB_ERROR_CHECKSUM:
		stat.crc_error++;
		dbSerialEMPrint("DHT Checksum error,\t");
		break;
	case DHTLIB_ERROR_TIMEOUT:
		stat.time_out++;
		dbSerialEMPrint("DHT Time out error,\t");
		break;
	default:
		stat.unknown++;
		dbSerialEMPrint("DHT Unknown error,\t");
		break;
	}
	// DISPLAY DATA

	dbSerialEMPrint(ConvertMillis() + String(DHT_humidity, 1));
	dbSerialEMPrint(",\t");
	dbSerialEMPrint(String(DHT_temp, 1));
	dbSerialEMPrint(",\t");
	dbSerialEMPrint(String(DHT_dewpoint, 1));
	dbSerialEMPrint(",\t");
	dbSerialEMPrint(stop - start);
	dbSerialEMPrintln();

	if (stat.total % 20 == 0)
	{
		dbSerialEMPrintln("\nTOT\tOK\tCRC\tTO\tUNK");
		dbSerialEMPrint(ConvertMillis() + String(stat.total));
		dbSerialEMPrint("\t");
		dbSerialEMPrint(stat.ok);
		dbSerialEMPrint("\t");
		dbSerialEMPrint(stat.crc_error);
		dbSerialEMPrint("\t");
		dbSerialEMPrint(stat.time_out);
		dbSerialEMPrint("\t");
		dbSerialEMPrint(stat.connect);
		dbSerialEMPrint("\t");
		dbSerialEMPrint(stat.ack_l);
		dbSerialEMPrint("\t");
		dbSerialEMPrint(stat.ack_h);
		dbSerialEMPrint("\t");
		dbSerialEMPrint(stat.unknown);
		dbSerialEMPrintln("\n");
	}

	// DHT successfull read
	if (DHT_succes) {
		//Compute heat index in Celcius (the default)
		DHThif = DHT.computeHeatIndex(DHT_temp, DHT_humidity, false);
		DHT_humidity_stat = HumidityStatus(DHT_humidity);

		dbSerialEMPrintln(ConvertMillis() + " Collected data from DHT22 sensor");
		dbSerialEMPrintln(ConvertMillis() + " Temperature: " + String(DHT_temp) + " *C");
		dbSerialEMPrintln(ConvertMillis() + " Humidity: " + String(DHT_humidity) + " %RH");
		dbSerialEMPrintln(ConvertMillis() + " Dewpoint: " + String(DHT_dewpoint) + " *C");
		dbSerialEMPrintln(ConvertMillis() + " HeatIndex: " + String(DHThif) + " *C");
		dbSerialEMPrintln(ConvertMillis() + " DiscomfortIndex: " + String(DiscomfortIndex(DHT_temp, DHT_humidity)) + " -");
		dbSerialEMPrintln(ConvertMillis() + " DInumber: " + String(DHTDInumber) + " -");
	}
}

// Get airquality based on MQ135
//------------------------------
void get_AirQuality()
{

	if (AirQualitySensorInstalled) {

		CO2_value = gasSensor.getPPM();
		RZeroValue = gasSensor.getRZero();

		if (BME280SensorInstalled) {
			RZeroValue_Corr = gasSensor.getCorrectedRZero(BME280_temp, BME280_humidity);
			CO2_value_Corr = gasSensor.getCorrectedPPM(BME280_temp, BME280_humidity);
		}
		else if (DHTSensorInstalled) {
			RZeroValue_Corr = gasSensor.getCorrectedRZero(DHT_temp, DHT_humidity);
			CO2_value_Corr = gasSensor.getCorrectedPPM(DHT_temp, DHT_humidity);
		}
		else
		{
			RZeroValue_Corr = RZeroValue;
			CO2_value_Corr = CO2_value;
		}
		dbSerialEMPrintln(ConvertMillis() + " Collected data from MQ135 sensor");
		dbSerialEMPrintln(ConvertMillis() + " AnalogValue... " + String(3.3*analogRead(A0) / 1023.) + " Volt");
		dbSerialEMPrintln(ConvertMillis() + " MQ135 value... " + String(CO2_value) + " ppm");
		dbSerialEMPrintln(ConvertMillis() + " MQ135 value Corr... " + String(CO2_value_Corr) + " ppm");
		dbSerialEMPrintln(ConvertMillis() + " RZero... " + String(RZeroValue) + " kOhm");
		dbSerialEMPrintln(ConvertMillis() + " RZero Corr... " + String(RZeroValue_Corr) + " kOhm");
	}
}

void get_BME280() {

	dbSerialEMPrintln(ConvertMillis() + " Collected data from BME280 sensor");
	bool ok = bme.measure();
	if (ok) {
		// Valid measurment
		BME280_temp = bme.getTemperature() / 100.0F;
		BME280_pressure = bme.getPressure() / 100.0F;
		BME280_humidity = bme.getHumidity() / 1024.0F;
		BME280_humidity_stat = HumidityStatus(BME280_humidity);
		BME280_pressure_for = 0;

		//BME280_alti		= bme.readAltitude(SEALEVELPRESSURE_HPA);

		BME280_dewpoint = dewPoint(BME280_temp, BME280_humidity, BME280_pressure);

		dbSerialEMPrint(ConvertMillis() + " Temperature = ");
		dbSerialEMPrint(BME280_temp);
		dbSerialEMPrintln(" *C");
		dbSerialEMPrint(ConvertMillis() + " Pressure = ");
		dbSerialEMPrint(BME280_pressure);
		dbSerialEMPrintln(" mBar");
		//dbSerialEMPrint(ConvertMillis() + " Approx. Altitude = ");
		//dbSerialEMPrint(BME280_alti);
		//dbSerialEMPrintln(" m");
		dbSerialEMPrint(ConvertMillis() + " Humidity = ");
		dbSerialEMPrint(BME280_humidity);
		dbSerialEMPrintln(" %");
		dbSerialEMPrint(ConvertMillis() + " Dewpoint = ");
		dbSerialEMPrint(BME280_dewpoint);
		dbSerialEMPrintln(" *C");
	}
	else {
		dbSerialEMPrintln(ConvertMillis() + " Measurement BME280 sensor not valid");
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
	// update time display

	// send data to Blynk
	if (BlynkServerUsed) {
		Blynk.virtualWrite(V9, 155 * WDstatus);
		terminal.flush();
	}
}

// send data to ThingSpeak
// -----------------------
void updateThingSpeak()
{
	if (tsDataThingSpeak == "" || !ThinkspeakServerUsed)
	{
		dbSerialEMPrintln(ConvertMillis() + " No valid data for ThingSpeak...");
		return;
	}

	if (BlynkServerUsed) {
		terminal.println(ConvertMillis() + " ThingSp. connect");
	}
	FunctionName = __FUNCTION__;
	const int httpPort = 80;
	dbSerialEMPrintln(ConvertMillis() + " Connecting to ThingSpeak...");
	dbSerialEMPrintln(tsDataThingSpeak);

	if (!client.connect(thingSpeakAddress, httpPort)) {
		dbSerialEMPrintln(ConvertMillis() + " Connection to ThingSpeak Failed (1)...");
		if (BlynkServerUsed) {
			terminal.println(ConvertMillis() + " ThingSp. failed");
		}
		return;
	}

	client.print("POST /update HTTP/1.1\n");
	client.print("Host: api.thingspeak.com\n");
	client.print("Connection: close\n");
	client.print("X-THINGSPEAKAPIKEY: " + writeAPIKey + "\n");
	client.print("Content-Type: application/x-www-form-urlencoded\n");
	client.print("Content-Length: ");
	client.print(tsDataThingSpeak.length());
	client.print("\n\n");
	client.print(tsDataThingSpeak);

	// Break long sequence
	yield();

	if (client.connected()) {
		//delay(200);
		client.setTimeout(500);

		while (client.available()) {
			String line = client.readStringUntil('\r');
			dbSerialEMPrint(line);
		}
		dbSerialEMPrintln("\n" + ConvertMillis() + " Disconnecting ThingSpeak");
		client.stop();
		if (BlynkServerUsed) {
			terminal.println(ConvertMillis() + " ThingSp. success");
		}
	}
	else {
		dbSerialEMPrintln(ConvertMillis() + "Connection to ThingSpeak failed (2)...");
		if (BlynkServerUsed) {
			terminal.println(ConvertMillis() + " ThingSp. failed");
		}
	}
	FunctionName = "idle";
}

// This method makes a HTTP connection to the server and POSTs data
// ----------------------------------------------------------------
void postData()
{
	// Combine yourdatacolumn header (yourdata=) with the data recorded from your arduino
	// (yourarduinodata) and package them into the String yourdata which is what will be
	// sent in your POST request

	if (tsDataMyHouse == "" || !MyHouseServerUsed)
	{
		dbSerialEMPrintln(ConvertMillis() + " No valid data for MyHouse...");
		return;
	}

	if (BlynkServerUsed) {
		terminal.println(ConvertMillis() + " MyHouse connect");
	}
	FunctionName = __FUNCTION__;

	// If there's data and a successful connection, send the HTTP POST request
	dbSerialEMPrintln(ConvertMillis() + " Connecting to MyHouse...");
	dbSerialEMPrintln("POST /data/receive_data.php HTTP/1.1");
	dbSerialEMPrintln("Host: myhouse.my-net.nl");
	dbSerialEMPrintln("User-Agent: Arduino/1.0");
	dbSerialEMPrintln("Connection: close");
	dbSerialEMPrintln("Content-Type: application/x-www-form-urlencoded;");
	dbSerialEMPrint("Content-Length: ");
	dbSerialEMPrintln(tsDataMyHouse.length());
	dbSerialEMPrintln();
	dbSerialEMPrintln(tsDataMyHouse);

	if (!client.connect(MyHouseServer, 80)) {

		dbSerialEMPrintln(ConvertMillis() + " Connection to MyHouse Failed (1)...");
		if (BlynkServerUsed) {
			terminal.println(ConvertMillis() + " MyHouse failed");
		}
		return;
	}

	dbSerialEMPrintln(ConvertMillis() + " Connected to MyHouse ...");

	// Break long sequence
	yield();

	// If there's a successful connection, send the HTTP POST request
	// EDIT: The POST 'URL' to the location of your insert_mysql.php on your web-host
	client.println("POST /data/receive_data.php HTTP/1.1");
	// EDIT: 'Host' to match your domain
	client.println("Host: myhouse.my-net.nl");
	client.println("User-Agent: Arduino/1.0");
	client.println("Connection: close");
	client.println("Content-Type: application/x-www-form-urlencoded;");
	client.print("Content-Length: ");
	client.println(tsDataMyHouse.length());
	client.println();
	client.println(tsDataMyHouse);

	if (client.connected()) {
		//delay(200);
		client.setTimeout(500);
		while (client.available()) {
			String line = client.readStringUntil('\r');
			dbSerialEMPrint(line);
		}
		// Close connection
		dbSerialEMPrintln("\n" + ConvertMillis() + " Disconnecting MyHouse...");

		client.stop();
		if (BlynkServerUsed) {
			terminal.println(ConvertMillis() + " MyHouse success");
		}
	}
	else {
		// If you couldn't make a connection:
		dbSerialEMPrintln(ConvertMillis() + " Connection to MyHouse Failed(2)...");
		if (BlynkServerUsed) {
			terminal.print(ConvertMillis() + " MyHouse failed");
		}
	}
	FunctionName = "idle";
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
		}
	}
	else {
		// If you couldn't make a connection:
		dbSerialEMPrintln(ConvertMillis() + " Connection Domoticz failed (2)");
		if (BlynkServerUsed) {
			terminal.println(String(ConvertMillis()) + " Domoticz failed");
		}
	}
	FunctionName = "idle";
}

// This method gets the data and prepares the global strings
// ---------------------------------------------------------
void DoIt() {
	FunctionName = __FUNCTION__;
	String TimeString;
	float AnValue;

	// make date stamp
	if (RTCUsed) {
		DateTime now = rtc.now();
		TimeString = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
	}
	else {
		TimeString = String(year()) + "-" + String(month()) + "-" + String(day()) + " " + String(hour()) + ":" + String(minute()) + ":" + String(second());
	}
	dbSerialEMPrintln(ConvertMillis() + " DoIt started... ");

	// temperature / humidity and barometric pressure
	//-----------------------------------------------
	if (BME280SensorInstalled) {
		// get and display BME280 data
		get_BME280();
	}
	// get light intensity (LUX)
	//------------------------------------
	if (BH1750SensorInstalled) {
		// get and display BH1750 data
		get_BH1750();
	}

	// get gas sensor ccs811 (ppm)
	//------------------------------------
	if (ccs811SensorInstalled) {
		// get and display ccs811 data
		get_ccs811();
	}

	// get humidity and temperature DHT
	//------------------------------------
	if (DHTSensorInstalled) {
		// get and display DHT0 data
		get_DHT();
	}

	// get air quality or analoge in value
	//------------------------------------
	if (AirQualitySensorInstalled) {
		get_AirQuality();
		AnValue = CO2_value_Corr;
	}
	else
	{
		AnValue = 3.3*analogRead(A0) / 1023.0;
	}
	// Break long sequence
	yield();
	// Prepare strings if valid values are available
	// ---------------------------------------------
	// Send data to Domoticz
	if (DomoticzServerUsed) {
		if (DHTSensorInstalled) {
			tsDataDomoticz = "/json.htm?type=command&param=udevice&idx=" + String(DomoticzId) + "&nvalue=0&svalue=" + String(DHT_temp) + ";" + String(DHT_humidity) + ";" + String(DHT_humidity_stat);
		}
		if (BME280SensorInstalled) {
			tsDataDomoticz = "/json.htm?type=command&param=udevice&idx=" + String(DomoticzId) + "&nvalue=0&svalue=" + String(BME280_temp) + ";" + String(BME280_humidity) + ";" + String(BME280_humidity_stat) + ";" + String(BME280_pressure) + ";" + String(BME280_pressure_for);
		}
	}
	else {
		tsDataDomoticz = "";
	}
	// send data to ThingSpeak
	if (ThinkspeakServerUsed) {
		if (DHTSensorInstalled) {
			tsDataThingSpeak = "field1=" + String(DHT_temp) + "&field2=" + String(DHT_humidity) + "&field3=" + String(DHT_dewpoint) + "&field4=" + String(DHT_humidity_stat) + "&field5=" + "" + "&field6=" + String(AnValue);
		}
		if (BME280SensorInstalled) {
			tsDataThingSpeak = "field1=" + String(BME280_temp) + "&field2=" + String(BME280_humidity) + "&field3=" + String(BME280_dewpoint) + "&field4=" + String(BME280_humidity_stat) + "&field5=" + String(BME280_pressure) + "&field6=" + String(AnValue);
		}
		if (AirQualitySensorInstalled) {
			tsDataThingSpeak = tsDataThingSpeak + "&field7=" + String(CO2_value_Corr);
		}
		if (BH1750SensorInstalled) {
			tsDataThingSpeak = tsDataThingSpeak + "&field8=" + String(BH1750lux);
		}
	}
	else {
		tsDataThingSpeak = "";
	}
	//send data to MySql (myhouse.my-net.nl/data/receive_data.php)
	if (year() == 2013 || !MyHouseServerUsed) {
		tsDataMyHouse = "";
	}
	else {
		tsDataMyHouse = "sensor" + String(Sensor) + "result=" + "'" + String(BME280_temp) + "','" + String(BME280_humidity) + "','" + String(DHThif) + "','" + String(AnValue) + "','" + TimeString + "'";
	}

	//Display RTC time
	if (RTCUsed) {
		GetRTCTime();
	}

	// Update display with the latest values

	FunctionName = "idle";

	dbSerialEMPrintln(ConvertMillis() + " " + tsDataDomoticz);
	dbSerialEMPrintln(ConvertMillis() + " " + tsDataThingSpeak);
	dbSerialEMPrintln(ConvertMillis() + " " + tsDataMyHouse);
	dbSerialEMPrintln(ConvertMillis() + " DoIt stopped... ");
}

void SendDataToBlynk() {
	if (BlynkServerUsed) {
		// Send data to Blynk
		FunctionName = __FUNCTION__;
		dbSerialEMPrintln(ConvertMillis() + " Start Blynk communication");
		//terminal.println(String(ConvertMillis()) + " Start Blynk communication");

		if (BME280SensorInstalled) {
			Blynk.virtualWrite(V0, BME280_pressure);
			Blynk.virtualWrite(V1, BME280_temp);
			Blynk.virtualWrite(V2, BME280_humidity);
			Blynk.virtualWrite(V3, BME280_dewpoint);

			// Update bridges
			switch (Sensor) {
			case 1: // Garden
				bridgeCV.virtualWrite(V50, BME280_temp);
				Blynk.virtualWrite(V50, BME280_temp);
				if (BlynkServerUsed) {
					terminal.println(String(ConvertMillis()) + " V50 send");
				}
				break;
			case 2: // MyHouse
				bridgeCV2.virtualWrite(V60, BME280_temp);
				Blynk.virtualWrite(V60, BME280_temp);
				if (BlynkServerUsed) {
					terminal.println(String(ConvertMillis()) + " V60 send");
				}
				break;
			}
		}
		if (BH1750SensorInstalled) {
			Blynk.virtualWrite(V4, BH1750lux);
		}
		if (AirQualitySensorInstalled) {
			Blynk.virtualWrite(V5, CO2_value);
			Blynk.virtualWrite(V6, RZeroValue);
			Blynk.virtualWrite(V7, CO2_value_Corr);
			Blynk.virtualWrite(V8, RZeroValue_Corr);
		}
		if (ccs811SensorInstalled) {
			Blynk.virtualWrite(V15, css811_CO2);
			Blynk.virtualWrite(V16, css811_TVOC);
		}

		if (DHTSensorInstalled) {
			Blynk.virtualWrite(V21, DHT_temp);
			Blynk.virtualWrite(V22, DHT_humidity);
			Blynk.virtualWrite(V23, DHT_dewpoint);
		}
		// Software version
		Blynk.virtualWrite(V30, version);

		dbSerialEMPrintln(ConvertMillis() + " Stop Blynk communication");
		//terminal.println(String(ConvertMillis()) + " Stop Blynk communication");

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

void digitalClockDisplay() {
	// digital clock display of the time
	dbSerialEMPrint(ConvertMillis() + " System time: " + hour());
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
	/* Set the baudrate which is for debug communicate */
	dbSerialEMBegin(115200);
	delay(2000);
	dbSerialEMPrintln(ConvertMillis() + " Serial Setup done");
}



// This function will run every time Blynk connection is established
BLYNK_CONNECTED() {
	//get last data stored from server
	Blynk.syncVirtual(V40);

	// set-up bridges to others
	if (BridgeToCVAuth != "") {
		bridgeCV.setAuthToken(BridgeToCVAuth); // Token of CV
		bridgeCV2.setAuthToken(BridgeToCVAuth); // Token of CV
	}
}

void setup() {

	//Define sensor
	switch (Sensor) {
	case 1: // Garden
		sensor_name = "Sensor1";
		sensor_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = false;

		// MyHouse server
		MyHouseServerUsed = true;
		MyHouseServer = "myhouse.my-net.nl"; // This could also be 192.168.1.18/~me if you are running a server on your computer on a local network.

											 // Thinkspeak key
		ThinkspeakServerUsed = true;
		writeAPIKey = "A08IB7CB5AN9CS3S"; // Sensor 1 - outside

										  // Domoticz
		DomoticzServerUsed = true;
		DomoticzServer = "192.168.2.15"; // adress Domotics server
		DomoticzUserPass = "YWRtaW46IyNEb21vdGljeiMj";
		DomoticzId = "17";

		// Blynk
		BlynkServerUsed = true;
		BlynkServer = "192.168.2.15"; // IP address Blynk server
		BlynkAuth = "73fe74168f2d43fea397b6bc70ed12ed"; // Blynk Token
														//		BridgeToCVAuth = "06a8e6ec53b34c15a8d3dc7a45f8ed6b"; // Token of CV on user house@my-net.nl
		BridgeToCVAuth = "af26c5fce7bf4d979bb3da1f1e6f7a34"; // Token of CV on user aj.koster@my-net.nl
															 // Sensors
		AirQualitySensorInstalled = false;
		BME280SensorInstalled = true;
		BME280Address = 0x76;
		BH1750SensorInstalled = false;
		DHTSensorInstalled = false;
		ccs811SensorInstalled = false;
		break;
	case 2: // Thermostate home
		sensor_name = "Sensor2";
		sensor_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = true;

		// MyHouse server
		MyHouseServerUsed = true;
		MyHouseServer = "myhouse.my-net.nl"; // This could also be 192.168.1.18/~me if you are running a server on your computer on a local network.

											 // Thinkspeak key
		ThinkspeakServerUsed = true;
		writeAPIKey = "LGA9FX28P8A4DODE"; // Sensor 2 - inside

										  // Domoticz
		DomoticzServerUsed = true;
		DomoticzServer = "192.168.2.15"; // IP address Domotics server
		DomoticzUserPass = "YWRtaW46IyNEb21vdGljeiMj";
		DomoticzId = "21";

		// Blynk
		BlynkServerUsed = true;
		BlynkServer = "192.168.2.15"; // IP address Blynk server
		BlynkAuth = "ad20f00456024a97a309d89facd32170"; // Blynk Token
														//		BridgeToCVAuth = "06a8e6ec53b34c15a8d3dc7a45f8ed6b"; // Token of CV on user house@my-net.nl
		BridgeToCVAuth = "af26c5fce7bf4d979bb3da1f1e6f7a34"; // Token of CV on user aj.koster@my-net.nl

															 // Sensors
		AirQualitySensorInstalled = true;
		BME280SensorInstalled = true;
		BME280Address = 0x76;
		BH1750SensorInstalled = true;
		BM1750Address = 0x23;
		DHTSensorInstalled = false;
		ccs811SensorInstalled = false;
		break;
	case 3: // Testboard
		sensor_name = "Sensor3";
		sensor_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = true;

		// MyHouse server
		MyHouseServerUsed = false;
		MyHouseServer = ""; // This could also be 192.168.1.18/~me if you are running a server on your computer on a local network.

							// Thinkspeak key
		ThinkspeakServerUsed = false;
		writeAPIKey = ""; // Sensor  

						  // Domoticz
		DomoticzServerUsed = false;
		DomoticzServer = ""; // IP address Domotics server
		DomoticzUserPass = "";
		DomoticzId = "";

		//Blynk
		BlynkServerUsed = true;
		BlynkServer = "192.168.2.15"; // IP address Blynk server
		BlynkAuth = "76c706af72964df59752368de306395d"; // Blynk Token

														// Sensors
		AirQualitySensorInstalled = false;
		BME280SensorInstalled = false;
		BME280Address = 0x76;
		BH1750SensorInstalled = false;
		BM1750Address = 0x23;
		DHTSensorInstalled = false;
		ccs811SensorInstalled = false;
		break;
	default:
		sensor_name = "SensorDefault";
		sensor_network_pw = "##Sensor##";

		// RTC available
		RTCUsed = false;

		// MyHouse server
		MyHouseServerUsed = false;
		MyHouseServer = ""; // This could also be 192.168.1.18/~me if you are running a server on your computer on a local network.

							// Thinkspeak key
		ThinkspeakServerUsed = false;
		writeAPIKey = ""; // Sensor 

						  // Domoticz
		DomoticzServerUsed = false;
		DomoticzServer = ""; // IP address Domotics server
		DomoticzUserPass = "";
		DomoticzId = "";

		//Blynk
		BlynkServerUsed = false;
		BlynkServer = ""; // IP address Blynk server		
		BlynkAuth = ""; // Blynk Token

						// Sensors
		DHTSensorInstalled = false;
		AirQualitySensorInstalled = false;
		BME280SensorInstalled = false;
		BH1750SensorInstalled = false;
		ccs811SensorInstalled = false;
		break;
	}
	// Set-up serial communication for debugging
	setup_serial_debug();

	// Hello message
	//--------------
	dbSerialEMPrintln(ConvertMillis() + " Program: " + program);
	dbSerialEMPrintln(ConvertMillis() + " Version: " + version);
	dbSerialEMPrintln(ConvertMillis() + " Sensor name: " + sensor_name);

	// prepare GPIO1 (flag start initialization (actual value will be restored via Blynk)
	pinMode(Heating, OUTPUT);
	//digitalWrite(Heating, 0);

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

	if (!wifiManager.autoConnect(sensor_name, sensor_network_pw)) {
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
	Wire.begin(SDA, SCL);
	Wire.setClock(400000);
	delay(1000);

	// Optional scan I2C ports
	check_if_exist_I2C();
	yield();

	// BME280 sensor
	//--------------
	if (BME280SensorInstalled) {
		if (!bme.beginI2C(BME280Address)) {
			BME280SensorInstalled = false;
			dbSerialEMPrintln(ConvertMillis() + " Could not find a valid BME280 sensor, check wiring!");
		}
	}

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

	// BM1750 sensor
	//--------------
	if (BH1750SensorInstalled) {
		lightMeter.set_I2Caddress(BM1750Address);
		lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
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
	runner.addTask(t6);
	runner.addTask(t7);

	runner.addTask(t9);
	dbSerialEMPrintln(ConvertMillis() + " Tasks added to TaskScheduler");

	delay(1000);

	t1.enableDelayed(500);  // performOTA - 60 min
	t2.enableDelayed(1000); // DoIt - 30 sec
	t3.enableDelayed(1500); // UpdateThingSpeak - 7 min
	t4.enableDelayed(500);  // postData - 6 min
	t5.enableDelayed(2500); // postDataDomoticz - 5 min
	t6.enableDelayed(4000); // watchDog - 1 sec
	t7.enableDelayed(3000); // clockvalue - 5 sec

	t9.enableDelayed(4500); // SendDataToBlyn - 30 sec

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

	// Set GPIO15 according to the request
	digitalWrite(Heating, val); // pin D8 / GPIO15

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


/*-----( Declare User-written Functions )-----*/
// dewPoint function NOAA
// reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
// reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
//
// Temperature in degree C, humidity in %RH, pressure in mBar.
double dewPoint(double celsius, double humidity, double pressure)
{
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + celsius);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / RATIO))) - 1);
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1);
	RHS += log10(pressure); // NB = 1013 = air pressure, which you can substitute from BME280 if you want to, but it needs to be converted to suit your specific altitude i think?

							// factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;

	// (2) DEWPOINT = F(Vapor Pressure)
	double T = log(VP / 0.61078); // temp var
	return (241.88 * T) / (17.558 - T);
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

	if (pinValue == 0)
	{
		// Set GPIO15 according to the request
		digitalWrite(Heating, 0); // pin D8 / GPIO15
		terminal.println(ConvertMillis() + " Heating Off ");
		terminal.flush();
	}
	else
	{
		// Set GPIO15 according to the request
		digitalWrite(Heating, 1); // pin D8 / GPIO15
		terminal.println(ConvertMillis() + " Heating On ");
		terminal.flush();
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
