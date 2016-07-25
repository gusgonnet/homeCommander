//The MIT License (MIT)
//Copyright (c) 2016 Gustavo Gonnet
//
//Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
// is furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all copies
// or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// github: https://github.com/gusgonnet/homeCommander
//
// This project contains the following three projects in one:
// hackster: https://www.hackster.io/gusgonnet/garage-commander-30383a
// hackster: https://www.hackster.io/gusgonnet/water-detection-system-227b08
// hackster: https://www.hackster.io/gusgonnet/pool-temperature-monitor-5331f2


// Pointers:
// garage related stuff starts with garage_
// flood sensor related stuff starts with flood_
// pool temperature monitor starts with pool_

// IO mapping
// D0 : relay: garage_BUTTON - this output is connected to the garage wall button or the garage remote
// D1 : relay
// D2 : relay
// D3 : relay
// D4 : garage_CLOSE : high means contact open (garage door opened), low means contact closed (garage door closed)
// D5 : garage_OPEN : high means contact open (garage door opened), low means contact closed (garage door closed)
// D6 : DHT dryer
// D7 : flood_WATER_SENSOR

// A0 : pool_THERMISTOR
// A1~A7 : not used

#include "application.h"
#include "elapsedMillis.h"
#include "PietteTech_DHT.h"
#include "blynk.h"
#include "blynkAuthToken.h"

#define APP_NAME "Home Commander"
String VERSION = "Version 0.59";

/*******************************************************************************
 * changes in version 0.51:
       * added dryer notifications project with DHT22
 * changes in version 0.52:
       * fixed dryer code
 * changes in version 0.53:
             * changed resetDryer to setDryer
 * changes in version 0.54:
             * PUSHBULLET_NOTIF renamed to PUSHBULLET_NOTIF_PERSONAL
             * removing consecutive publish since they might not work back to back
              source: https://docs.particle.io/reference/firmware/photon/#particle-publish-
              NOTE: Currently, a device can publish at rate of about 1 event/sec,
              with bursts of up to 4 allowed in 1 second. Back to back burst
              of 4 messages will take 4 seconds to recover.
 * changes in version 0.55:
              * adding alarm after max on time of dryer in case the clothes are still dry
 * changes in version 0.56:
              * adding blynk support (for allowing my family to see/control this project)
              * adding time in notifications
* changes in version 0.57:
              * removing initial dryer notif
* changes in version 0.58:
              * adding opening/closing the garage from blynk
              * updating the blynk cloud periodically
* changes in version 0.59:
              * Notifying the user that the "Pool is ready!" when temperature
                 reaches POOL_TARGET_TEMP

*******************************************************************************/

#define PUSHBULLET_NOTIF_HOME "pushbulletHOME"         //-> family group in pushbullet
#define PUSHBULLET_NOTIF_PERSONAL "pushbulletPERSONAL" //-> only my phone
#define AWS_EMAIL "awsEmail"
const int TIME_ZONE = -4;

/*******************************************************************************
 DHT sensor
*******************************************************************************/
#define DHTTYPE  DHT22                // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN   6                    // Digital pin for communications
#define DHT_SAMPLE_INTERVAL   30000   // Sample dryer every 30 seconds
void dht_wrapper(); // must be declared before the lib initialization
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);
bool bDHTstarted;       // flag to indicate we started acquisition
elapsedMillis dhtSampleInterval;
int n;                  // counter
unsigned int DHTnextSampleTime;  // Next time we want to start sample -> BORRAR

//dryer begin
//String to store the sensor temp
char resultstr[64];
//String to store the sensor humidity
char humiditystr[64];
bool dryer_on = false;
String dryer_stat = "";
int humidity_samples_below_10 = 0;
float currentTemp = 20.0;
float currentHumidity = 0.0;
//temperature related variables - to be exposed in the cloud
String currentTempString = String(currentTemp); //String to store the sensor's temp so it can be exposed
String currentHumidityString = String(currentHumidity); //String to store the sensor's humidity so it can be exposed

//milliseconds for the max time the dryer can be on
// in my case, my dryer goes up to 99 minutes
// this indirect method will be used to raise an alarm if the clothes are still not fully dry
//  after this time has elapsed
#define DRYER_MAX_TIMER 5940000
elapsedMillis dryerMaxTimer;
//dryer end

//pool begin
#include <math.h>
//pool end

//garage begin
#define GARAGE_READ_INTERVAL 1000
#define GARAGE_OPEN "open"
#define GARAGE_CLOSED "closed"
#define GARAGE_OPENING "opening"
#define GARAGE_CLOSING "closing"
#define GARAGE_NOTIF "GARAGE"
unsigned long garage_interval = 0;
int garage_BUTTON = D0;
int garage_CLOSE = D4;
int garage_OPEN = D5;
String garage_status_string = "unknown";
//garage end

//flood detection begin
//this reads the flood sensor every 2 seconds
#define FLOOD_READ_INTERVAL 2000

//this defines the frequency of the notifications sent to the user
#define FLOOD_FIRST_ALARM 10000 //10 seconds
#define FLOOD_SECOND_ALARM 60000 //1 minute
#define FLOOD_THIRD_ALARM 300000 //5 minutes
#define FLOOD_FOURTH_ALARM 900000 //15 minutes
#define FLOOD_FIFTH_ALARM 3600000 //1 hour
#define FLOOD_SIXTH_ALARM 14400000 //4 hours - and every 4 hours ever after, until the situation is rectified (ie no more water is detected)

int flood_SENSOR = D7;
elapsedMillis flood_timer;
elapsedMillis flood_alarm_timer;

int flood_alarms_array[6]={FLOOD_FIRST_ALARM, FLOOD_SECOND_ALARM, FLOOD_THIRD_ALARM, FLOOD_FOURTH_ALARM, FLOOD_FIFTH_ALARM, FLOOD_SIXTH_ALARM};
int flood_alarm_index = 0;
bool flood_detected = false;
unsigned long flood_next_alarm = 0;
//flood detection end

//pool begin
// this is the thermistor used
// https://www.adafruit.com/products/372
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

#define POOL_READ_INTERVAL 60000
#define POOL_NOTIF "POOL"

unsigned long pool_interval = 0;
int samples[NUMSAMPLES];
int pool_THERMISTOR = A0;
//this is coming from http://www.instructables.com/id/Datalogging-with-Spark-Core-Google-Drive/?ALLSTEPS
char pool_tmp[64]; //String to store the sensor data
char pool_temperature_ifttt[64];

float poolCurrentTemp;
#define POOL_TARGET_TEMP 29
#define POOL_HYST_TEMP 28
bool poolReadyAlreadyNotified = false;

//by default, we'll display the temperature in degrees celsius, but if you prefer farenheit please set this to true
bool useFahrenheit = false;
//pool end

/*******************************************************************************
 Here you decide if you want to use Blynk or not
 Your blynk token goes in another file to avoid sharing it by mistake
  (like I just did in my last commit)
 The file containing your token has to be named blynkAuthToken.h and it should contain
 something like this:
  #define BLYNK_AUTH_TOKEN "1234567890123456789012345678901234567890"
 replace with your project auth token (the blynk app will give you one)
*******************************************************************************/
#define USE_BLYNK "yes"
char auth[] = BLYNK_AUTH_TOKEN;
WidgetLED dryerStatusLed(V22); //register led to virtual pin 22
#define BLYNK_GARAGE_BUTTON V30
#define BLYNK_GARAGE_BUTTON_DEBOUNCE 1000
elapsedMillis blynkGarageButtonDebounce;
bool blynkGarageButtonPressed = false;
bool ButtonDown = false;

//this defines how often the readings are sent to the blynk cloud (millisecs)
#define BLYNK_STORE_INTERVAL 5000
elapsedMillis blynkStoreInterval;

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup() {

  //publish startup message with firmware version
  Particle.publish(APP_NAME, VERSION, 60, PRIVATE);

  Time.zone(TIME_ZONE);

  //garage begin
  pinMode(garage_BUTTON, OUTPUT);
  pinMode(garage_OPEN, INPUT_PULLUP);
  pinMode(garage_CLOSE, INPUT_PULLUP);

  //declare cloud functions
  //https://docs.particle.io/reference/firmware/photon/#particle-function-
  //Currently the application supports the creation of up to 4 different cloud functions.
  //The length of the funcKey is limited to a max of 12 characters.
  // If you declare a function name longer than 12 characters the function will not be registered.
  bool success = Particle.function("garage_open", garage_open);
  if (not success) {
      Particle.publish("ERROR", "Failed to register function garage_open", 60, PRIVATE);
  }

  success = Particle.function("garage_stat", garage_stat);
  if (not success) {
      Particle.publish("ERROR", "Failed to register function garage_stat", 60, PRIVATE);
  }
  //garage end

  //flood detection begin
  pinMode(flood_SENSOR, INPUT_PULLUP);
  //flood detection end

  //pool begin
  pool_interval = 0;
  pinMode(pool_THERMISTOR, INPUT);

  //declare cloud variables
  //https://docs.particle.io/reference/firmware/photon/#particle-variable-
  //Currently, up to 10 cloud variables may be defined and each variable name is limited to a maximum of 12 characters
  if (Particle.variable("pool_tmp", pool_tmp, STRING)==false) {
    Particle.publish(APP_NAME, "ERROR: Failed to register variable pool_tmp", 60, PRIVATE);
  }

  success = Particle.function("pool_get_tmp", pool_get_tmp);
  if (not success) {
      Particle.publish("ERROR", "Failed to register function pool_get_tmp", 60, PRIVATE);
  }
  //pool end

  //dryer begin
  // Start the first sample immediately
  DHTnextSampleTime = 0;
  if (Particle.variable("currentTemp", currentTempString)==false) {
    Particle.publish(APP_NAME, "ERROR: Failed to register variable currentTemp", 60, PRIVATE);
  }
  if (Particle.variable("humidity", currentHumidityString)==false) {
    Particle.publish(APP_NAME, "ERROR: Failed to register variable humidity", 60, PRIVATE);
  }
  if (Particle.variable("dryer_stat", dryer_stat)==false) {
    Particle.publish(APP_NAME, "ERROR: Failed to register variable dryer_stat", 60, PRIVATE);
  }
  success = Particle.function("setDryer", setDryer);
  if (not success) {
    Particle.publish("ERROR", "Failed to register function setDryer", 60, PRIVATE);
  }
  //dryer end

  if (USE_BLYNK == "yes") {
    //init Blynk
    Blynk.begin(auth);
  }

}

// This wrapper is in charge of calling the DHT sensor lib
void dht_wrapper() { DHT.isrCallback(); }

/*******************************************************************************
 * Function Name  : loop
 * Description    : this function runs all the time
 *******************************************************************************/
void loop() {

  if (USE_BLYNK == "yes") {
    //all the Blynk magic happens here
    Blynk.run();
  }

  flood_check();
  if ( flood_detected ) {
    flood_notify_user();
  }

  //pool temp
  if( (millis() - pool_interval >= POOL_READ_INTERVAL) or (pool_interval==0) ) {
    pool_calculate_current_temp();
    pool_notifyTargetTempReached();
    pool_interval = millis(); //update to current millis()
  }

  //read garage status every now and then
  if( millis() - garage_interval >= GARAGE_READ_INTERVAL ) {
    garage_read();
    garage_interval = millis();
  }

  dryer_status();

  //only open the garage in the case the button is pressed for more than 1 second (BLYNK_GARAGE_BUTTON_DEBOUNCE)
  if ( blynkGarageButtonPressed and (blynkGarageButtonDebounce > BLYNK_GARAGE_BUTTON_DEBOUNCE) ) {
    garage_open("dummy");
    blynkGarageButtonPressed = false;
  }

  //publish readings to the blynk server every minute so the History Graph gets updated
  // even when the blynk app is not on (running) in the users phone
  updateBlynkCloud();

}

/*******************************************************************************
 * Function Name  : garage_open
 * Description    : garage_BUTTON goes up for one second
 * Return         : 0
 *******************************************************************************/
int garage_open(String args)
{
    //Particle.publish(GARAGE_NOTIF, "garage_open triggered", 60, PRIVATE);
    digitalWrite(garage_BUTTON, HIGH);
    delay(1000);
    digitalWrite(garage_BUTTON, LOW);

    return 0;
}

/*******************************************************************************
 * Function Name  : garage_read()
 * Description    : reads and publishes the status of the garage, intended for using it with a service like ifttt
 * Return         : 0
 *******************************************************************************/
int garage_read()
{
    int open = digitalRead(garage_OPEN);
    int closed = digitalRead(garage_CLOSE);
    String previous_garage_status_string = garage_status_string;

    //input goes low when the reed switch is activated
    if ( not closed ) {
        garage_status_string = GARAGE_CLOSED;
    }

    //input goes low when the reed switch is activated
    if ( not open ) {
        garage_status_string = GARAGE_OPEN;
    }

    //if both inputs are high, it means that the garage is moving
    // so if it was open, we believe it's closing now
    // and if it was closed, we believe it's opening now
    if ( open and closed ) {
        if ( previous_garage_status_string == GARAGE_OPEN ) {
            garage_status_string=GARAGE_CLOSING;
        }
        if ( previous_garage_status_string == GARAGE_CLOSED ) {
            garage_status_string=GARAGE_OPENING;
        }
    }

    //if status of the garage changed from last scan, publish the new status
    if ( previous_garage_status_string != garage_status_string ) {
        Particle.publish(PUSHBULLET_NOTIF_PERSONAL, "Your garage door is " + garage_status_string + getTime(), 60, PRIVATE);
    }

    return 0;
}

/*******************************************************************************
 * Function Name  : garage_stat  // function name cannot be longer than 12 chars otherwise it wont be registered!
 * Description    : reads and publishes the status of the garage, intended for using it with a service like ifttt
 * Return         : 0
 *******************************************************************************/
int garage_stat(String args)
{
    int opened = digitalRead(garage_OPEN);
    int closed = digitalRead(garage_CLOSE);

    if ( not closed ) {
        garage_status_string = GARAGE_CLOSED;
    }
    if ( not opened ) {
        garage_status_string = GARAGE_OPEN;
    }

   Particle.publish(PUSHBULLET_NOTIF_PERSONAL, "Your garage door is " + garage_status_string + getTime(), 60, PRIVATE);

    return 0;
}

/*******************************************************************************
 * Function Name  : pool_notifyTargetTempReached
 * Description    : notify the user that the pool is ready for jumping in!
 * Return         : none
 *******************************************************************************/
void pool_notifyTargetTempReached()
{
  if ( (not poolReadyAlreadyNotified ) and ( poolCurrentTemp > POOL_TARGET_TEMP ) ) {
    Particle.publish(PUSHBULLET_NOTIF_HOME, "Pool is ready! (" + String(POOL_TARGET_TEMP) + "°C)", 60, PRIVATE);
    poolReadyAlreadyNotified = true;
  }

  //now reset notif if temp goes lower than POOL_HYST_TEMP
  if ( poolCurrentTemp < POOL_HYST_TEMP ){
    poolReadyAlreadyNotified = false;
  }

}

/*******************************************************************************
 * Function Name  : pool_calculate_current_temp
 * Description    : read the value of the thermistor, convert it to degrees and store it in pool_tmp
 * Return         : 0
 *******************************************************************************/
int pool_calculate_current_temp()
{
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
    samples[i] = analogRead(pool_THERMISTOR);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = (4095 / average)  - 1;
  average = SERIESRESISTOR / average;


  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  // Convert Celsius to Fahrenheit
  // source: http://playground.arduino.cc/ComponentLib/Thermistor2#TheSimpleCode
  if (useFahrenheit) {
    steinhart = (steinhart * 9.0)/ 5.0 + 32.0;
  }

  //assign to global variable
  poolCurrentTemp = steinhart;

  int steinhart1 = (steinhart - (int)steinhart) * 100;
  // for negative temperatures
  steinhart1 = abs(steinhart1);

  char currentPoolTempChar[32];
  sprintf(currentPoolTempChar,"%0d.%d", (int)steinhart, steinhart1);
  String currentPoolTempString = String(currentPoolTempChar);

  //publish readings
  Particle.publish(APP_NAME, "Pool temperature: " + currentPoolTempString + "°C", 60, PRIVATE);

  char tempInChar[32];
  sprintf(tempInChar,"%0d.%d", (int)steinhart, steinhart1);

  //Write temperature to string, google sheets will get this variable
  sprintf(pool_tmp, "{\"t\":%s}", tempInChar);

  //this variable will be published by function status()
  sprintf(pool_temperature_ifttt, "%s", tempInChar);

  return 0;
}

/*******************************************************************************
 * Function Name  : pool_get_tmp // function name cannot be longer than 12 chars otherwise it wont be registered!
 * Description    : publish the pool temp for services like ifttt
 * Return         : 0
 *******************************************************************************/
int pool_get_tmp(String args)
{
  Particle.publish(PUSHBULLET_NOTIF_PERSONAL, "Your pool is at " + String(pool_temperature_ifttt) + " degrees", 60, PRIVATE);
  return 0;
}

/*******************************************************************************
 * Function Name  : flood_check
 * Description    : check water leak sensor at FLOOD_READ_INTERVAL, turns on led on D7 and raises alarm if water is detected
 * Return         : 0
 *******************************************************************************/
int flood_check()
{
    if (flood_timer < FLOOD_READ_INTERVAL) {
        return 0;
    }

    //time is up, so reset timer
    flood_timer = 0;

    if (not digitalRead(flood_SENSOR)) {

        //if flood is already detected, no need to do anything, since an alarm will be fired
        if (flood_detected){
            return 0;
        }

        flood_detected = true;

        //reset alarm timer
        flood_alarm_timer = 0;

        //set next alarm
        flood_alarm_index = 0;
        flood_next_alarm = flood_alarms_array[0];

    } else {
        flood_detected = false;
    }
    return 0;
}

/*******************************************************************************
 * Function Name  : flood_notify_user
 * Description    : will fire notifications to user at scheduled intervals
 * Return         : 0
 *******************************************************************************/
int flood_notify_user()
{

    if (flood_alarm_timer < flood_next_alarm) {
        return 0;
    }


    //time is up, so reset timer
    flood_alarm_timer = 0;

    //set next alarm or just keep current one if there are no more alarms to set
    if (flood_alarm_index < arraySize(flood_alarms_array)-1) {
        flood_alarm_index = flood_alarm_index + 1;
        flood_next_alarm = flood_alarms_array[flood_alarm_index];
    }

  //send an alarm to user (this one goes to pushbullet servers)
  Particle.publish(PUSHBULLET_NOTIF_PERSONAL, "Flood detected!" + getTime(), 60, PRIVATE);

  return 0;
}


/*******************************************************************************
 * Function Name  : setDryer
 * Description    : call this function to set the status of the dyer
                     when the algorithm gets stuck (For instance, when you manually stopped the dryer cycle)
                     when you want to set the cycle on (For instance, when the dryer was on and you updated the firmware)
                    you could call this function with the DO Button, the blynk app, the Porter app or even Particle dev
 * Return         : 0
 *******************************************************************************/
int setDryer(String status) {

  //update the fan status only in the case the status is on or off
  if ( status == "on" ) {
    dryer_on = true;
    dryer_stat = "dryer_on";
    Particle.publish(PUSHBULLET_NOTIF_PERSONAL, "Dryer on" + getTime(), 60, PRIVATE);

    if (USE_BLYNK == "yes") {
      dryerStatusLed.on();
    }

    return 0;
  }

  if ( status == "off" ) {
    dryer_on = false;
    dryer_stat = "dryer_off";
    Particle.publish(PUSHBULLET_NOTIF_PERSONAL, "Dryer off" + getTime(), 60, PRIVATE);

    if (USE_BLYNK == "yes") {
      dryerStatusLed.off();
    }

    return 0;
  }

  return -1;
}

/*******************************************************************************
 * Function Name  : dryer_status
 * Description    : reads the temperature of the DHT22 sensor at every DHT_SAMPLE_INTERVAL
 * Return         : 0
 *******************************************************************************/
int dryer_status() {

  //time is up? no, then come back later
  if (dhtSampleInterval < DHT_SAMPLE_INTERVAL) {
   return 0;
  }

  //time is up, reset timer
  dhtSampleInterval = 0;

  // start the sample
  if (!bDHTstarted) {
    DHT.acquireAndWait(5);
    bDHTstarted = true;
  }

  //still acquiring sample? go away
  if (DHT.acquiring()) {
    return 0;
  }

  //sample acquired - go ahead and store temperature and humidity in internal variables
  publishTemperature( (float)DHT.getCelsius(), (float)DHT.getHumidity() );

  //reset the sample flag so we can take another
  bDHTstarted = false;

  //if humidity goes above 50% then we believe the dryer has just started a cycle
  if ( ( not dryer_on ) and ( currentHumidity > 50 ) and ( currentTemp > 30 ) ) {
    dryer_on = true;
    humidity_samples_below_10 = 0;
    //Particle.publish(PUSHBULLET_NOTIF_HOME, "Starting drying cycle" + getTime(), 60, PRIVATE);
    // Particle.publish(AWS_EMAIL, "Starting drying cycle", 60, PRIVATE);

    if (USE_BLYNK == "yes") {
      dryerStatusLed.on();
    }

    //this fires up the max time the dryer can be on
    dryerMaxTimer = 0;
  }

  //if humidity goes below 10% and temperature goes over 50 degrees for a number of samples
  // we believe the clothes are dry
  // modify these parameters if you want to dry even more your clothes
  // example: to have clothes drier modify to "if ( dryer_on and ( currentHumidity < 8) and ( currentTemp > 50 ) ) {"
  if ( dryer_on and ( currentHumidity < 10) and ( currentTemp > 50 ) ) {
    humidity_samples_below_10 = humidity_samples_below_10 +1;
  }

  //if there are 5 samples below 10% then we are sure the cycle is done
  if ( dryer_on and (humidity_samples_below_10 >= 5) ) {
    Particle.publish(PUSHBULLET_NOTIF_HOME, "Your clothes are dry" + getTime(), 60, PRIVATE);
    // Particle.publish(AWS_EMAIL, "Your clothes are dry", 60, PRIVATE);
    dryer_on = false;
  }

  //this indirect method will be used to raise an alarm if the clothes are still not fully dry
  // after this time has elapsed
  if ( dryer_on and (dryerMaxTimer > DRYER_MAX_TIMER) ) {
    Particle.publish(PUSHBULLET_NOTIF_HOME, "ALARM: Your clothes are still not dry (and your dryer is off!)" + getTime(), 60, PRIVATE);
    dryer_on = false;
  }

  if( dryer_on ) {
    dryer_stat = "dryer_on";
  } else {
    dryer_stat = "dryer_off";
  }

  if (USE_BLYNK == "yes") {
    if ( dryer_on ) {
      dryerStatusLed.on();
    } else {
      dryerStatusLed.off();
    }
  }

  return 0;
}

/*******************************************************************************
 * Function Name  : publishTemperature
 * Description    : the temperature/humidity of the dryer are passed as parameters,
                    get stored in internal variables and then published
 * Return         : 0
 *******************************************************************************/
int publishTemperature( float temperature, float humidity ) {

 char currentTempChar[32];
 currentTemp = temperature;
 int currentTempDecimals = (currentTemp - (int)currentTemp) * 100;
 sprintf(currentTempChar,"%0d.%d", (int)currentTemp, currentTempDecimals);

 char currentHumidityChar[32];
 currentHumidity = humidity;
 int currentHumidityDecimals = (currentHumidity - (int)currentHumidity) * 100;
 sprintf(currentHumidityChar,"%0d.%d", (int)currentHumidity, currentHumidityDecimals);

 //publish readings into exposed variables
 currentTempString = String(currentTempChar);
 currentHumidityString = String(currentHumidityChar);

 //publish readings
 Particle.publish(APP_NAME, dryer_stat + " " + currentTempString + "°C " + currentHumidityString + "%", 60, PRIVATE);

 return 0;

}

/*******************************************************************************
 * Function Name  : getTime
 * Description    : returns the time in the following format: 14:42:31
                    TIME_FORMAT_ISO8601_FULL example: 2016-03-23T14:42:31-04:00
 * Return         : the time
 *******************************************************************************/
String getTime() {
  String timeNow = Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL);
  timeNow = timeNow.substring(11, timeNow.length()-6);
  return " " + timeNow;
}

//dryer exposed variables
BLYNK_READ(V20) {
  //this is a blynk value display
  // source: http://docs.blynk.cc/#widgets-displays-value-display
  Blynk.virtualWrite(V20, currentTemp);
}
BLYNK_READ(V21) {
  //this is a blynk value display
  // source: http://docs.blynk.cc/#widgets-displays-value-display
  Blynk.virtualWrite(V21, currentHumidity);
}
BLYNK_READ(V22) {
  //this is a blynk led
  // source: http://docs.blynk.cc/#widgets-displays-led
  if ( dryer_on ) {
    dryerStatusLed.on();
  } else {
    dryerStatusLed.off();
  }
}

//pool exposed variables
BLYNK_READ(V25) {
  //this is a blynk value display
  // source: http://docs.blynk.cc/#widgets-displays-value-display
  Blynk.virtualWrite(V25, pool_temperature_ifttt);
}

//garage exposed variables
BLYNK_READ(V31) {
  //this is a blynk value display
  // source: http://docs.blynk.cc/#widgets-displays-value-display
  Blynk.virtualWrite(V31, garage_status_string);
}

/*******************************************************************************
 * Function Name  : BLYNK_WRITE
 * Description    : these functions are called by blynk when the blynk app wants
                     to write values to the photon
                    source: http://docs.blynk.cc/#blynk-main-operations-send-data-from-app-to-hardware
 *******************************************************************************/
BLYNK_WRITE(BLYNK_GARAGE_BUTTON) {
  // open the garage only when blynk sends a 1, after the user presses for more than one second
  // to avoid opening the garage by mistake
  // background: in a BLYNK push button, blynk sends 0 then 1 when user taps on it
  // source: http://docs.blynk.cc/#widgets-controllers-button

  //this means the button has been pressed
  if ( param.asInt() == 1 ) {
    blynkGarageButtonDebounce = 0;
    blynkGarageButtonPressed = true;

  //this means the button has been released
  } else {
    blynkGarageButtonPressed = false;
  }

}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V20);
  Blynk.syncVirtual(V21);
  Blynk.syncVirtual(V22);
  Blynk.syncVirtual(V25);
  Blynk.syncVirtual(V31);
}

/*******************************************************************************
 * Function Name  : updateBlynkCloud
 * Description    : publish readings to the blynk server every minute so the
                    History Graph gets updated even when
                    the blynk app is not on (running) in the users phone
 * Return         : none
 *******************************************************************************/
void updateBlynkCloud() {

  //is it time to store in the blynk cloud? if so, do it
  if ( (USE_BLYNK == "yes") and (blynkStoreInterval > BLYNK_STORE_INTERVAL) ) {

    //reset timer
    blynkStoreInterval = 0;

    Blynk.virtualWrite(V20, currentTemp);
    Blynk.virtualWrite(V21, currentHumidity);

    if ( dryer_on ) {
      dryerStatusLed.on();
    } else {
      dryerStatusLed.off();
    }

    Blynk.virtualWrite(V25, pool_temperature_ifttt);
    Blynk.virtualWrite(V31, garage_status_string);

  }

}
