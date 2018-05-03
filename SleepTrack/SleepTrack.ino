#include <ArduinoJson.h>
#include <dht.h>
#include <Wire.h>
#include <Ethernet.h>


/******************** ETHERNET SETTINGS ********************/

byte mac[] = { 0x90, 0xA2, 0xDA, 0x0D, 0x85, 0xD9 };   //physical mac address
byte ip[] = { 192, 168, 0, 112 };                   // ip in lan can change
byte subnet[] = { 255, 255, 255, 0 };              //subnet mask
byte gateway[] = { 192, 168, 0, 1 };              // default gateway
EthernetServer server(80);                       //server port

/******************** SENSOR SETUP ********************/
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
const int MPU_addr=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
bool collect; // If sensor should collect or not, based on movement
double volts;
int counter = 0; // Incrimental counter for controlling how long to record data after triggered.
// Set up variables that interact with the GYRO.
int sleepBadAcX, sleepBadAcY, sleepBadAcZ, sleepBadGyX, sleepBadGyY, sleepBadGyZ = 0; 
int16_t oldAcX,oldAcY,oldAcZ,oldGyX,oldGyY,oldGyZ; // Previous values to compare to, so we can detect a change.
int avgTmp, num = 0;
dht DHT; // For temp humidity reading
#define DHT11_PIN 7
long startTime = millis(); // Get startime of arduino 
int minutesAsleep = 0;
int hoursAsleep =0;
int snoring = 0;
int sleepScore = 20000;

void setup() // Begin the server, begin wire (FOR TEMPS)
{
  
  Ethernet.begin(mac,ip,gateway,subnet);     // initialize Ethernet device
  server.begin();                                // start to listen for clients
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);
  collect = false;
}
 
 
void loop() 
{
  EthernetClient client = server.available();
client.println("HTTP/1.1 200 OK");
client.println("Content-Type: text/html");
client.println("Connnection: Keep-Alive");
client.println("Access-Control-Allow-Origin: *"); // CORS can be cahnged to only allow certain IPS for greater security, EX, Doctors, patients, etc.
client.println();


  long elapsedTime =   millis() - startTime; 
   int chk = DHT.read11(DHT11_PIN);
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level
 
   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;
   getprevGyro(); // get first round of "old" data.
   // collect data for 50 mS
  
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(0);
      
      if (sample < 1024)  // toss out false readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   volts = (peakToPeak * 5.0) / 1024;  // convert to volts
 
   Serial.print("SOUND:" );
   Serial.println(volts);

shouldCollect();


// This tells us if there is a big difference in old GYRO vs new Readings, if there is we can assume patient is restless.
if(collect){
getGyro();
if (oldAcX + AcX > 0  ){
    sleepBadAcX +=1 ;
    sleepScore --;

  }
if (oldGyX - GyX > 100  ){
    sleepBadGyX +=1 ;
    sleepScore --;
  }
if (oldGyY - GyY > 100  ){
    sleepBadGyY +=1 ;
    sleepScore --;
  }
  if (oldGyZ - GyZ > 100  ){
    sleepBadGyZ +=1 ;
    sleepScore --;

  }
}



//TEMP COLLECTION SETTINGS
avgTmp  = ((Tmp/340.00+36.53)* 1.8 + 32); // Convert to F
Serial.println("TEMP");
Serial.println(avgTmp);





// This is where the body temp goes, if the temp falls below XX we start reading, this can be tweaked with more variables, better numbers in future, such as when a user puts it on if body temp goes up by XX within 10 seconds.
if (avgTmp  <= 98.5){
long secondsAsleep = (elapsedTime / 1000);
Serial.println(secondsAsleep);

if (secondsAsleep >= 60){
  minutesAsleep ++;
  startTime = millis();
  secondsAsleep =0;
  }
  if (minutesAsleep >=60){
  hoursAsleep ++;
  startTime = millis();
  minutesAsleep = 0;
  
    }

}

Serial.println(hoursAsleep);
Serial.println("Hours");
Serial.println(minutesAsleep);
Serial.println("Minutes");

// This is to send the data as JSON to the internet.
StaticJsonBuffer<200> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();



root["DATE"] = "4/28/15";
root["MINUTES_ASLEEP"] = minutesAsleep;
root["HOURS_ASLEEP"] = hoursAsleep;
root["MOVE_X_DIR"] = sleepBadGyX;
root["MOVE_Y_DIR"] = sleepBadGyY;
root["MOVE_Z_DIR"] = sleepBadGyZ;
root["ACCELERATION"] = sleepBadAcX;
root["LOUD_SOUND_COUNT"] = snoring;
root["BODY_TEMP"] = "96.6"; // Sensor broken temperary.
root["PILLOW_TEMP"] = avgTmp;
root["HUMIDITY"] = "15%"; // DHT.humidity;
root["sleep_score"] = sleepScore;
root["Daily_tip"] = dailyTip();

root.printTo(client);
client.stop();

}
void getprevGyro(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); 
  Tmp=Wire.read()<<8|Wire.read();
  oldAcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  oldAcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  oldAcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  oldGyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  oldGyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  oldGyZ=Wire.read()<<8|Wire.read();
  }

void getGyro(){
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | TEMP = "); Serial.print((Tmp/340.00+36.53)* 1.8 + 32);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);

  
  }
  
void shouldCollect(){

if(volts > .6){
  collect = true;
  snoring ++;
  counter = 0;
  }
else{
  if (counter > 100){
  collect = false;
  }}
  counter +=1;
    }

void calcScore(){
  if (avgTmp < 100 && avgTmp > 80){
    sleepScore ++;
    }
  }

  String dailyTip(){ // Some sample tips.
    if (avgTmp > 98){
      return "Your temperature is too high, try to lower the thermostat.";
      }
    else if (sleepBadGyX + sleepBadGyX + sleepBadGyX > 100){
      return "You seem to be turning alot in bed, try listening to relaxing music before bed";
      }  
    else if (DHT.humidity > 50){
      return "Your humididty is high, try turning on a dehumidifier.";
      }
    else if (snoring > 100){
      return "You seem to be snoring alot, try purchasing a snoring mouthguard.";
      }
     else {return "No tips now try again later.";
}
    
    
    }
