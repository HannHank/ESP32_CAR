#include <Arduino.h>
#include <GPSNeom8n.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <math.h>
#include <Wire.h>
#include <ESP32_Servo.h>
#include <oled.h>
#include <Path.h>
#include <HMC5883L.h> //
#include <ADXL345.h>
#include <ultrasonic_Sensors.h>
#include <Data.h>

//byte PWM_PIN = 3;
Servo Lenkung;
Servo Motor;
//////Compass stuff
// compass compass;
HMC5883L compass;
ADXL345 accelerometer;
#include <offset.h>

float heading1;
float heading2;
int point = 0;
//gps
int gpsSat = 0;
long dist = 0;
long olddist = 0;
int Test = 0;
int obstacle_avoid_angle = 0;
int Toleranz = 0;
int speed = 0;
int obstacle_state = 0;
int pwm;
int diff = -025;
int fix_Motor = 0;
int old_pwm ;
int changed = 0;
int state_changed = 0;

//Die Task für die ultrasonic_Sensors
void taskOne(void * parameter ){
  for(;;){
    //front left



    // digitalWrite(tri_ultrasonic_FL, LOW);
    // delayMicroseconds(2);
    // // Sets the trigPin on HIGH state for 10 micro seconds
    // digitalWrite(tri_ultrasonic_FL, HIGH);
    
    // delayMicroseconds(10);
    // digitalWrite(tri_ultrasonic_FL, LOW);
    
    // // Reads the echoPin, returns the sound wave travel time in microseconds
    // durationFL = pulseIn(echo_ultrasonic_FL, HIGH);
    // // Calculating the distance
    // distanceFL = durationFL*0.034/2;

    // digitalWrite(tri_ultrasonic_FR, LOW);
    // //front right
    // delayMicroseconds(2);
    // digitalWrite(tri_ultrasonic_FR, HIGH);
    // delayMicroseconds(10);
    
    // digitalWrite(tri_ultrasonic_FR, LOW);
    // durationFR = pulseIn(echo_ultrasonic_FR, HIGH);
    // // Calculating the distance
    // distanceFR = durationFR*0.034/2;

     for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    // Serial.print(i);
    // Serial.print("=");
    // Serial.print(sonar[i].ping_cm());
    // Serial.print("cm ");
    obstacle_avoid_angle = 0;
    if(i == 0){
        distanceFL = sonar[i].ping_cm();
    }
    else{
        distanceFR = sonar[i].ping_cm();
    }
   
   }
    // Serial.println();


  
  obstacle_avoid_angle = 0;
    if(distanceFL <= 70 && distanceFL >= 1){
      obstacle_avoid_angle += map(distanceFL,0,100,100,0);
      //obstacle_avoid_angle += 70 - distanceFL;
      //Serial.print("Avoid angle FL = ");
      //Serial.println(distanceFL);
    
      //Serial.println("________________________________________________________________");
    }
    if(distanceFR <= 70 && distanceFR >= 1){
    
      obstacle_avoid_angle -= map(distanceFL,0,100,100,0);
     
    }
   


    //Middel
    // digitalWrite(tri_ultrasonic_M, LOW);
    // delayMicroseconds(2);
    // digitalWrite(tri_ultrasonic_M, HIGH);
    // delayMicroseconds(10);
    
    // digitalWrite(tri_ultrasonic_M, LOW);
    // durationM = pulseIn(echo_ultrasonic_M, HIGH);
    // // Calculating the distance
    // distanceM = durationM*0.034/2;
    // Serial.println(distanceM);
    // Serial.print("Car: ");
    // Serial.println(Car_Break);
    // Serial.println(distanceFL);
    // Serial.println(distanceFR);
    // if(distanceM <= 20 && distanceM >=1 && Car_Break == 0){
      
    //   Motor.write(70);
    //   Car_Break = 1;
    //   // Serial.println("ok");
     

    // }else{
    //     Car_Break = 0;
    // }
    // Prints the distance on the Serial Monitor
  
    	int pwm = pulseIn(19,HIGH);
      pwm = map(pwm, 994, 1988, 50, 125);
      int pwm_fix = pulseIn(34,HIGH);
       
      pwm_fix = map(pwm_fix, 994, 1988, 50, 125);
      // Serial.println(pwm_fix);
      // Serial.println("PWM: ");
      // Serial.println(pwm);
      // Serial.println(fix_Motor);
      if(fix_Motor == 0 && pwm != -25){
      
        Motor.write(pwm);
        old_pwm = pwm;
       
      }
      else{
        fix_Motor = 1;
       
        Motor.write(old_pwm);

      }
      // Serial.print("changed: ");
      // Serial.println(changed);
      if(pwm_fix <= 94 && pwm_fix>= 88){
        if(changed == 0){
         fix_Motor = 0;
        }
        if(changed == 1){
          changed = 2;
        
        }if(changed == 3){
          changed = 0;
        
        }
        
      }
      else{
         fix_Motor = 1;
         
          
          if(changed == 0)
          {
            changed = 1;
            
           
          }
          if(changed == 2){
           changed = 3;
           Motor.write(70);
           
          }
        
      }
      if( pwm <= 93 && pwm >= 85 ){
            
           
      }
       else{
            if(changed != 2){

            }else{
            Motor.write(70);
            delay(5000);
            changed = 0;
            }
      //   //  Car_Break = 1;
      }
      
      //obstacle_avoid_angle -= 100 - distanceFR;
      //Serial.print("Avoid angle FR= ");
      //Serial.println(distanceFR);
    

    // if(distanceM <= 20){
      
    // }
     delay(10);
    //Serial.println("avoid angle == ");
   //Serial.println(obstacle_avoid_angle);
  }
   vTaskDelete( NULL );
} 
//Funktion zum Berechnen der Distanz
float geoDistance(double lat, double lon, double lat2, double lon2)
{
  const float R = 6371000; // km
  float p1 = lat * DEGTORAD;
  float p2 = lat2 * DEGTORAD;
  float dp = (lat2 - lat) * DEGTORAD;
  float dl = (lon2 - lon) * DEGTORAD;

  float x = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  float y = 2 * atan2(sqrt(x), sqrt(1 - x));

  return R * y;
}
//Berechnen der Kompassdaten OHNE die Neigung zu korrigieren
float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}
//Berechnen der Kompass Daten MIT Neigung korrigiert
float tiltCompensate(Vector mag, Vector normAccel)
{
  // Pitch & Roll

  float roll;
  float pitch;

  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

//Der Winkel des Magnetsensors auslesen
float correctAngle(float heading)
{
  if (heading < 0)
  {
    heading += 2 * PI;
  }
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  return heading;
}
//Den Winkel zwischen aktueller Position und Ziel berechnen
double getBearing(double lat1, double lng1, double lat2, double lng2)
{
  lat1 = radians(lat1);
  lng1 = radians(lng1);
  lat2 = radians(lat2);
  lng2 = radians(lng2);

  double dLng = lng2 - lng1;
  double dPhi = log(tan(lat2 / 2.0 + PI / 4.0) / tan(lat1 / 2.0 + PI / 4.0));

  if (abs(dLng) > PI)
  {
    if (dLng > 0.0)
      dLng = -(2.0 * PI - dLng);
    else
      dLng = (2.0 * PI + dLng);
  }

  return fmod((degrees(atan2(dLng, dPhi)) + 360.0), 360.0);
}
void setup()
{

  ////////////////////////////////////////////////////////////////
  setupGPSm8n();
  ///////////////////////////////////////////////////////////////

  if (!accelerometer.begin())
  {
    delay(500);
  }

  accelerometer.setRange(ADXL345_RANGE_2G);

  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
    // Serial.println("Compass begin");
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  // insert(51.001023,13.681800);
  // insert(51.000966,13.681750);
  
  
  //displayPath();

 
  // print(destinationlon,true,8,6);
  // Serial.print(" : ");
  // Serial.print(destinationlon);
  // DEBUG_PORT.println();
  //
  // compass.setOffset(0,0);
  Motor.attach(18);
  Motor.write(90);
  calculate_offsets();
  compass.setOffset(offX, offY); 
  pinMode(19,INPUT);
  pinMode(34,INPUT);
  start_up();
 
  Lenkung.attach(5);  
  Lenkung.write(90);

  ///
  initializeData();

  //first waypoint
  
  // destinationlat = way1.lat;
  // destinationlon = way1.lon;
  // point = 1;
  define_Ultrasonic_pins();
  xTaskCreate(  taskOne,
                "TaskOne",
                10000,
                NULL,
                1,
                NULL);
  
}

void loop()
{
  //clear display
  display.clearDisplay();
  //calibrating the Compass
  t = millis();
  if(Serial.available()){
    String Route = Serial.readStringUntil('\n');
    Serial.print("Daten bekommen = ");
    Serial.println(Route);
    if(Route == "clear_Route"){
        deletList();
        initializePath();
        point = 0;
    }
    else if(Route == "obstacle_avoid_ON"){
        obstacle_state = 0;

    }
    else if(Route == "obstacle_avoid_OFF"){
        obstacle_state = 1;
    
    }
    else{
        DynamicJsonBuffer jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(Route);
        double dataLat = root[String("location")][String("lat")]; //reading the location lat from the json
        double dataLon = root[String("location")][String("lng")]; //reading the location lng from the json
        insert(dataLat,dataLon); //insert the location to the linked List
        initializePath();
        displayPath();
    }
  }
   if(ptr != NULL && Car_Break == 0){
   //if (ss.available() > 0 ){
    smartDelay(0);
    lat = gps.location.lat();
    lon = gps.location.lng();
    dist = geoDistance(lat, lon, ptr->lat, ptr->lon);
    speed = gps.speed.kmph();
    print(gps.satellites.value(), gps.satellites.isValid(), 5);
    print(gps.location.lat(), gps.location.isValid(), 10, 6); //eigentlich 10 zu 6
    print(gps.location.lng(), gps.location.isValid(), 11, 6);
    print(dist, true, 4);
    print(point,true, 3);
    
    
    DEBUG_PORT.println();
  

    if (dist <= 2)
    { 
      ptr = ptr->next; 
      destinationlat = ptr->lat;
      destinationlon = ptr->lon;
      point += 1;
      
    }
    

  Vector mag = compass.readNormalize();
  Vector acc = accelerometer.readScaled();

  //   // Calculate headings
  heading1 = noTiltCompensate(mag);
  heading2 = tiltCompensate(mag, acc);

  if (heading2 == -1000)
  {
    heading2 = heading1;
  }

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (3.0 + (59.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180 / M_PI;
  heading2 = heading2 * 180 / M_PI;

  // Serial.println(gpsSat);
  // Serial.println(gpsSat);
  double targetHeading = getBearing(lat, lon, ptr->lat, ptr->lon);
  // Serial.print("Heading =");
  // Serial.println(heading1);
  // Serial.print("targetHeading =");
  // Serial.println(targetHeading);

  float turn = targetHeading - heading1;
  while (turn < -180)
    turn += 360;
  while (turn > 180)
    turn -= 360;
  // if(turn < 0){
  //   // Serial.print("kleiner Null");
  //   // Serial.print(turn);
  //   turn = turn *-1;
  //   // Serial.print("jetzt =");
  //   // Serial.println(turn);
  // }
  // else{
  // //  Serial.print("größer Null");
  //  // Serial.print(turn);
  //   turn = turn *-1;
  //   //Serial.print("jetzt =");
  //  // Serial.println(turn);
  // }

  int autoSteer = map(turn, 180, -180, 210, -30); //Hier habe ich Veränderungen vorgenommen
  if(obstacle_state == 0){
  autoSteer +=  obstacle_avoid_angle;
  }
  autoSteer = constrain(autoSteer, 10, 170);//alte Werte sind 50 bis 130

  float angleError = abs(turn);
  // ''Serial.print("angelError =");
  // Serial.println(angleError);

  // Serial.print("AutoSteer");
  // Serial.println(autoSteer);''

  if (autoSteer > 92)
  {
    //Serial.println("Links");
    // display.setTextColor(WHITE);
    // display.setCursor(0, 20);
    // display.print("links");
  }
  else if (autoSteer < 88)
  {
    //Serial.println("rechts");
    // display.setTextColor(WHITE);
    // display.setCursor(0, 20);
    // display.print("rechts");
  }
  else
  {
    //Serial.println("Geradeaus");
    // display.setTextColor(WHITE);
    // display.setCursor(0, 20);
    // display.print("Geradeaus");
  }
  
  Lenkung.write(autoSteer);


  //print GPS Data
  // display.setTextColor(WHITE);
  // display.setCursor(0, 0);
  // display.print("SATS =");
  // display.print(gpsSat);
  // display.setCursor(0, 10);
  // display.print("dist =");
  // display.print(dist);00,

  // display.setCursor(40, 20);
  // display.print(point);
  // display.display();
   }
   else{ //when there is no route in the linked list
    Lenkung.write(90);
    
      // int turn = 0;
      // int autoSteer = map(turn, 180, -180, 190, -10); //Hier habe ich Veränderungen vorgenommen
      // autoSteer +=  obstacle_avoid_angle;
      // autoSteer = constrain(autoSteer, 10, 170);//alte Werte sind 50 bis 130
      // Lenkung.write(autoSteer);
      // Serial.print("AutoSteer");
      // Serial.println(autoSteer);
      //if (ss.available() > 0 ){
    smartDelay(0);

    dist = 0;

    

    
    
  
  //}
}
}
