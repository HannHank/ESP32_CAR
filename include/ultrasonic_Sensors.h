//this header initialize all of the ultrasonic_sensors
#include <NewPing.h>
#define SONAR_NUM 2     // Number of sensors.
#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.

const int tri_ultrasonic_FL = 2;
const int echo_ultrasonic_FL = 15;
const int tri_ultrasonic_FR = 0; 
const int echo_ultrasonic_FR = 4;
const int tri_ultrasonic_M = 14; 
const int echo_ultrasonic_M = 12;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(tri_ultrasonic_FL, echo_ultrasonic_FL, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(tri_ultrasonic_FR, echo_ultrasonic_FR, MAX_DISTANCE), 
};


int Car_Break = 0;
long durationFL;
int distanceFL;
long durationFR;
int distanceFR;
long durationM;
int distanceM;

void define_Ultrasonic_pins(){
    pinMode(tri_ultrasonic_FL,OUTPUT);
    pinMode(echo_ultrasonic_FL,INPUT);

    pinMode(tri_ultrasonic_FR,OUTPUT);
    pinMode(echo_ultrasonic_FR,INPUT);

    pinMode(tri_ultrasonic_M,OUTPUT);
    pinMode(echo_ultrasonic_M,INPUT);
}