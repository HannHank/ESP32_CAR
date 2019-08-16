//this header initialize all of the ultrasonic_sensors
const int tri_ultrasinic_FL = 0;
const int echo_ultrasinic_FL = 4;
const int tri_ultrasinic_FR = 2; 
const int echo_ultrasinic_FR = 15;

long durationFL;
int distanceFL;
long durationFR;
int distanceFR;
void define_Ultrasonic_pins(){
    pinMode(tri_ultrasinic_FL,OUTPUT);
    pinMode(echo_ultrasinic_FL,INPUT);

    pinMode(tri_ultrasinic_FR,OUTPUT);
    pinMode(echo_ultrasinic_FR,INPUT);
}