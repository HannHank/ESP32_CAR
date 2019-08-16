#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99
#define WAYPOINT_DIST_TOLERANE 5
#define HEADING_TOLERANCE 5

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define M_PI		3.14159265358979323846
#define Task_t 10          // Task Time in milli seconds
double destinationlat = 0;
double destinationlon = 0; 
int headingError;
int sat = 0;
float kmh = 0;
long olddistance = 0;

int dt=0;
unsigned long t;

enum directions
{
  left = TURN_LEFT,
  right = TURN_RIGHT,
  straight = TURN_STRAIGHT
};
directions turnDirection = straight;


struct geoloc {
  float lat;
  float lon;
};

float lat = 0;
float lon = 0;



struct geoloc way1;
struct geoloc way2;
struct geoloc way3;
struct geoloc way4;
struct geoloc way5; 
struct geoloc way6; 
void initializeData(){


way1.lat = 51.000966; 
way1.lon =  13.681750; 
way2.lat = 51.001023;
way2.lon =  13.681800;
way3.lat = 51.001241;
way3.lon = 13.681918;

}  