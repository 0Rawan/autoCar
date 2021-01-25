/****************************************************************************
 * GPS ROBOT CAR
 * Navigate Autonomously GPS Waypoints Using Arduino
 * By Raul Alvarez-Torrico
 * 
 * Magnetic Declination:
 * Go to this page to find your magnetic declination: 
 * http://www.magnetic-declination.com/
 * The declination is provided in degrees and minutes format; you'll have to
 * convert to decimal degrees. Include the '-' sign if present.
 * Then, search this line and modify it: const float mag_declination = -9.38;
 * 
 * Digital Compass:
 * Put the HMC5883L digital compass flat, at least 15cm above the rest of the 
 * electronics to avoid interference. The 'x' direction marked in the module
 * should point towards the front of the car.
 * 
 * GPS module:
 * Put the antenna near the center if possible, it doesn't matter much, though.
 * 
 * Motors:
 * Check the correct rotation direction of your motors. If any motor
 * is reversed, switch their terminals to correct.
 * 
 * I used global variables almost everywhere because I needed to print their
 * values with Print_Data() for debugging.
 * 
 * Additional resources for this project: http://bit.ly/GRCresources
 * or you can reach me at: raul@tecbolivia.com
*****************************************************************************/

// Libraries
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "HMC5883L.h"

#define BUZZER_PIN 7          // Output pin for the buzzer
#define TOLERANCE_RADIUS 1.0  // Tolerance radius for reaching the goal

// Robot car velocities. Change if you want to adjust the car's behavior
// (range of allowed velocities: 0 ~ 255)
#define FORWARD_VEL     230 
#define SLOW_TURN_VEL   220 
#define PIVOT_WHEEL_VEL 50 
#define FAST_TURN_VEL   180

// Constants for defining error ranges for the stepped proportional control
#define MAX_HEADING_ANGLE  180
#define MIN_HEADING_ANGLE  5
#define ANGLE_RANGE_DIV 0.25

// Calibration constants for the motors in differential drive config.
// e. g. if the left motor is slower that the right one, you can add some 
// % to the left and reduce the same % to the right, and viceversa.
#define K_RIGHT_MOTOR 1.0
#define K_LEFT_MOTOR 1.0

// Objects for interfacing with sensors
HMC5883L compass; // Create compass object
TinyGPS gps;      // Create gps object
SoftwareSerial serial_gps(2, 3); // RX and TX pins in use

// Stores the next action the robot car should take (just for debugging)
String str_action; 

// Control signal pins for the L298N the motor driver
int left_motor_pin1 = 10; // IN1 to the driver
int left_motor_pin2 = 11; // IN2 to the driver
int right_motor_pin1 = 5; // IN3 to the driver
int right_motor_pin2 = 6; // IN4 to the driver

//------- Globals for storing waypoints
// Struct to store waypoints (GPS coordinates):
struct t_waypoint { 
  float lat;
  float lon;
};

// Place here your own waypoints, or you'll have your robot car trying to
// reach some place in my hometown! You can use Google Maps to define them:

// Alalay Park waypoints {lat, lon}
t_waypoint nav_waypoints[] =  {
  { -17.4176, -66.13265},   // Point 1
  { -17.41757, -66.13282},  // Point 2
  { -17.41773, -66.13282},  // Point 3
  { -17.41776, -66.13265}   // Point 4
};

// Olimpia court waypoints {lat, lon}
//t_waypoint nav_waypoints[] =  {
//  { -17.4193295, -66.1648977},  // Point 1
//  { -17.419237, -66.1648821},   // Point 2
//  { -17.419206, -66.165064},    // Point 3
//  { -17.419298, -66.165086}     // Point 4
//};

byte waypoint_index = 0;  // Index to the current waypoint in array
// Get the number of waypoints:
int num_waypoints = sizeof(nav_waypoints) / sizeof(nav_waypoints[0]); 

// Auxiliary variables for computing the navigation vector:
float waypoint_lat_rad; 
float waypoint_lon_rad;

//------- Globals for compass readings
int16_t mx, my, mz;   // Earth's magnetic field components
float compass_angle;  // Stores the angle retrieved from the compass

// Magnetic declination in my current area/city. Change it for the value
// defined for your area/city or leave it at zero:
const float mag_declination = -9.38; 

int heading_error; // Stores the heading error

//------- Globals for GPS readings
float f_gps_reading_lat, f_gps_reading_lon; // Stores the reading from the GPS receiver
float waypoint_angle; // Stores the azimuth to the current goal
float last_calc_dist = 0; // Las computed distance to goal (just for debugging)

// Auxiliaries for dumping GPS data
long lat, lon; 
unsigned long age, date, time, chars;
unsigned short sentences, failed;

//------- Globals for the moving average filter
// Increse NUM_FILTERING_POINTS if you want more filtering of the GPS readings,
// it also adds more delay to the system. Sometimes less could be better.
#define NUM_FILTERING_POINTS  15 
float buffer_gps_lat[NUM_FILTERING_POINTS]; // Buffer for latitudes
float buffer_gps_lon[NUM_FILTERING_POINTS]; // Buffer for longitudes
t_waypoint gps_filtered; // For storing a filtered GPS reading
unsigned int buffer_fill_index = 0; // Index for filling filter buffers

/* Setup function */
void setup() {
  // Initialize control pins for the motor driver
  pinMode (left_motor_pin1, OUTPUT);
  pinMode (left_motor_pin2, OUTPUT);
  pinMode (right_motor_pin1, OUTPUT);
  pinMode (right_motor_pin2, OUTPUT);

  Stop(); // Stop the car

  Wire.begin();           // Initialize I2C comm. for the compass
  Serial.begin(9600);     // Initialize serial comm. for debugging
  serial_gps.begin(9600); // Initialize serial comm. with the GPS module

  pinMode(BUZZER_PIN, OUTPUT); // Initialize the buzzer pin

  compass.initialize();   // Initialize the compass

  delay(1000);
  Buzzer();               // Beep to signal the car is ready
}

/* Main loop */
void loop() { 

  // Uncomment the following line and comment the rest in this function if 
  // you want to verify the rotation direction of your motors:
//  Forward(FORWARD_VEL);
  
  Get_Compass_Heading();  // Read the digital compass

  if (Query_Gps()) {      // Query the GPS
    Gps_Dump(gps);        // Read the GPS

    // Store the new GPS reading in filter buffers
    Store_Gps_Reading(f_gps_reading_lat, f_gps_reading_lon);

    // Get filtered GPS latitude and longitude
    gps_filtered = Compute_Filtered_Gps();

    // Compute distance and heading to the goal
    Compute_Navigation_Vector(gps_filtered.lat, gps_filtered.lon);

    // Print data to the PC just for debugging
    Print_Data(); 
  }

  // Move the car to the goal
  Control_Navigation(); 
}

/* Stores a GPS reading in the moving average filter's buffers */
void Store_Gps_Reading(float lat, float lon)
{
  // Shift all buffer values towards the tail
  for (int i = (NUM_FILTERING_POINTS - 1); i > 0; --i) {
    buffer_gps_lat[i] = buffer_gps_lat[i-1];
    buffer_gps_lon[i] = buffer_gps_lon[i-1];
  }

  // Insert new values at the head
  buffer_gps_lat[0] = lat;
  buffer_gps_lon[0] = lon;

  // Increment the number of readings stored in buffers
  if(buffer_fill_index < NUM_FILTERING_POINTS) {
    ++buffer_fill_index;
  }
}

/* Computes filtered latitude and longitude using a moving average filter */
t_waypoint Compute_Filtered_Gps()
{
  float lat_sum;
  float lon_sum;
  
  t_waypoint filtered_waypoint;

  lat_sum = 0;
  lon_sum = 0;

  // Add all values in each buffer
  for (int i = 0; i < buffer_fill_index; ++i) {
    lat_sum = lat_sum + buffer_gps_lat[i];
    lon_sum = lon_sum + buffer_gps_lon[i];
  }

  // Take the average
  filtered_waypoint.lat = lat_sum / float(buffer_fill_index);
  filtered_waypoint.lon = lon_sum / buffer_fill_index;
  
  return filtered_waypoint; // Return filtered values
}

/* Queries the GPS receiver */
bool Query_Gps()
{
  while (serial_gps.available())
  {
    if (gps.encode(serial_gps.read())) {
      return true;
    }
  }
  return false;
}

/* Gets a reading from the compass */
void Get_Compass_Heading(void) {
  // Obtain magnetic field components in x, y and z
  compass.getHeading(&mx, &my, &mz);

  // Calculate the X axis angle W.R.T. North
  compass_angle = atan2(my, mx);
  compass_angle = compass_angle * RAD_TO_DEG; // RAD_TO_DEG = 180/M_PI
  compass_angle = compass_angle - mag_declination; // Compensate for magnetic declination

  // Always convert to positive angles
  if (compass_angle < 0) {
    compass_angle = compass_angle + 360;
  }
}

/* Computes the navigation vector */
void Compute_Navigation_Vector(float gps_lat, float gps_lon) {
  t_waypoint cur_waypoint; 

  // Get current goal
  cur_waypoint = Get_Waypoint_With_Index(waypoint_index);

  float gps_f_lat_rad;
  float gps_f_lon_rad;
  float a_haversine = 0;
  float c_haversine = 0;
  float d_haversine = 0;
  float parcial = 0;
  float delta_lat = 0;
  float delta_lon = 0;

  // Compute the distance to the goal with Haversine formula
  // *******************************************************
  delta_lat = radians(cur_waypoint.lat - gps_lat); 
  gps_f_lat_rad = radians(gps_lat);
  waypoint_lat_rad = radians(cur_waypoint.lat);
  delta_lon = radians(cur_waypoint.lon - gps_lon);
  
  a_haversine = sin(delta_lat / 2.0) * sin(delta_lat / 2.0);
  parcial = cos(gps_f_lat_rad) * cos(waypoint_lat_rad);
  parcial = parcial * sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
  a_haversine += parcial;
  
  c_haversine = 2 * atan2(sqrt(a_haversine), sqrt(1.0 - a_haversine));
  
  d_haversine = 6371000.0 * c_haversine; // Multiply by Earth's radius in meters
  
  last_calc_dist = d_haversine;

  // Check if we are inside the goal's tolerance radius
  if (d_haversine < TOLERANCE_RADIUS) {
    Stop();       // Stop the car
    delay(3000);  // Delay just to check visually were exactly the car reached the goal
    waypoint_index++; // Switch to the next waypoint
    Buzzer(); delay(100); Buzzer(); // Beep to signal 'waypoint reached'
  }

  // Check if we reached all waypoints 
  if (waypoint_index == num_waypoints) {
    Stop(); // Stop the car
    // Beep to signal 'all waypoints reached':
    Buzzer(); delay(100); Buzzer(); delay(100); Buzzer();
    while (1); // Stop the program (reset Arduino board to repeat)
  }
  // Compute the forward azimuth
  // **************************************************
  gps_f_lon_rad = radians(gps_lon);  
  waypoint_lon_rad = radians(cur_waypoint.lon);

  waypoint_angle = atan2(sin(waypoint_lon_rad - gps_f_lon_rad) * cos(waypoint_lat_rad), 
                cos(gps_f_lat_rad) * sin(waypoint_lat_rad) - sin(gps_f_lat_rad) * cos(waypoint_lat_rad) * cos(waypoint_lon_rad - gps_f_lon_rad));
  
  waypoint_angle = waypoint_angle * 180 / PI; // Convert from radians to degrees

  // Always convert to positive angles
  if (waypoint_angle < 0) {
    waypoint_angle += 360;
  }
}

/* Controls the robot car navigation towards the goal */
void Control_Navigation() {
  heading_error = (waypoint_angle - compass_angle); // Compute the heading error
  
  // Correct angle for wrap around
  if (heading_error < -180) {
    heading_error = heading_error + 360;
  }
  if (heading_error > 180) {
    heading_error = heading_error - 360;
  }

  // ----- Stepped proportional control
  // The error is between +5 and +45 (for ANGLE_RANGE_DIV = 0.25):
  if (heading_error > MIN_HEADING_ANGLE && heading_error <= MAX_HEADING_ANGLE * ANGLE_RANGE_DIV) {
    // Turn right
    Turn_Right(SLOW_TURN_VEL);
    str_action = "Right";
  }

  // The error is between +45 and +180 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error > MAX_HEADING_ANGLE * ANGLE_RANGE_DIV && heading_error <= MAX_HEADING_ANGLE) {
    // Turn right fast
    Turn_Right_Fast_Spin(FAST_TURN_VEL);
    str_action = "Right fast";
  }

  // The error is between -5 and -45 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error < -MIN_HEADING_ANGLE && heading_error >= -MAX_HEADING_ANGLE * ANGLE_RANGE_DIV) {
    // Turn left
    Turn_Left(SLOW_TURN_VEL);
    str_action = "Left";
  }

  // The error is between -45 and -180 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error < -MAX_HEADING_ANGLE * ANGLE_RANGE_DIV && heading_error >= -MAX_HEADING_ANGLE) {
    // Turn left fast
    Turn_Left_Fast_Spin(FAST_TURN_VEL);
    str_action = "Left fast";
  }

  // The error is between -5 and +5 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error >= -MIN_HEADING_ANGLE && heading_error <= MIN_HEADING_ANGLE) {
    // Forward
    Forward(FORWARD_VEL);
    str_action = "Forward";
  }

  // Just for debugging:
  else {
    str_action = "Not defined";
  }
}

/*  TinyGPS auxiliary routine, comes with the library */
void Gps_Dump(TinyGPS &gps)
{
  gps.get_position(&lat, &lon, &age);

  //  Query_Gps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&f_gps_reading_lat, &f_gps_reading_lon, &age);

  Query_Gps();

  gps.stats(&chars, &sentences, &failed);
  //  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: "); Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}


/* Returns a waypoint from the array */
struct t_waypoint Get_Waypoint_With_Index(int index)
{
  return nav_waypoints[index]; 
}

/* Moves the car forward */
void Forward(unsigned char vel)
{
  analogWrite (right_motor_pin1, LOW);
  analogWrite (right_motor_pin2, vel * K_RIGHT_MOTOR);
  analogWrite (left_motor_pin1, LOW);
  analogWrite (left_motor_pin2, vel * K_LEFT_MOTOR);
}

/* Turns the car left slowly */
void Turn_Left(unsigned char vel)
{
  analogWrite (right_motor_pin1, LOW);
  analogWrite (right_motor_pin2, vel * K_RIGHT_MOTOR);
  analogWrite (left_motor_pin1, 0);
  analogWrite (left_motor_pin2, PIVOT_WHEEL_VEL * K_LEFT_MOTOR);
}

/* Turns the car right slowly */
void Turn_Right(unsigned char vel)
{
  analogWrite (right_motor_pin1, 0) ;
  analogWrite (right_motor_pin2, PIVOT_WHEEL_VEL * K_RIGHT_MOTOR);
  analogWrite (left_motor_pin1, LOW);
  analogWrite (left_motor_pin2, vel * K_LEFT_MOTOR);
}

/* Turns the car left fast pivoting */
void Turn_Left_Fast_Spin(unsigned char vel)
{
  // Solo motor derecho gira con alta velocidad, el izquierdo se detiene
  analogWrite (right_motor_pin1, LOW);
  analogWrite (right_motor_pin2, vel * K_RIGHT_MOTOR);
  analogWrite (left_motor_pin1, vel * K_LEFT_MOTOR);
  analogWrite (left_motor_pin2, LOW);
}

/* Turns the car right fast pivoting */
void Turn_Right_Fast_Spin(unsigned char vel)
{
  analogWrite (right_motor_pin1, vel * K_RIGHT_MOTOR);
  analogWrite (right_motor_pin2, LOW);
  analogWrite (left_motor_pin1, LOW);
  analogWrite (left_motor_pin2, vel * K_LEFT_MOTOR);
}

/* Stops the car */
void Stop(void)
{
  analogWrite (right_motor_pin1, LOW);
  analogWrite (right_motor_pin2, LOW);
  analogWrite (left_motor_pin1, LOW);
  analogWrite (left_motor_pin2, LOW);
}

/* Generates a beep with a passive buzzer */
void Buzzer() {

  for (int i = 0; i < 500; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(250);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(250);
  }
}

/* Prints data to the serial port for debugging */
/* Open the Serial Plotter in the Arduino IDE to see a graph of the unfiltered
   and filtered latitude. Comment/uncomment sections to see other data.
*/
void Print_Data(void)
{
  // Original and filtered latitude (multiplied by a factor to help the
  // Serial Plotter's minimal capabilities!)
  Serial.print("GPSLAT: "); Serial.print(1000000*f_gps_reading_lat, 1);
  Serial.print(" GPSLATF: "); Serial.print(1000000*gps_filtered.lat, 1);

  // Original and filtered longitude
//  Serial.print(" GPSLON: "); Serial.print(1000000*(f_gps_reading_lon), 1);
//  Serial.print(" GPSLONF: "); Serial.print(1000000*(gps_filtered.lon), 1);


  // Waypoint angle, robot car angle and heading error
//  Serial.print(" CMPANG: "); Serial.print(compass_angle);
//  Serial.print(" GPSANG: "); Serial.print(waypoint_angle);
//  Serial.print(" HDERR: "); Serial.print(heading_error);

  // Put here other data you want to plot


  Serial.println();
}

