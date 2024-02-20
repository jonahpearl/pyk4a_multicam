/*** INCLUDES ***/
#include <Wire.h> // For I2C Communications with the thermistor digital to analog converter (DAC) and intertial measurement unit (IMU)
#include <Adafruit_Sensor.h> // Adafruit helper library for interacting with sensors
#include <Adafruit_BNO055.h> // Adafruit library for interacting with the IMU
#include <utility/imumaths.h> // Library for performing math operations on the IMU data
#include <AccelStepper.h> // For controlling the Stepper motor with accelerations and deccelarations
#include <Adafruit_MCP4725.h> // For interacting with the DAC

/*** Global Variables ***/

// Loop Update Rates
// We run four loops: the overall main loop and within this, three slower loops
// The slow loops are the sensor sampling and printing loop running at ~1kHz; 
// the motor update loop, running at around 125Hz;
// and the syncing loop, running at a period of ~5s

#define SENSOR_SAMPLE_PERIOD 2 //1kHz
#define MOTOR_UPDATE_PERIOD 12 //125Hz
#define SYNCING_PERIOD 5000

//IMU Initializations
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // 0x28 is the address of the IMU when the addr. pin is tied to ground
sensors_event_t event;
imu::Vector<3> linacc;
int zero_counter = 0;
int bno_restart = 0;

//Motor Initializations
bool direction_state = 0;
bool step_state = 0;
#define stp 7 //2
#define dir 8 //3
#define MS1 10 //4
#define MS2 11 //5
#define SPRD 13
#define SPRD_VAL HIGH
#define EN  9 //6
int rotation_factor = 0; // how many multiples of 360 to add/subtract from the motor's moveTo command
int deg_per_rot = 360; // not 200? -> Belt ratio!
int step_amount = 1; // Number of rotation factors to add/subtract
int microstep = 64; // Microstepping ratio. '8' means 8 microsteps per step
int desired_speed = 900;
int desired_acc = 3;
float steps_per_rotation = (((float)microstep)*200.0)/360.0;
AccelStepper stepper(1, stp, dir);
float prev_yaw = 0; // previous value of yaw, for motor purposes...
float yaw_diff = 0;
float yaw = 0;
long desired_position = 0;

// GPIO Initializations
const int ttl1_pin = 5; 
const int ttl2_pin = 4; 
const int ttl3_pin = 6; 
const int ttl4_pin = 22;

const int sync1_pin = 0; 
const int sync2_pin = 1;
const int sync3_pin = 2;
const int sync4_pin = 15;


const int azure_trigger_pin = 17; // listens for azure trigger
volatile int trigger_count = 0;
const int led_min_delay_off_trigger_millis = 10;  // arbitrary number safely above ~6.4
const int led_max_delay_off_trigger_millis = 28;  // arbitrary number safely below 33
elapsedMillis sinceTrigger; // reset by every interrupt. unfortunately can't seem to make this volatile.

// Thermistor Initializations
Adafruit_MCP4725 dac;

const int thermistor_pin = A0;
int thermistor = 0;

float dac_value = 0; // Float to track desired DAC value

bool dac_update_needed = false; // Flag to update DAC as needed

float dac_correction_amount = 0.05; // 50mV, amount to modify DAC value by per loop iteration if there is a max/min reached

int therm_lower_bound = 200; // If the value goes below this, the DAC will start compensating
int therm_upper_bound = 900; // If the value goes above this, the DAC will start compensating

// Serial initializations
// serial comm params (in non-pyk4a scripts, just for putting header at beginning of file)
char message = 'e';
int header_sent = 0;
String header = "time,led1,led2,led3,led4,yaw,roll,pitch,acc_x,acc_y,acc_z,therm,dac,trigger";

// Loop Timing Initializations
unsigned long time;
unsigned long previousMillis_sensor = 0; // will store last time the sensor was sampled
unsigned long previousMillis_motor = 0; // will store last time the motor was updated
elapsedMillis sinceSync = 0; // will store the last time the syncing pins were updated
unsigned long currentMillis = 0; // Stores current system time

//Syncing variables
int bit1 = 0;
int bit2 = 0;
int bit3 = 0;
int bit4 = 0;

// Error variables
uint8_t system_status, self_test_results, system_error;


void setup() {
  // Thermistor Setup
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
  pinMode(thermistor_pin,INPUT);
  
  // Motor Setup
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);
  resetEDPins(); //Set step, direction, microstep and enable pins to default states
  digitalWrite(EN, LOW);

  digitalWrite(SPRD, SPRD_VAL);
  
  stepper.setMaxSpeed(desired_speed*microstep);
  stepper.setSpeed(desired_speed*microstep); // for runSpeed command
  stepper.setAcceleration(desired_speed*desired_acc*microstep);
  
  //Serial setup
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  // IMU Setup

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P0);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  displayCalStatus();
  Serial.println();

//  delay(10000);
//
//  displayCalStatus();
//  Serial.println();
  
  bno.setExtCrystalUse(false); // changed to false for nanoBNO055 which doesn't have a crystal... (3/23/21 gg)
  
  //GPIO Syncer Setup
  pinMode(ttl1_pin, OUTPUT);
  pinMode(ttl2_pin, OUTPUT);
  pinMode(ttl3_pin, OUTPUT);
  pinMode(ttl4_pin, OUTPUT);

  pinMode(sync1_pin, OUTPUT);
  pinMode(sync2_pin, OUTPUT);
  pinMode(sync3_pin, OUTPUT);
  pinMode(sync4_pin, OUTPUT);

  digitalWrite(ttl1_pin, HIGH);
  digitalWrite(ttl2_pin, HIGH);
  digitalWrite(ttl3_pin, HIGH);
  digitalWrite(ttl4_pin, HIGH);

  pinMode(azure_trigger_pin, INPUT);
  digitalWrite(azure_trigger_pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(azure_trigger_pin), countTriggerOne, RISING);
  
  sinceSync = 0;
}



/**** EVERYTHING BELOW IS WORK IN PROGRESS ****/
void loop(void)
{ // This is the overall system loop that operates at many MHz. Need to sample only @ 1KHz:

  // Get current time in system timer, in mS:
  currentMillis = millis();
  
  if (currentMillis - previousMillis_sensor >= SENSOR_SAMPLE_PERIOD) // Then its time to sample the sensors! Other slow loops will happen in here; We print at 1KHz
  {

    previousMillis_sensor = currentMillis; // update previousMillis variable for future comparison

    Serial.print(currentMillis); // Print out the current timestamp
//    Serial.print(micros());
    Serial.print(",");

    // Run syncing loop:
    if((sinceSync >= SYNCING_PERIOD) and (sinceTrigger >= led_min_delay_off_trigger_millis) and (sinceTrigger <= led_max_delay_off_trigger_millis)) // Then its time to update the syncing pins!
    {
      sinceSync = sinceSync - SYNCING_PERIOD;

      // Generate random bit code
      bit1 = random(0,2);
      bit2 = random(0,2);
      bit3 = random(0,2);
      bit4 = !bit4;
        
      // Update syncing pins:

      //LEDs
      digitalWrite(ttl1_pin, bit1);
      digitalWrite(ttl2_pin, bit2);
      digitalWrite(ttl3_pin, bit3);
      digitalWrite(ttl4_pin, bit4);

      // Copy of LEDs
      digitalWrite(sync1_pin, bit1);
      digitalWrite(sync2_pin, bit2);
      digitalWrite(sync3_pin, bit3);
      digitalWrite(sync4_pin, bit4);

    }

  
    // Catch moments where IMU connection might drop out momentarily
    if (bno_restart)
    {
      while (bno_restart){  // try to restart every five seconds
        if(!bno.begin()){
          /* There was a problem detecting the BNO055 ... check your connections */
          Serial.println("Error");
          delay(10);
        }
        else{
          bno_restart = 0;
          delay(80); // if successful pause for restart, and resume
        }
      }
    }

    // Print out syncing values
    Serial.print(bit1);
    Serial.print(",");
    Serial.print(bit2);
    Serial.print(",");
    Serial.print(bit3);
    Serial.print(",");
    Serial.print(bit4);
    Serial.print(",");

    // Check for BNO disconnection
    if (event.orientation.x == -0.0625) // -0.0625 is the standard output if there is a comm. error with the BNO
    {
      Serial.println("BNO Error");
      
      bno_restart = 1; // Flag BNO for a restart
    }

    // Check for no output from BNO:
    if ((event.orientation.x == 0.0 && event.orientation.y == 0.0 && event.orientation.z == 0.0)) // If all zeros from BNO:
    { // Count times there's only zeros
      zero_counter++;
      if (zero_counter >= 10) // If more than 10 zero lines in a row, flag for restart
      {
        bno_restart = 1;
        zero_counter = 0;
      }
    }


    if (currentMillis - previousMillis_motor >= MOTOR_UPDATE_PERIOD) // Then its time to update the desired motor position! We do this slower than 1KHz to avoid hysterisis
      {
        previousMillis_motor = currentMillis;

        //Sample BNO:
        /* Get a new sensor event */
        bno.getEvent(&event);
        linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        
        yaw = event.orientation.x; // * 200/360;
        yaw_diff =  yaw -  prev_yaw;
        if(abs(yaw_diff) >=180){
          if(yaw_diff<0){ // CW case
            rotation_factor += step_amount;
          }
          else if(yaw_diff>0){ // CCW case
            rotation_factor -= step_amount;
          }

          desired_position = (long) (yaw + deg_per_rot*rotation_factor)*steps_per_rotation;
          //float desired_speed = stepper.speed()*microstep;
          //stepper.setSpeed(desired_speed);
          stepper.moveTo(desired_position); // microstep
        }
        else{
          
          desired_position = (long) (yaw + deg_per_rot*rotation_factor)*steps_per_rotation;
          //float desired_speed = stepper.speed()*microstep;
          //stepper.setSpeed(desired_speed);
          stepper.moveTo(desired_position); // microstep
        }
  
        prev_yaw = yaw;
      }
    
    
    /* Print the IMU data */
    Serial.print(event.orientation.x, 4);
    Serial.print(",");  // ("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print(",");
    Serial.print(event.orientation.z, 4);
      
    Serial.print(",");
    Serial.print(linacc.x());
    Serial.print(",");
    Serial.print(linacc.y());
    Serial.print(",");
    Serial.print(linacc.z());
    Serial.print(",");

    // Sample Thermistor
    thermistor = analogRead(thermistor_pin);

    // DAC Code; The digital to analog converter helps compensate for DC drift in the thermistor, while preserving dynamic range
      if (thermistor < 200)
    {
      dac_update_needed = true;
      dac_value += dac_correction_amount; // Add to dac output
    }
    else if (thermistor > 900)
    {
      dac_update_needed = true;
      dac_value -= dac_correction_amount; // Subtract from dac output
    }
  
    if (dac_value <= 0.05 || dac_value >= 3.25)
    {
      dac_update_needed = false; // DAC is already at extremes then, so no need to update
      //Serial.println("DAC is Maxed out! Switch resistors!"); // Let user know that the DAC is maxed out
  
        //Clamp dac_value between 0 and 3.3
      if (dac_value < 0)
      {
        dac_value = 0;
      }
      else if (dac_value > 3.3)
      {
        dac_value = 3.3;
      }
    }
    
//    dac_update_needed = 0; // TEMPORARY DISABLE DAC
    if (dac_update_needed)
    {
      int dac_value_int = ConvertDACFloatToInt(dac_value); // Convert Float value to usable integer
      dac.setVoltage(dac_value_int, false); // Set the DAC output to the desired value. Only do this if the value needs to be changed!
      dac_update_needed = false;
  
    }

    // Print thermistor and DAC values
    Serial.print(thermistor);
//    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(dac_value);

    Serial.print(",");
    Serial.print(trigger_count);
    
    Serial.println(""); // Print newline! Thats all the data we want to publish

    
    if (Serial.available()){
      message = Serial.read();  
//      Serial.println(message);
//      Serial.flush();
//      delay(5000);
      if (message=='h'){ // print header at beginning of file when requested 
        Serial.println(header);
        Serial.flush();  // wait for python to read it
        header_sent = 1;
      }
      else if (message=='r'){  //reset counters at end of daq
        trigger_count = 0;
      }
      else{
        Serial.flush();
        Serial.println("Unexpected message");
        Serial.flush();
      }
    }  
  }
  stepper.run(); // This tells the motor to move 1 step. Required to call this as quickly as possible, hence it being in the main loop
}


void countTriggerOne()
{
  sinceTrigger = 0;
  trigger_count++;
}

/*** HELPER FUNCTIONS ***/

// Helper function to convert 0-3.3V float to a 0-4095 value the DAC can use
int ConvertDACFloatToInt(float dac_value)
{
  //Clamp dac_value between 0 and 3.3
  if (dac_value < 0)
  {
    dac_value = 0;
  }
  else if (dac_value > 3.3)
  {
    dac_value = 3.3;
  }

  //Convert! DAC integer is a 12 bit setting, so 0 to 4095
  // DAC value is a float from 0 to 3.3
  // 3.3 / 4095 = 0.0008058608 V/count

  int dac_value_int = (int) dac_value/0.0008058608;

  return dac_value_int;
}


//Reset Easy Driver pins to default states
void resetEDPins()
{
  switch (microstep)
  {
    case 8:
      digitalWrite(stp, LOW);
      digitalWrite(dir, LOW);
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, LOW);
      digitalWrite(SPRD, SPRD_VAL);
      digitalWrite(EN, HIGH);
      break;
    case 32:
      digitalWrite(stp, LOW);
      digitalWrite(dir, LOW);
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, LOW);
      digitalWrite(SPRD, SPRD_VAL);
      digitalWrite(EN, HIGH);
      break;
    case 64:
      digitalWrite(stp, LOW);
      digitalWrite(dir, LOW);
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, HIGH);
      digitalWrite(SPRD, SPRD_VAL);
      digitalWrite(EN, HIGH);
      break;
    case 16:
      digitalWrite(stp, LOW);
      digitalWrite(dir, LOW);
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(SPRD, SPRD_VAL);
      digitalWrite(EN, HIGH);
      break;
  }

}

  
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  
 
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
