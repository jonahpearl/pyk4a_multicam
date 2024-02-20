/*** INCLUDES ***/
#include <Wire.h> // For I2C Communications with the thermistor digital to analog converter (DAC) and intertial measurement unit (IMU)
#include <Adafruit_Sensor.h> // Adafruit helper library for interacting with sensors
#include <Adafruit_MCP4725.h> // For interacting with the DAC

/*** Global Variables ***/

#define SENSOR_SAMPLE_PERIOD 2 //1kHz
#define SYNCING_PERIOD 5000
#define N_SYNC_LEDS 4

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
char message = 'e';
int header_sent = 0;
String header = "time,led1,led2,led3,led4,therm,dac,trigger";


// Loop Timing Initializations
unsigned long time;
unsigned long previousMillis_sensor = 0; // will store last time the sensor was sampled
elapsedMillis sinceSync = 0; // will store the last time the syncing pins were updated
unsigned long currentMillis = 0; // Stores current system time

//Syncing variables
int sync_bits[] = {0,0,0,0};
const int sync_pins[] = {0,1,2,3};
const int second_sync_pins[] = {4,5,6,7};

void update_sync_state(int *sbits, const int *pins){
  for (int i=0; i < N_SYNC_LEDS; i++){
    digitalWrite(pins[i], sbits[i]);
  }
}

void setup() {
  // Thermistor Setup
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
  pinMode(thermistor_pin,INPUT);
  
  //Serial setup
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
 
  //GPIO Syncer Setup
  for (int i=0; i < N_SYNC_LEDS; i++){
    pinMode(sync_pins[i], OUTPUT);
  }
  for (int i=0; i < N_SYNC_LEDS; i++){
    pinMode(second_sync_pins[i], OUTPUT);
  }

  // Azure trigger set up
  pinMode(azure_trigger_pin, INPUT);
  digitalWrite(azure_trigger_pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(azure_trigger_pin), countTriggerOne, RISING);
  sinceSync = 0;
}


void loop(void)
{ // This is the overall system loop that operates at many MHz. Need to sample only @ 1KHz:

  // Get current time in system timer, in mS:
  currentMillis = millis();
  
  if (currentMillis - previousMillis_sensor >= SENSOR_SAMPLE_PERIOD) // Then its time to sample the sensors! Other slow loops will happen in here; We print at 1KHz
  {

    previousMillis_sensor = currentMillis; // update previousMillis variable for future comparison

    Serial.print(currentMillis); // Print out the current timestamp
    Serial.print(",");

    // Run syncing loop:
    if((sinceSync >= SYNCING_PERIOD) and (
      ((sinceTrigger >= led_min_delay_off_trigger_millis) and (sinceTrigger <= led_max_delay_off_trigger_millis))
      or 
      (trigger_count == 0)
      )
    ) 
    {
      // Time to update the syncing pins!

      // Update timer
      sinceSync = sinceSync - SYNCING_PERIOD;

      // Generate random bit code for first three bits, flip fourth bit
      for (int i=0; i < (N_SYNC_LEDS - 1); i++){
        sync_bits[i] = random(0,2);
      }
      sync_bits[N_SYNC_LEDS - 1] = !sync_bits[N_SYNC_LEDS - 1];
        
      // Update syncing pins
      update_sync_state(sync_bits, sync_pins);
      update_sync_state(sync_bits, second_sync_pins);

    }

    // Print out syncing values
    Serial.print(sync_bits[0]);
    Serial.print(",");
    Serial.print(sync_bits[1]);
    Serial.print(",");
    Serial.print(sync_bits[2]);
    Serial.print(",");
    Serial.print(sync_bits[3]);
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
