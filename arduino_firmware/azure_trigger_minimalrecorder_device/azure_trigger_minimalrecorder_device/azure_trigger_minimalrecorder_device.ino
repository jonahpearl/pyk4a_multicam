/*** Global Variables ***/
#define SAMPLE_PERIOD 2  // ie 1 Khz
elapsedMillis fs_timer;

const int azure_trigger_pin = 17; // listens for azure trigger
int trigger_count = 0;

int your_data = 0;

char message = 'e';
String header = "time,data,trigger";

void setup() {
 
  // Serial setup
  Serial.begin(115200);

  // Set up trigger interrupt counter
  pinMode(azure_trigger_pin, INPUT);
  digitalWrite(azure_trigger_pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(azure_trigger_pin), countTriggerOne, RISING);

  fs_timer = 0;
}


void loop(void)
{ 
  if (fs_timer >= SAMPLE_PERIOD)
  {
    fs_timer = fs_timer - SAMPLE_PERIOD;

    Serial.print(millis()); // Print out the current timestamp
    
    your_data += 1;
    Serial.print(",");
    Serial.print(your_data);
    
    Serial.print(",");
    Serial.print(trigger_count);
    
    Serial.println("");

    if (Serial.available()){
      message = Serial.read();  
      if (message=='h'){ // print header at beginning of file when requested 
        Serial.println(header);
        Serial.flush();  // wait for python to read it
      }
      else if (message=='r'){  //reset trigger counter
        trigger_count = 0;
      }
    }  
  }
}


void countTriggerOne()
{
  trigger_count++;
}
