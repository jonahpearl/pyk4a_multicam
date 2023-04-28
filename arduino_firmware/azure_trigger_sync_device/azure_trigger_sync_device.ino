#define SAMPLING_HZ 30
#define SEC_TO_USEC 1000000
const int SERIAL_START_DELAY = 100;

//int initial_trigger_pin = 2; // pin which master teensy will trigger to start the 30 Hz pulses
int output_pin = 13;
int output_pin_2 = 14;
int output_pin_3 = 15;
int start_flag = 0;
unsigned long sampling_period_micros = 0;

unsigned int trigger_width_micros = 50;  // 0.0001 --> 100 usec
int in_stim = 0;
elapsedMicros sinceTrigger;

unsigned int interrupt_check_period_millis = 5000;
elapsedMillis sinceInterruptCheck;

long num_frames;

long readLongFromSerial()
{
  union u_tag
  {
    byte b[4];
    long lval;
  } u;
  u.b[0] = Serial.read();
  u.b[1] = Serial.read();
  u.b[2] = Serial.read();
  u.b[3] = Serial.read();
  return u.lval;
}

void run_pulses(unsigned long num_pulses){
  unsigned long counter = 0;
  sinceTrigger = 0;

  // For as many frames as requested, run this loop
  while (counter < num_pulses){

    // If time for a frame, send a trigger
    if ((not in_stim) and (sinceTrigger >= sampling_period_micros)){
      digitalWrite(output_pin, HIGH);
      digitalWrite(output_pin_2, HIGH);
      digitalWrite(output_pin_3, HIGH);
      sinceTrigger = sinceTrigger - sampling_period_micros;  // this logic accounts for potential accumulating delays, st the true frame rate remains precisely 30 Hz
      in_stim = 1;
      counter++;

      // Turn off the trigger after "trigger_width_micros" have elapsed
    } else if ((in_stim) and (sinceTrigger >= trigger_width_micros)){
      digitalWrite(output_pin, LOW);
      digitalWrite(output_pin_2, LOW);
      digitalWrite(output_pin_3, LOW);
      in_stim = 0;
    }

    // Check if user is requesting an interrupt by sending a serial input
    if ((sinceInterruptCheck >= interrupt_check_period_millis) and (sinceTrigger < 10000)){
      if (Serial.available()){
        Serial.read();
        Serial.println("Breaking!");
        Serial.flush();
        break;
      }
    }
  }

  // For safety, turn trigger off at end, if it's on
  if (in_stim){
    digitalWrite(output_pin, LOW);
    digitalWrite(output_pin_2, LOW);
    digitalWrite(output_pin_3, LOW);
    in_stim = 0;
  }
}

void serial_flush(void) {
  while (Serial.available()) Serial.read();
}

void setup() {
  // put your setup code here, to run once:
  
  pinMode(output_pin, OUTPUT);
  pinMode(output_pin_2, OUTPUT);
  pinMode(output_pin_3, OUTPUT);
  sampling_period_micros = round((1.0/SAMPLING_HZ) * SEC_TO_USEC);  // confirmed comes out to 33333
  Serial.begin(9600);
  delay(SERIAL_START_DELAY);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Waiting...");
  Serial.println(Serial.available());
  delay(100);
  // Python sends a 4-byte long int to indicate num pulses to do
  if (Serial.available()==4){

    // Get the number
    num_frames = readLongFromSerial();

    // Respond
    Serial.print(num_frames);
    Serial.println(" sync pulses started!");
    Serial.flush();

    // Run the trigger program
    run_pulses(num_frames);

    serial_flush();
  }
}
