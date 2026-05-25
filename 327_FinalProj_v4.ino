//--------------------------------------------------------------------------
// Code for Team 15 Final Project - Golf Haptic Interface
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5;        // PWM output pin for base motor
int dirPin = 8;        // direction output pin for base motor
int pwmPin_hand = 6;   // PWN output pin for handle motor -- NEWLY ADDED
int dirPin_hand = 7;   // direction output pin for handle motor -- NEWLY ADDED
int sensorPosPin = A2; // input pin for MR sensor
int ERMpwmPin = 3;     // PWM output pin for ERM motor -- CHANGED TO PIN 3
int ERMdir = 12;        // direction output pin for ERM motor

const int ENC1_A = 2;  // Encoder channel A pin -- NEWLY ADDED
const int ENC1_B = 4;  // Endoder channel B pin -- NEWLY ADDED


// Position tracking variables - BASE
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Kinematics variables - BASE
double xh = 0;           // position of the base [m]
double xh_prev;          // Distance of the base at previous time step
double xh_prev2;
double dxh;              // Velocity of the base
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the base
double dxh_filt_prev;
double dxh_filt_prev2;   // UNUSED?


// Position tracking variables - HANDLE
long count1 = 0;
uint8_t lastState1;
const float DEG_PER_COUNT = 30.0/2284.0;

// Kinetmatics variables - HANDLE
double xh_hand = 0;
double xh_hand_prev = 0;
double dxh_hand = 0;
double dxh_hand_filt = 0;
double dxh_hand_prev = 0;


// Define kinematic parameters
// double rh = 0.052;  //[m] SHOULD BE THIS, ONLY USING 0.0875 FOR TEST PURPOSES
double rh = 0.0875;
// double rp = 0.007;  //[m] 7.0mm <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// double rs = rh;     //[m] 5.2cm <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
double rp = 0.0095/2;   //[m] 9mm
double rs = 0.075;   //[m] 7cm
double rh_hand = 0.038;   // handle radius [m]
double rs_hand = 0.095;     // 9.5cm

// Base force output variables
double force_base = 0;           // force at the base
double Tp_base = 0;              // torque of the motor pulley
double duty_base = 0;            // duty cylce (between 0 and 255)
unsigned int output_base = 0;    // output command to the motor

// Handle force output variables
double force_hand = 0;           // force at the handle
double Tp_hand = 0;              // torque of the motor pulley
double duty_hand = 0;            // duty cylce (between 0 and 255)
unsigned int output_hand = 0;    // output command to the motor


// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  setPwmFrequency(ERMpwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);

  // Output pins
  pinMode(pwmPin, OUTPUT);       // PWM pin for base motor
  pinMode(dirPin, OUTPUT);       // dir pin for base motor
  pinMode(pwmPin_hand, OUTPUT);  // PWM pin for handle motor -- NEWLY ADDED
  pinMode(dirPin_hand, OUTPUT);  // dir pin for handle motor -- NEWLY ADDED
  pinMode(ERMpwmPin, OUTPUT);   // PWM pin for ERM  
  pinMode(ERMdir, OUTPUT);  // dir pin for ERM

  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  analogWrite(pwmPin_hand, 0);     // set to not be spinning (0/255) - NEWLY ADDED
  digitalWrite(dirPin_hand, LOW);  // set direction - NEWLY ADDED

  analogWrite(ERMpwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(ERMdir, LOW);     // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

  lastState1 = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts for BASE **********  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
      flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
      updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is 
      flipped = false;
  }
 
  //*************************************************************
  //*** Section 2. Compute position in meters for BASE **********
  //*************************************************************

  // Print updatedPos via serial monitor
  // Serial.println(updatedPos);

  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = 0.0148 * updatedPos - 10.855;
  // Compute the position of the handle (in meters) based on ts (in radians)
  xh = rh * (ts*(PI/180.0));

  // Print xh via serial monitor
  //Serial.println(xh, 5);

  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;
  
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev; 
      
  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;
    
  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;
    
  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;
  //Serial.println(dxh_filt, 5);

  //*************************************************************
  //*** Section 3. Compute output torque for motor at BASE ******
  //*************************************************************

  double b = 5;
  force_base = -b * dxh_filt;

  // Compute the require motor pulley torque (Tp) to generate that force
  Tp_base = force_base*rp/rs;
  
  //*************************************************************
  //*** Section 4. Compute position in meters for HANDLE ********  
  //*************************************************************

  // ----- Read quadrature encoder -----
  uint8_t state1 = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);

  if (lastState1 == 0b00 && state1 == 0b01) count1++;
  else if (lastState1 == 0b01 && state1 == 0b11) count1++;
  else if (lastState1 == 0b11 && state1 == 0b10) count1++;
  else if (lastState1 == 0b10 && state1 == 0b00) count1++;

  else if (lastState1 == 0b00 && state1 == 0b10) count1--;
  else if (lastState1 == 0b10 && state1 == 0b11) count1--;
  else if (lastState1 == 0b11 && state1 == 0b01) count1--;
  else if (lastState1 == 0b01 && state1 == 0b00) count1--;

  lastState1 = state1;

  // Convert counts to angle
  double theta_hand_deg = count1 * DEG_PER_COUNT;
  double theta_hand_rad = theta_hand_deg * PI / 180.0;

  // Convert angle to handle position
  xh_hand = rh_hand * theta_hand_rad;


  // Compute handle velocity
  dxh_hand = (xh_hand - xh_hand_prev) / 0.001;

  // Low-pass filtered velocity
  dxh_hand_filt = 0.9 * dxh_hand + 0.1 * dxh_hand_prev;

  // Store previous values
  xh_hand_prev = xh_hand;
  dxh_hand_prev = dxh_hand;

  //*************************************************************
  //*** Section 5. Compute output torque for motor at HANDLE ****
  //*************************************************************

  double kw = 30; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< SETTING THIS LOW FOR SAFETY!!
  double wall_position = 0.005; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CAN EXPERIMENT WITH!!

  static bool inContact = false;
  static unsigned long impactTime = 0;

  bool currentlyInContact = (xh_hand > wall_position);

  if (currentlyInContact && !inContact) {
    impactTime = millis() / 64;
    inContact = true;
  }

  if (!currentlyInContact) {
    inContact = false;
  }

  if (currentlyInContact) {
    force_hand = -kw * (xh_hand - wall_position);
  } else {
    force_hand = 0;
  }

  double t = (millis() / 64 - impactTime);

  double impactDuration = 40.0;
  double motorStrength = 0.5 + 0.5 * fabs(dxh_hand_filt);

  if (currentlyInContact && t < impactDuration) {
    analogWrite(ERMpwmPin, (int)(motorStrength * 255));
  } else {
    analogWrite(ERMpwmPin, 0);
  }

  // Compute the require motor pulley torque (Tp) to generate that force
  Tp_hand = force_hand*rh_hand*rp/rs_hand;
 
  //*************************************************************
  //*** Section 6. Force output for BASE ************************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force_base > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty_base = sqrt(abs(Tp_base)/0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty_base > 1) {            
    duty_base = 1;
  } else if (duty_base < 0) { 
    duty_base = 0;
  }  
  output_base = (int)(duty_base* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin, output_base);  // output the signal

  //*************************************************************
  //*** Section 7. Force output for HANDLE **********************
  //*************************************************************

  // Determine correct direction for motor torque
  if(force_hand > 0) {
    digitalWrite(dirPin_hand, HIGH);
  } else {
    digitalWrite(dirPin_hand, LOW);
  }

  // Convert torque to duty cycle <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 0.10 needs to be checked!!! variable per motor!
  duty_hand = sqrt(abs(Tp_hand)/0.10);

  // Make sure the duty cycle is between 0 and 25% <<<<<<<<<<<<<<<<<<<<<<< setting as 25% for safety!!
  if (duty_hand > 0.25) {            
    duty_hand = 0.25;
  } else if (duty_hand < 0) { 
    duty_hand = 0;
  }  
  output_hand = (int)(duty_hand* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin_hand, output_hand);  // output the signal

}

// --------------------------------------------------------------
// Function to set PWM Freq
// --------------------------------------------------------------

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1:    mode = 0x01; break;
      case 8:    mode = 0x02; break;
      case 64:   mode = 0x03; break;
      case 256:  mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1:    mode = 0x01; break;
      case 8:    mode = 0x02; break;
      case 32:   mode = 0x03; break;
      case 64:   mode = 0x04; break;
      case 128:  mode = 0x05; break;
      case 256:  mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
