//--------------------------------------------------------------------------
// Code for Team 15 Final Project - Golf Haptic Interface
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// Pin declares
int pwmPin = 5;        		  // PWM output pin for BASE motor
int dirPin = 8;        		  // direction output pin for BASE motor
int pwmPin_hand = 6;   		  // PWN output pin for HANDLE motor -- NEWLY ADDED
int dirPin_hand = 7;   		  // direction output pin for HANDLE motor -- NEWLY ADDED
int sensorPosPin = A3;      // input pin for BASE MR sensor
int sensorPosPin_hand = A2; // input pin for HANDLE MR sensor -- NEWLY ADDED
int ERMpwmPin = 3;     		  // PWM output pin for ERM motor 1
int ERMpwmPin_2 = 4;     		// PWM output pin for ERM motor 2
// int ERMdir = 12;        	// direction output pin for ERM motor


// Position tracking variables - BASE
int updatedPos = 0;     	// keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         	// current raw reading from MR sensor
int lastRawPos = 0;     	// last raw reading from MR sensor
int lastLastRawPos = 0; 	// last last raw reading from MR sensor
int flipNumber = 0;     	// keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Kinematics variables - BASE
double xh = 0;           	// position of the BASE [m]
double xh_prev;          	// Distance of the BASE at previous time step
double dxh;              	// velocity of the BASE
double dxh_prev;
double dxh_filt;         	// Filtered velocity of the BASE
double dxh_filt_prev;


// Position tracking variables - HANDLE
int updatedPos_hand = 0;
int rawPos_hand = 0;
int lastRawPos_hand = 0;
int lastLastRawPos_hand = 0;
int flipNumber_hand = 0;
int tempOffset_hand = 0;
int rawDiff_hand = 0;
int lastRawDiff_hand = 0;
int rawOffset_hand = 0;
int lastRawOffset_hand = 0;
boolean flipped_hand = false;

// Kinetmatics variables - HANDLE
double xh_hand = 0;
double xh_hand_prev = 0;
double dxh_hand = 0;
double dxh_hand_prev = 0;
double dxh_hand_filt = 0;
double dxh_hand_filt_prev = 0;


// Define kinematic parameters
double rp = 0.005;              //[m] 5mm
double rs = 0.0625;             //[m] 6.25cm
double rs_hand = 0.095;         //[m] 9.5cm

// BASE force output variables
double force_base = 0;           // force at the BASE
double Tp_base = 0;              // torque of the motor pulley
double duty_base = 0;            // duty cylce (between 0 and 255)
unsigned int output_base = 0;    // output command to the motor

// HANDLE force output variables
double force_hand = 0;           // force at the HANDLE
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
  pinMode(sensorPosPin, INPUT);      // BASE MR sensor
  pinMode(sensorPosPin_hand, INPUT); // HANDLE MR sensor

  // Output pins
  pinMode(pwmPin, OUTPUT);        // PWM pin for BASE motor
  pinMode(dirPin, OUTPUT);        // dir pin for BASE motor
  pinMode(pwmPin_hand, OUTPUT);   // PWM pin for HANDLE motor
  pinMode(dirPin_hand, OUTPUT);   // dir pin for HANDLE motor
  pinMode(ERMpwmPin, OUTPUT);     // PWM pin for ERM 1
  pinMode(ERMpwmPin_2, OUTPUT);     // PWM pin for ERM 2
  // pinMode(ERMdir, OUTPUT);    	  // dir pin for ERM
 
  // Initialize motor
  analogWrite(pwmPin, 0);         // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);      // set direction

  analogWrite(pwmPin_hand, 0);    // set to not be spinning (0/255) - NEWLY ADDED
  digitalWrite(dirPin_hand, LOW); // set direction - NEWLY ADDED

  analogWrite(ERMpwmPin, 0);      // set to not be spinning (0/255)
  analogWrite(ERMpwmPin_2, 0);    // set to not be spinning (0/255)
 
  // Initialize BASE MR position variables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;

  // Initialize HANDLE MR position variables
  lastLastRawPos_hand = analogRead(sensorPosPin_hand);
  lastRawPos_hand = analogRead(sensorPosPin_hand);
  flipNumber_hand = 0;
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
  } else {                       // anytime no flip has occurred
      updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is
      flipped = false;
  }
 
  //*************************************************************
  //*** Section 2. Compute position in meters for BASE **********
  //*************************************************************

  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = -0.0171 * updatedPos + 4.3915;
  // Compute the position of the handle (in meters) based on ts (in radians)
  xh = rs*ts*(PI/180.0);

  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;
 
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev;
     
  // Record the position and velocity
  xh_prev = xh;
  dxh_prev = dxh;
  dxh_filt_prev = dxh_filt;

  //*************************************************************
  //*** Section 3. Compute output torque for motor at BASE ******
  //*************************************************************

  double b = 0.15;

  dxh_filt = 0.03 * dxh_filt + 0.97 * dxh_filt_prev;

  double position_deadband = 0.003;
  double velocity_deadband = 0.03;

  if (abs(dxh_filt) < velocity_deadband) {
    force_base = 0;
  } else {
    force_base = b * dxh_filt;
  }

  Tp_base = force_base * rp / rs;
 
  //*************************************************************
  //*** Section 4. Compute position in meters for HANDLE ********
  //*************************************************************

  rawPos_hand = analogRead(sensorPosPin_hand);
  rawDiff_hand = rawPos_hand - lastRawPos_hand;
  lastRawDiff_hand = rawPos_hand - lastLastRawPos_hand;
  rawOffset_hand = abs(rawDiff_hand);
  lastRawOffset_hand = abs(lastRawDiff_hand);

  lastLastRawPos_hand = lastRawPos_hand;
  lastRawPos_hand = rawPos_hand;

  if ((lastRawOffset_hand > flipThresh) && (!flipped_hand)) {
    if (lastRawDiff_hand > 0) {
      flipNumber_hand--;
    } else {
      flipNumber_hand++;
    }

    if (rawOffset_hand > flipThresh) {
      updatedPos_hand = rawPos_hand + flipNumber_hand * rawOffset_hand;
      tempOffset_hand = rawOffset_hand;
    } else {
      updatedPos_hand = rawPos_hand + flipNumber_hand * lastRawOffset_hand;
      tempOffset_hand = lastRawOffset_hand;
    }

    flipped_hand = true;
  } else {
    updatedPos_hand = rawPos_hand + flipNumber_hand * tempOffset_hand;
    flipped_hand = false;
  }
  
  //*************************************************************
  //*** Section 5. Compute position in meters for HANDLE ********
  //*************************************************************

  // Compute the angle of the sector pulley (ts_hand) in degrees based on updatedPos_hand
  double ts_hand = -0.0074 * updatedPos_hand + 6.9865;

  // Compute the position of the handle (in meters) based on ts_hand (in radians)
  xh_hand = (ts_hand*(PI/180.0));]

  // Calculate velocity with loop time estimation
  dxh_hand = (double)(xh_hand - xh_hand_prev) / 0.001;
 
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_hand_filt = .9*dxh_hand + 0.1*dxh_hand_prev;
     
  // Record the position and velocity
  xh_hand_prev = xh_hand;
  dxh_hand_prev = dxh_hand;
  dxh_hand_filt_prev = dxh_hand_filt;

  //*************************************************************
  //*** Section 6. Compute output torque for motor at HANDLE ****
  //*************************************************************

  double kw = 50; 
  double bw = 0.05;
  double ball_position = 0.15;
  double ballSize = 0.05;

  static bool inContact = false;
  static unsigned long impactTime = 0;
  
  bool currentlyInContact = (xh_hand >= ball_position);
  bool freeAfterContact = (xh_hand >= ball_position + ballSize);

  if (currentlyInContact && !inContact) {
      impactTime = millis()/64;
      inContact = true;
  }

  if (!currentlyInContact) {
      inContact = false;
  }

  if (currentlyInContact && freeAfterContact) {
    force_hand = 0;
  } else if (currentlyInContact) {
    force_hand = kw * (xh_hand - ball_position) + bw * dxh_hand_filt;
  } else {
    force_hand = 0;
  }

  double t = (millis() / 64 - impactTime);

  double impactDuration = 100.0;
  double motorStrength = 1;

  if (currentlyInContact && t < impactDuration) {
    analogWrite(ERMpwmPin, (int)(motorStrength * 255));
    analogWrite(ERMpwmPin_2, (int)(motorStrength * 255));
  } else {
    analogWrite(ERMpwmPin, 0);
    analogWrite(ERMpwmPin_2, 0);
  }

  Tp_hand = force_hand*rp/rs_hand;
 
  //*************************************************************
  //*** Section 7. Force output for BASE ************************
  //*************************************************************

  if(force_base > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  duty_base = sqrt(abs(Tp_base)/0.0183);

  if (duty_base > 1) {            
    duty_base = 1;
  } else if (duty_base < 0) {
    duty_base = 0;
  }
  output_base = (int)(duty_base * 255);
  analogWrite(pwmPin, output_base);

  //*************************************************************
  //*** Section 8. Force output for HANDLE **********************
  //*************************************************************

  if(force_hand > 0) {
    digitalWrite(dirPin_hand, HIGH);
  } else {
    digitalWrite(dirPin_hand, LOW);
  }

  duty_hand = sqrt(abs(Tp_hand)/0.10);

  if (duty_hand > 0.5) {            
    duty_hand = 0.5;
  } else if (duty_hand < 0) {
    duty_hand = 0;
  }

  output_hand = (int)(duty_hand * 255);

  analogWrite(pwmPin_hand, output_hand);

  double angleDeg = ts; // deg
  double clubPosDeg = -ts_hand*4; //deg
  double velocityPct = dxh_hand/15*100; // percent max velocity

  //*************************************************************
  //*** Section 9. Print to graphics input **********************
  //*************************************************************

  Serial.print(angleDeg);
  Serial.print(",");
  Serial.print(clubPosDeg);
  Serial.print(",");
  Serial.println(velocityPct);
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