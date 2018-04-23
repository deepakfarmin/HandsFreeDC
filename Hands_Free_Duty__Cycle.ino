 
 /*
 *  Author: Deepak Reddy
 *  Version : v1.0
 *  Edited on : 03/18/2018
 */

#include "PROGMEM_readAnything_.h"

const uint8_t NUMBER_OF_ELEMENTS = 236;

 // Creating lookup table using ProgMEM
  const PROGMEM uint8_t PwmDutyCycle[NUMBER_OF_ELEMENTS]  {
 // 0   1    2   3     4    5    6    7    8   9    10   11   12   13   14   
  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,     // 1
  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,     // 2
  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,     // 3
  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,     // 4
  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,     // 5
  96,  97,  98,  99,  100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110,    // 6
  111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125,    // 7
  126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,    // 8
  141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155,    // 9
  156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170,    // 10
  171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185,    // 11
  186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200,    // 12
  201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215,    // 13
  216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230,    // 14
  231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245,    // 15
  246, 247, 248, 249, 250, 251, 252, 253, 254, 255                              // 16
  };


  const PROGMEM float PulsePerSec[NUMBER_OF_ELEMENTS]  {
 // 0       1      2       3      4         5       6      7       8       9        10     11       12      13     14        
  2.266,  2.351,  2.452,  2.831,  3.266,  3.576,  3.647,  3.745,  3.828,  3.928,  4.020,  4.094,  4.110,  4.429,  4.478,    // 1
  4.516,  4.630,  4.749,  4.932,  4.947,  5.010,  5.056,  5.111,  5.140,  5.175,  5.352,  5.408,  5.421,  5.457,  5.536,    // 2
  5.564,  5.565,  5.601,  5.700,  5.711,  5.732,  5.752,  5.818,  5.995,  6.017,  6.246,  6.261,  6.301,  6.348,  6.425,    // 3
  6.491,  6.560,  6.566,  6.829,  7.163,  7.249,  7.479,  7.651,  7.721,  7.795,  7.973,  8.056,  8.263,  8.271,  8.374,    // 4
  8.672,  8.686,  8.993,  9.005,  9.104,  9.209,  9.228,  9.397,  9.784,  9.984,  10.071, 10.176, 10.245, 10.535, 10.765,   // 5
  11.050, 11.280, 11.787, 11.791, 11.866, 11.913, 11.995, 11.998, 12.001, 12.110, 12.140, 12.174, 12.198, 12.450, 12.657,   // 6
  12.738, 12.759, 12.794, 12.796, 13.180, 13.291, 13.401, 13.419, 13.478, 13.495, 13.926, 14.227, 14.228, 14.459, 14.500,   // 7
  14.553, 14.966, 15.201, 15.464, 15.482, 15.491, 15.674, 15.681, 15.781, 16.094, 16.370, 16.386, 16.427, 16.598, 16.605,   // 8
  16.704, 16.710, 17.019, 17.359, 17.388, 17.506, 17.568, 17.805, 17.928, 18.472, 18.498, 18.579, 19.185, 19.744, 19.887,   // 9
  20.041, 20.050, 20.061, 20.148, 20.359, 20.684, 20.760, 20.895, 20.927, 20.965, 20.983, 21.103, 21.828, 22.305, 22.315,   // 10
  22.330, 22.518, 22.520, 22.667, 22.735, 23.103, 23.165, 23.456, 23.480, 23.784, 23.826, 24.225, 24.230, 24.287, 24.289,   // 11 
  24.709, 24.892, 24.963, 25.246, 25.259, 25.291, 25.308, 25.546, 25.622, 25.679, 25.910, 26.028, 26.035, 26.047, 26.573,   // 12
  26.579, 26.810, 26.968, 26.989, 27.033, 27.161, 27.182, 27.203, 27.247, 27.285, 27.349, 27.375, 27.470, 27.775, 27.776,   // 13
  27.817, 27.888, 27.973, 27.975, 28.145, 28.246, 28.273, 28.426, 28.526, 28.567, 28.586, 28.671, 28.838, 29.126, 29.144,   // 14
  29.165, 29.819, 30.078, 30.302, 31.476, 31.618, 32.276, 32.359, 32.428, 32.505, 32.549, 32.775, 32.775, 33.013, 33.069,   // 15
  33.748, 33.815, 34.062, 34.536, 34.790, 35.000, 35.416, 35.590, 35.606, 35.843, 35.983                                    // 16    
};

// end of lookup table

 // Assigning Pins
 const uint8_t REF_MOTOR_ENCODER = 18;  //Input from Reference Motor Encoder
 const uint8_t SHAFT_MOTOR_ENCODER = 21;  // Input from Shaft Motor Encoder
 const uint8_t RELAY = 40; 
 const uint8_t SHAFT_MOTOR_PWM = 3;  // PWM Output
 const uint8_t REF_MOTOR_PWM = 5;   // PWM Output
 const uint8_t REF_MOTOR_ENCODER_LED = 45;
 const uint8_t SHAFT_MOTOR_ENCODER_LED = 49;  
 // end of Assiging Pins 
  
 
 // Initializing Variables 
 volatile unsigned int RefMotorEncoderPulses = 0;  // Intializing Reference Motor Encoder Pulses
 volatile unsigned int ShaftMotorEncoderPulses = 0;     // Intializing Shaft Motor Encoder Pulses
 volatile unsigned long PreviousMillis = 0;    
 volatile unsigned long ShaftEncoderPulsePeriod = 0;        
 volatile unsigned int PreviousRefMotorPulses = 0; 
 const float mmTofeet = 1/304.8;              // Conversion of mm to feet
 volatile uint8_t ShaftMotorDutyCycle = 125;          // Setting PWM output duty cycle for Shaft Motor
 volatile float VehicleSpeed = 0;
 volatile bool StartTimer = false;
 volatile float DesiredShaftPulsePerSec = 0;
 volatile unsigned long CurrentShaftMotorPulsePerSec = 0;
 //volatile bool SendPWM = false; 
 const int interval = 1000;
// emd of Initializing Variables
 
// Finding Speed of Vehicle
 void toggleWheelSpeed()
 {
   static const int WheelDiameter = 250;                 // measured in mm. change from int to float if diameter is in decimel.
   static const float pi = 3.14159;
   static const float LinearDistance = pi*(float)WheelDiameter;                     // Circumference in mm.
   static const int REF_MOTOR_ENCODER_CPR = 10;                                   // No of Pulses Per Revoulution
   static const float DistanceInOneTick = LinearDistance*mmTofeet/(float)REF_MOTOR_ENCODER_CPR;   //  Distance covered in feet in single Tick
   float RefMotorPulsePerSec = float(RefMotorEncoderPulses - PreviousRefMotorPulses)*1000.0/(float)interval;   // Pulses Per Second
   VehicleSpeed = RefMotorPulsePerSec*DistanceInOneTick;   // Speed in feets per second
    if (VehicleSpeed == 0)
    {
      ShaftMotorDutyCycle = 0;
      analogWrite(SHAFT_MOTOR_PWM, ShaftMotorDutyCycle);
      StartTimer = false;
      digitalWrite(RELAY, LOW);
    }
     
 }
// End of toggleWheelSpeed
 

// Determining the value of PWM between 0 - 255
void ChangeDutyCycle()
{
  static const uint8_t SeedWheelTeeth = 15;
  static const uint8_t SeedWidth = 1;    //distance in feets
  static const int SHAFT_MOTOR_ENCODER_CPR = 36;
  static const float PulsesPerTeeth = (float)SHAFT_MOTOR_ENCODER_CPR/(float)SeedWheelTeeth;
  float SeedRate = VehicleSpeed/(float)SeedWidth;
  DesiredShaftPulsePerSec = PulsesPerTeeth*SeedRate;
  
  // Finding index of PWM value 
  uint8_t k;
  uint8_t kdx = 0; // default index of first element
  float InitialPulsePerSec;
  PROGMEM_readAnything(&PulsePerSec[kdx], InitialPulsePerSec);
  float InitialDifference = abs(InitialPulsePerSec - DesiredShaftPulsePerSec);
  for (size_t k = 1; k < NUMBER_OF_ELEMENTS; k++)
  {
    float NextPulsePerSec;
    PROGMEM_readAnything(&PulsePerSec[k], NextPulsePerSec);
    float FinalDifference = abs(NextPulsePerSec - DesiredShaftPulsePerSec);
    if (FinalDifference >= InitialDifference)
     {
        kdx = k-1;
        break;
     }
     InitialDifference = FinalDifference;
  }
  uint8_t GetPWM;
  PROGMEM_readAnything(&PwmDutyCycle[kdx], GetPWM);
  ShaftMotorDutyCycle = GetPWM;   
  
  //SendPWM = true;
  
}
// end of ChangeDutyCycle



void toggleSendPWM()
{
  analogWrite(SHAFT_MOTOR_PWM, ShaftMotorDutyCycle);
}
 
void setup() 
{
   Serial.begin(115200);

// Reference Motor encoder setup
  pinMode(REF_MOTOR_ENCODER_LED,OUTPUT);
  pinMode(REF_MOTOR_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REF_MOTOR_ENCODER), CountRefMotorEncoderPulses, CHANGE);

//  Shaft Motor Encoder Setup
  pinMode(SHAFT_MOTOR_ENCODER_LED, OUTPUT);
  pinMode(SHAFT_MOTOR_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHAFT_MOTOR_ENCODER), CountShaftMotorEncoderPulses, RISING);

// Motor PWM Pins 
  pinMode(REF_MOTOR_PWM, OUTPUT);
  pinMode(SHAFT_MOTOR_PWM, OUTPUT);
  pinMode(RELAY, OUTPUT);

// Changing PWM frequency to 31KHz on PIns 5 and 3
  TCCR3B = (TCCR3B & 0b11111000) | 0x01; 
 
     while (! Serial);
  Serial.println("Speed 0 to 255");

}

void loop() 
{
//   Serial.print("\n");
//        Serial.print("loop running");
//        
  if (Serial.available())
  {
     Serial.print("\n");
        Serial.print("Available");
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(millis());
            
    int speed = Serial.parseInt();
    if (speed > 0 && speed <= 255)
    {
      Serial.print("\n");
        Serial.print("Speed");
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(speed);
      digitalWrite(RELAY, HIGH);
      analogWrite(REF_MOTOR_PWM, speed);
      
      if (StartTimer == false)
      {
        PreviousMillis = millis();
        analogWrite(SHAFT_MOTOR_PWM, ShaftMotorDutyCycle);
        StartTimer = true;
        Serial.print("\n");
        Serial.print("Start");
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(StartTimer);
      }
    }
      else
       {
           Serial.print("\n");
        Serial.print("RELAY LOW");
       digitalWrite(RELAY, LOW);
       }
 } 
      
      if (((millis() - PreviousMillis) >= interval) && (StartTimer == true))
      {
        Serial.print("\n");
        Serial.print("Measuring Speed");
        noInterrupts();
        toggleWheelSpeed();
        ChangeDutyCycle();
        toggleSendPWM();
        interrupts();
        PreviousMillis = millis();
        PreviousRefMotorPulses = RefMotorEncoderPulses;
     //   PreviousShaftMotorPulses = ShaftMotorEncoderPulses;        
      }
}

  

// ISR Referencce Motor Encoder
 void CountRefMotorEncoderPulses()
 {
  RefMotorEncoderPulses++;
 }

// ISR Shaft Motor Encoder
  void CountShaftMotorEncoderPulses()
 {
 // static unsigned long PreviousMicros = 0;
  //unsigned long ShaftEncoderTime = micros();
  ShaftMotorEncoderPulses++;
  //ShaftEncoderPulsePeriod = (ShaftEncoderTime - PreviousMicros);
  //PreviousMicros = ShaftEncoderTime;
 }


