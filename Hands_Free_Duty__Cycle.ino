 
 /*
 *  Author: Deepak Reddy
 *  Version : v1.0
 *  Edited on : 03/18/2018
 */

 // Assigning Pins
 const uint8_t REF_MOTOR_ENCODER = 18;  //Input from Reference Motor Encoder
 const uint8_t SHAFT_MOTOR_ENCODER = 21;  // Input from Shaft Motor Encoder
 const uint8_t RELAY = 40; 
 const uint8_t SHAFT_MOTOR_PWM = 3;  // PWM Output
 const uint8_t REF_MOTOR_PWM = 5;   // PWM Output
 const uint8_t REF_MOTOR_ENCODER_LED = 45;
 const uint8_t SHAFT_MOTOR_ENCODER_LED = 49;

 // Creating lookup table using ProgMEM
 

 // Initializing Variables 
 volatile unsigned int RefMotorEncoderTicks = 0;  // Intializing Reference Motor Encoder ticks
 volatile unsigned int ShaftMotorEncoderTicks = 0;     // Intializing Shaft Motor Encoder ticks
 volatile unsigned long PreviousMillis = 0;    
 volatile unsigned long ShaftEncoderPulsePeriod = 0;        
 volatile unsigned int PreviousRefMotorTicks = 0; 
 const float mmTofeet = 1/304.8;              // Conversion of mm to feet
 volatile uint8_t ShaftMotorDutyCycle = 125;          // Setting PWM output duty cycle for Shaft Motor
 volatile float VehicleSpeed = 0;
 volatile bool StartTimer = false;
 volatile float DesiredShaftTicksRate = 0;
 volatile unsigned long CurrentShaftMotorTicksRate = 0;
 volatile bool SendPWM = false; 
 const int interval = 1000;

 void toggleWheelSpeed()
 {
   static const int WheelDiameter = 250;                 // measured in mm. change from int to float if diameter is in decimel.
   static const float pi = 3.14159;
   static const float LinearDistance = pi*WheelDiameter;                     // Circumference in mm.
   static const int REF_MOTOR_ENCODER_CPR = 10;                                   // No of Ticks Per Revoulution
   static const float DistanceInOneTick = LinearDistance*mmTofeet/REF_MOTOR_ENCODER_CPR;   //  Distance covered in feet in single Tick
//    Serial.print("\n");
//        Serial.print("RMT");
//        Serial.print("\t");
//        Serial.print("\t");
//        Serial.print(RefMotorEncoderTicks);
//         Serial.print("\n");
//        Serial.print("PRMT");
//        Serial.print("\t");
//        Serial.print("\t");
//        Serial.print(PreviousRefMotorTicks);
    int RefMotorTicksRate = (RefMotorEncoderTicks - PreviousRefMotorTicks)*1000/interval;   // Ticks Per Second
  
   VehicleSpeed = RefMotorTicksRate*DistanceInOneTick;   // Speed in feets per second
//        Serial.print("\n");
//        Serial.print("RMTR");
//        Serial.print("\t");
//        Serial.print("\t");
//        Serial.print(RefMotorTicksRate);

        Serial.print("\n");
        Serial.print("Speed");
        Serial.print("\t");
        Serial.print("\t");
        Serial.print(VehicleSpeed);
 }

void ChangeDutyCycle()
{
  static const uint8_t SeedWheelTeeth = 15;
  static const uint8_t SeedWidth = 1;    //distance in feets
  static const int SHAFT_MOTOR_ENCODER_CPR = 5;
  static const float TicksPerTeeth = SHAFT_MOTOR_ENCODER_CPR/SeedWheelTeeth;
  float SeedRate = VehicleSpeed/SeedWidth;
  DesiredShaftTicksRate = TicksPerTeeth*SeedRate;
  SendPWM = true;
  
  
}
 
void setup() 
{
   Serial.begin(115200);

// Reference Motor encoder setup
  pinMode(REF_MOTOR_ENCODER_LED,OUTPUT);
  pinMode(REF_MOTOR_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REF_MOTOR_ENCODER), CountRefMotorEncoderTicks, CHANGE);

//  Shaft Motor Encoder Setup
  pinMode(SHAFT_MOTOR_ENCODER_LED, OUTPUT);
  pinMode(SHAFT_MOTOR_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHAFT_MOTOR_ENCODER), CountShaftMotorEncoderTicks, RISING);

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
       interrupts();
        PreviousMillis = millis();     
        PreviousRefMotorTicks = RefMotorEncoderTicks;
//        Serial.print("\n");
//        Serial.print("DSMT");
//        Serial.print("\t");
//        Serial.print("\t");
//        Serial.print(DesiredShaftTicksRate);
     
            
      }

       noInterrupts();
       CurrentShaftMotorTicksRate = 1000000/ShaftEncoderPulsePeriod;
      interrupts();
//        Serial.print("\n");
//        Serial.print("CSMT");
//        Serial.print("\t");
//        Serial.print("\t");
//        Serial.print(CurrentShaftMotorTicksRate);
      
      if ( (CurrentShaftMotorTicksRate < DesiredShaftTicksRate) && (SendPWM == true))
      {
        while(CurrentShaftMotorTicksRate < DesiredShaftTicksRate)
        {
          ShaftMotorDutyCycle += 1;
          analogWrite(SHAFT_MOTOR_PWM, ShaftMotorDutyCycle);
          Serial.print("\n");
          Serial.print("Case 1");
          Serial.print("\t");
          Serial.print("\t");
          Serial.print(ShaftMotorDutyCycle);
          noInterrupts();
          CurrentShaftMotorTicksRate = 1000000/ShaftEncoderPulsePeriod; 
           interrupts();
          Serial.print("\n");
          Serial.print("DSMT");
          Serial.print("\t");
          Serial.print("\t");
           Serial.print(DesiredShaftTicksRate);
          Serial.print("\n");
          Serial.print("CSMT");
          Serial.print("\t");
          Serial.print("\t");
           Serial.print(CurrentShaftMotorTicksRate);
         
          SendPWM = false;      
          PreviousMillis = millis();     
        PreviousRefMotorTicks = RefMotorEncoderTicks;   
        }
        
      }
      else if ((CurrentShaftMotorTicksRate > DesiredShaftTicksRate) && (SendPWM == true))
      {
          while(CurrentShaftMotorTicksRate > DesiredShaftTicksRate)
        {
          ShaftMotorDutyCycle -= 1;
          analogWrite(SHAFT_MOTOR_PWM, ShaftMotorDutyCycle);
            Serial.print("\n");
          Serial.print("Case 2");
          Serial.print("\t");
          Serial.print("\t");
           Serial.print(ShaftMotorDutyCycle);
          noInterrupts();
          CurrentShaftMotorTicksRate = 1000000/ShaftEncoderPulsePeriod; 
          interrupts();
            Serial.print("\n");
          Serial.print("DSMT");
          Serial.print("\t");
          Serial.print("\t");
          Serial.print(DesiredShaftTicksRate);
          Serial.print("\n");
          Serial.print("CSMT");
          Serial.print("\t");
          Serial.print("\t");
          Serial.print(CurrentShaftMotorTicksRate); 
           SendPWM = false;        
           PreviousMillis = millis();     
        PreviousRefMotorTicks = RefMotorEncoderTicks;    
        }
      }
      
   // analogWrite(MainMotorPin, speed);
    
       
  }

  

// ISR Referencce Motor Encoder
 void CountRefMotorEncoderTicks()
 {
  RefMotorEncoderTicks++;
 }

// ISR Shaft Motor Encoder
  void CountShaftMotorEncoderTicks()
 {
  static unsigned long PreviousMicros = 0;
  unsigned long ShaftEncoderTime = micros();
  ShaftMotorEncoderTicks++;
  ShaftEncoderPulsePeriod = (ShaftEncoderTime - PreviousMicros);
  PreviousMicros = ShaftEncoderTime;
 }


