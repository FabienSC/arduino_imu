  #include <Wire.h>
  #include <Servo.h>
  #include <math.h>
  #include <ADXL345.h>
  #include <L3G.h>
  #include <PID_v1.h>
  #include <Adafruit_BMP085.h>  //pressure & temp
  #include <HMC5883L.h>
  long timer = 0;
  
  //**************************************************************************//
  //**************************************************************************//
  //**************************************************************************//
  
  ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
  L3G gyro;
  Adafruit_BMP085 bmp; //bmp is the pressure sensor
  HMC5883L compass;

  int accelX, accelY, accelZ;  //Accelerometer inputs
  int gyroX, gyroY, gyroZ;
  long rawAngle = 0;
  int angle = 0;
  int gyroTimer = 0;
  int avgT = 0;
  double avgFB = 0, avgLR = 0;
  double altitude;//altitude in cm
  long bmpAvg = 0;//ground Pressure
  float heading = 0;//compass heading in degrees
  
  int ignition = 0, engineSpeedL = 0, engineSpeedR = 0, servoL = 0, servoR = 0, inflation = 0;
  char remoteData[9] = {'0', '0', '0', '0', '0', '0', '0', '0', '0'};
  
  double axisLR=0, axisFB=0;   //correcting LR/FB speed for motors
  
  double outLR, outFB;         //PID corrections for the motor speeds
  PID pidLR(&axisLR, &outLR, 0, 0.036, 0.009, 0.0075, DIRECT);
  PID pidFB(&axisFB, &outFB, 0, 0.019, 0.0026, 0.0035, DIRECT);
  
  //**************************************************************************//
  //**************************************************************************//
  //**************************************************************************//
  
void setup(){
  Serial.begin(115200);
  bmp.begin((char)3);
  adxl.powerOn();
  delay(10);
  adxl.setRate(3200);//affects the rate at which the accelerometer updates its internal data (not output speed)
  delay(10);
  Wire.begin();
  delay(10);
  gyro.init();
  delay(10);
  gyro.enableDefault();
//  compass = HMC5883L(); // Construct a new HMC5883 compass.
//  compass.SetScale(1.3); // Set the scale of the compass.
//  compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

//  for(int i=0; i<50; i++)
//    {bmpAvg = bmpAvg + bmp.readPressure(); delay(5);}
//  bmpAvg = bmpAvg/50;
//  
//  pidLR.SetSampleTime(5);
//  pidFB.SetSampleTime(5);
//  pidLR.SetOutputLimits(-20, 20);
//  pidFB.SetOutputLimits(-20, 20);
//  pidLR.SetMode(AUTOMATIC);
//  pidFB.SetMode(AUTOMATIC);
  
  Serial.println("start:");
  Serial.println(micros());
}

void loop(){//----------------------------------------------
  // timer = micros();
   
  //****   ADD WHAT YOU WANT TO TIME HERE  ******************
  
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   myTimedFunction();
   
//   pidLR.Compute();
//   pidFB.Compute();
  //*********************************************************
  
 // Serial.println((long)micros()-timer-2);
  
}//--------------------------------------------------

void myTimedFunction()
  {
    adxl.readAccel(&accelY, &accelX, &accelZ);
    gyro.read();
    gyroX = (int)gyro.g.x;
    gyroY = (int)gyro.g.y;
    accelX = accelX;
    accelY = accelY;
    
//    avgT = micros() - gyroTimer - 2;
//    gyroTimer = micros();
//    if (avgT > 5000)
//      {avgT = 0;}
//    avgFB = 0.995 *(avgFB + (avgT * gyroY)/40000.0) - (accelY) * 5.0;//True FB position
//    avgLR = 0.995 *(avgLR - (avgT * gyroX)/40000.0) - (accelX) * 5.0;//true LR position
//    axisFB = avgFB/1000.0;//scaled down FB position
//    axisLR = avgLR/1000.0;//scaled down LR position
//
//    altitude = .9 * altitude + 0.1 * bmp.readAltitude(bmpAvg);//returns value in meters
    
//    MagnetometerRaw raw = compass.ReadRawAxis();
//    MagnetometerScaled scaled = compass.ReadScaledAxis();
//    int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
//    heading = atan2(scaled.YAxis, scaled.XAxis);
//    
//    float headingDegrees = heading * 180/M_PI; 
    
    Serial.println(accelX);
  }
