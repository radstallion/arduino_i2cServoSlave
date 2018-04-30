#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PID_v1.h>
#include <Wire.h>
#include <L298N.h>
//initilize motor
L298N motor(5, 6, 7);


//initilize Encoder 
//read encoder : myEnc.read()
Encoder myEnc(2, 3);


//initilize PID parameters
// compute PID : myPID.Compute();
// set pid : myPID.SetTunings(Kp/100.0, Ki/100.0, Kd/100.0);
double Kp = 0.1, Ki=0.2, Kd=0.1;
double Setpoint, CurrentPosition, Output;
PID myPID(&CurrentPosition, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

uint8_t PID_interval = 1;
unsigned long PID_previousMillis = 0; 
unsigned long PID_currentMillis = millis();

//initilize device ID parameters
uint8_t deviceID = 0,idPin[] = {9,10,11,12};
long position  = 0;


//initilize DEBUG and I2C parameters
uint8_t DEBUG = 0;
uint8_t CMD;
uint8_t buffer[16];
uint16_t DEBUG_interval = 500;
unsigned long DEBUG_previousMillis = 0; 
unsigned long DEBUG_currentMillis = millis();

//register
#define TEST 0
#define GET_POSITION 1
#define GET_KPID 2
#define SET_POSITION 3
#define SET_KPID 4


//initilize limit switch parameters
#define HOME_SWITCH 4
void setup() {

    
    for(int i; i<4;i++){
        pinMode(idPin[i],INPUT_PULLUP);
    }
    for(int i; i<4;i++){
        deviceID += digitalRead(idPin[i]) << i;
    } 

    pinMode(HOME_SWITCH,INPUT_PULLUP);

    Wire.begin(deviceID+8);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    Serial.begin(19200);
    //turn the PID on
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(PID_interval);
    myPID.SetOutputLimits(-255, 255);

    while(digitalRead(HOME_SWITCH)){
        motor.setSpeed(60);
        motor.forward();
    }
    myEnc.write(0); 
   
}



void loop() {



    //none blocking PID MOTOR CONTROL stuffs
    if(digitalRead(HOME_SWITCH)){
        PID_currentMillis = millis();
        if (PID_currentMillis - PID_previousMillis >= PID_interval) {
            PID_previousMillis = PID_currentMillis;

            CurrentPosition = double(myEnc.read());
            myPID.Compute();

            //motor control
            if(Output>0){
                motor.setSpeed(Output);
                motor.forward();
            }else if(Output<0){
                motor.setSpeed(-Output);
                motor.backward();
            }else{
                motor.stop();
            }
        }
    }else(
        
    )
    
    
    
    //none blocking DEBUG stuffs
    if (DEBUG) {
        DEBUG_currentMillis = millis();
        if (DEBUG_currentMillis - DEBUG_previousMillis >= DEBUG_interval) {
            DEBUG_previousMillis = DEBUG_currentMillis;
            Serial.print("DEVICE ID \t\t: ");
            deviceID = 0;
            for(int i; i<4;i++){
                deviceID += digitalRead(idPin[i]) << i;
            } 
            Serial.println(deviceID);
            Serial.print("CurrentPosition\t: ");
            Serial.println(CurrentPosition);
            Serial.print("Setpoint \t\t: ");
            Serial.println(Setpoint);
            Serial.print("Output \t\t: ");
            Serial.println(Output);

            Serial.println("-------------------------------");
        }
    }
    
  
}


void requestEvent()
{
  if (CMD == GET_POSITION)
  {
    int position = int(CurrentPosition);
    uint8_t pos[] = {uint8_t(position & 0xFF), uint8_t((position & 0xFF00) >> 4)};
    Wire.write(pos, 2);
  }
}

void receiveEvent(int HowMany)
{
    if (Wire.available() > 0) {
        CMD = Wire.read();
        if (CMD == GET_POSITION || CMD == GET_KPID) {
        //      do nothing
        } else {
            uint8_t i = 0;
            while (Wire.available()) {
                buffer[i] = Wire.read();
                i++;
            }
        }
        if (CMD == SET_POSITION) {
            Setpoint = double(buffer[0] + (buffer[1] << 4));

        } else if (CMD == SET_KPID) {
            myPID.SetTunings(buffer[0]/100.0, buffer[1]/100.0, buffer[2]/100.0);
        } else if (CMD == TEST) {
            if (DEBUG) {
                DEBUG = 0;
                Serial.println("shutdown DEBUG");
            } else {
                DEBUG = 1;
                Serial.println("start DEBUG");
            }
        }
    }
}
