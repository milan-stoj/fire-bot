#include <PID_v1.h>

/***** GLOBAL VARIABLES *****/
//standard PWM DC control
int E1 = 5; //M1 Speed Control
int E2 = 6; //M2 Speed Control
int M1 = 4; //M1 Direction Control
int M2 = 7; //M1 Direction Control

// define Variables to count encoder pulses
volatile signed long Count1 = 0;
volatile signed long Count2 = 0;

// distance in CM before object is in tracking-range ("sensor-tivity")
const int sensortivity = 50;

// setting pins for ultrasonic sensors ("ping-ers")
const int rPingPin = 10;
const int cPingPin = 11;
const int lPingPin = 12;

// pin for IR sensor
const int flamePin = 2;

// variables for storing ping-distances
int distRight = 0;
int distCenter = 0;
int distLeft = 0;

const int motorSpeed = 100;
int deltaSpeed = 0;
long duration, inches, cm;

/***** FIRE SENSOR VARIABLES *****/
const int firePin = 2; // analog pin for IR sensor

// sensor value when fire is in suppressing range.
const int fireValue = 100;      
const boolean suppress = false; // boolean for suppressing fire
int sensorValue;
const int flameThresh = 10;
const int flameMax = 300;

/***** WATER PUMP SPRAY *****/
// this pin controls the pump, HIGH when there is a "fire" detected.
const int sprayPin = A1;
// define PID Variables
double Setpoint, Input, Output;                            
// define PID Tuning Variables
double Kp = 0.0, Ki = 200, Kd = 0;                         
// define PID Links
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); 
double Error;

void setup(void)
{
    // set Up Interrupts for Encoder Reads.
    // function EncoderRead1 is called when the interrupt state rises
    attachInterrupt(digitalPinToInterrupt(2), EncoderRead1, RISING);
    attachInterrupt(digitalPinToInterrupt(3), EncoderRead2, RISING);
    for (int i = 4; i <= 7; i++)
        pinMode(i, OUTPUT);

    Serial.begin(9600); // set Baud Rate
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);

    Input = deltaSpeed;
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(1);
    myPID.SetOutputLimits(-50, 155);
    myPID.SetTunings(Kp, Ki, Kd);
    distRight = getDistance(rPingPin);
    distCenter = getDistance(cPingPin);
    distLeft = getDistance(lPingPin);
}

/***** MAIN LOOP *****/
void loop(void)
{
    /***** MOVING FORWARD *****/
    while (distLeft > sensortivity && distRight > sensortivity && distCenter > sensortivity && sensorValue <= flameThresh)
    { // while sensors are within sensortivity value
        goForward();
        Count1 = 0;
        Count2 = 0;
        distRight = getDistance(rPingPin);
        distCenter = getDistance(cPingPin);
        distLeft = getDistance(lPingPin);
        delay(10);
        sensorValue = analogRead(flamePin);
        if (sensorValue > flameThresh)
        {
            Stop();
        }
    }

    /***** TURNING RIGHT *****/
    while (distLeft < distCenter && distLeft < sensortivity && sensorValue < flameThresh)
    {
        goRight();
        distLeft = getDistance(lPingPin);
        distCenter = getDistance(cPingPin);
        sensorValue = analogRead(flamePin);
        if (sensorValue > flameThresh)
        {
            Stop();
        }
    }

    /***** TURNING LEFT *****/
    while (distRight < distCenter && distRight < sensortivity && sensorValue < flameThresh)
    {
        goLeft();
        distRight = getDistance(rPingPin);
        distCenter = getDistance(cPingPin);
        sensorValue = analogRead(flamePin);
        if (sensorValue > flameThresh)
        {
            Stop();
        }
    }

    /***** REVERSING *****/
    while (distCenter < distLeft && distCenter < distRight && distCenter < sensortivity && sensorValue < flameThresh)
    {
        goBack();
        delay(750);
        if (getDistance(rPingPin) < getDistance(lPingPin))
        {
            goLeft();
            delay(250);
        }
        else
        {
            goRight();
            delay(250);
        }
        distRight = getDistance(rPingPin);
        distCenter = getDistance(cPingPin);
        sensorValue = analogRead(flamePin);
        if (sensorValue > flameThresh)
        {
            Stop();
        }
    }

    /***** ENTERING FLAME SUPPRESSION *****/
    sensorValue = analogRead(flamePin);
    while (sensorValue > flameThresh)
    {
        Stop();
        int sensorValue = analogRead(firePin);
        delay(500);
        goRight();
        delay(100);
        Stop();
        delay(500);
        int rightSensorValue = analogRead(firePin);
        goLeft();
        delay(100);
        Stop();
        delay(500);
        int leftSensorValue = analogRead(firePin);

        if (rightSensorValue > leftSensorValue)
        {
            sensorValue = analogRead(firePin);
            goRight();
            delay(200);
            Stop();
            delay(500);
            int rightSensorValue = analogRead(firePin);
            while (rightSensorValue > sensorValue && sensorValue < flameMax)
            {
                sensorValue = analogRead(firePin);
                goRight();
                delay(100);
                Stop();
                delay(100);
                rightSensorValue = analogRead(firePin);
            }
            Stop();
        }
        else
        {
            sensorValue = analogRead(firePin);
            goLeft();
            delay(200);
            Stop();
            delay(500);
            int leftSensorValue = analogRead(firePin);
            while (leftSensorValue > sensorValue && sensorValue < flameMax)
            {
                sensorValue = analogRead(firePin);
                goLeft();
                delay(100);
                Stop();
                delay(100);
                leftSensorValue = analogRead(firePin);
            }
            Stop();
        }

        // Centered on flame at this point
        delay(500);
        delay(250);
        while (true)
        {
            Stop();
        }

        if (newSensorVal < sensorValue){
            goRight();
            delay(100);
            Stop();
        }
    }
}

/***** MOTION FUNCTIONS *****/
void goForward()
{
    deltaSpeed = Count2 - Count1;
    Input = deltaSpeed;
    myPID.Compute();
    Error = Setpoint - Input;
    Serial.println(Error);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);

    analogWrite(E1, motorSpeed + Output);
    analogWrite(E2, motorSpeed - Output);
}

void flameCrawl()
{
    deltaSpeed = Count2 - Count1;
    Input = deltaSpeed;
    myPID.Compute();
    Error = Setpoint - Input;
    Serial.println(Error);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);

    analogWrite(E1, 60 + Output);
    analogWrite(E2, 60 - Output);
}

void goBack()
{
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(E1, motorSpeed);
    analogWrite(E2, motorSpeed);
}

void goLeft()
{
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 100);
    analogWrite(E2, 100);
}

void goRight()
{
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, 100);
    analogWrite(E2, 100);
}

void Stop()
{
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, 0);
    analogWrite(E2, 0);
}

/***** ULTRASONIC SENSORS *****/
long microsecondsToInches(long microseconds)
{
    // According to Parallax's datasheet for the PING))), there are 73.746
    // microseconds per inch (i.e. sound travels at 1130 feet per second).
    // This gives the distance travelled by the ping, outbound and return,
    // so we divide by 2 to get the distance of the obstacle.
    // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
    return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
    // The speed of sound is 340 m/s or 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the object we
    // take half of the distance travelled.
    return microseconds / 29 / 2;
}

long getDistance(int pingPin)
{

    // The PING is triggered by a HIGH pulse of 2 or more microseconds.
    // Gives a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
    // The same pin is used to read the signal from the PING: a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
    // convert the time into a distance
    // inches = microsecondsToInches(duration);
    cm = microsecondsToCentimeters(duration);
    return (cm);
}

/*****ENCODER FUNCTIONS*****/
// function to read the encoder 1 state once the interrupt occurs
void EncoderRead1()
{
    // based on the forward square waves generated by the encoder.
    if (digitalRead(2) == HIGH && digitalRead(8) == LOW)  See diagram
    {
        Count1--;
    }
    else
    { // if wheel is not going forward, reduce count
        Count1++;
    }
}

void EncoderRead2()
{                                                        
    // function to read the encoder 2 state once the interrupt occurs
    // based on the forward square waves generated by the encoder. 
    if (digitalRead(3) == HIGH && digitalRead(9) == LOW) 
    {
        Count2++;
    }
    else
    { // if wheel is not going forward, reduce count
        Count2--;
    }
}
