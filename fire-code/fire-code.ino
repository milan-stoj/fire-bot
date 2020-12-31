#include <PID_v1.h> // requires arduino PID libray installed 

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
const int sensortivity = 35;

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

const int motorSpeed = 90;
int deltaSpeed = 0;
long duration, inches, cm;

/***** FIRE SENSOR VARIABLES *****/
const int firePin = A2; // analog pin for IR sensor

// sensor value when fire is in suppressing range.
const int fireValue = 100;
const boolean suppress = false; // boolean for suppressing fire
int sensorValue, rSensorValue, lSensorValue;
const int flameThresh = 80;
const int flameMax = 125;

/***** WATER PUMP SPRAY *****/
// this pin controls the pump, HIGH when there is a "fire" detected.
const int sprayPin = A1;
// define PID Variables
double Setpoint, Input, Output;
// define PID Tuning Variables
double Kp = 0.0, Ki = 500, Kd = 0;
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

    /***** PID SETUP *****/
    Input = deltaSpeed;
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC); // Setting PID mode and parameters.
    myPID.SetSampleTime(1);
    myPID.SetOutputLimits(-30, 30);
    myPID.SetTunings(Kp, Ki, Kd);
}

/***** MAIN LOOP *****/
void loop(void)
{
    distRight = getDistance(rPingPin);
    distCenter = getDistance(cPingPin);
    distLeft = getDistance(lPingPin);

    /***** MOVING FORWARD *****/
    while (
        distLeft > sensortivity &&
        distRight > sensortivity &&
        distCenter > sensortivity)
    {

        checkFire(); // check for flames
        goForward();
        Count1 = 0;
        Count2 = 0;
        distRight = getDistance(rPingPin);
        distCenter = getDistance(cPingPin);
        distLeft = getDistance(lPingPin);
    }

    /***** TURNING RIGHT *****/
    while (
        distLeft < distCenter &&
        distLeft < sensortivity)
    {

        checkFire();
        goRight();
        distLeft = getDistance(lPingPin);
        distCenter = getDistance(cPingPin);
    }

    /***** TURNING LEFT *****/
    while (
        distRight < distCenter &&
        distRight < sensortivity)
    {

        checkFire();
        goLeft();
        distRight = getDistance(rPingPin);
        distCenter = getDistance(cPingPin);
    }

    /***** REVERSING *****/
    while (
        distCenter < distLeft &&
        distCenter < distRight &&
        distCenter < sensortivity)
    {

        checkFire();
        goBack();
        delay(1000);
        if (getDistance(rPingPin) < getDistance(lPingPin))
        {
            goLeft();
            delay(500);
        }
        else
        {
            goRight();
            delay(500);
        }
        distRight = getDistance(rPingPin);
        distCenter = getDistance(cPingPin);
    }
}

    void goForward()
    { 
        // PID section. The error is calculated by taking the two encoder 
        // counts and finding the difference. The difference determines the 
        // direction the PID will drive the robot, and the intensity of the 
        // output on the control wheel.
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
        analogWrite(E1, 80);
        analogWrite(E2, 80);
    }

    void goRight()
    { 
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        analogWrite(E1, 80);
        analogWrite(E2, 80);
    }

    void Stop()
    { 
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        analogWrite(E1, 0);
        analogWrite(E2, 0);
    }

    /***** CODE FOR FIRE SENSOR *****/
    void checkFire()
    { 
        // Checks the fire sensor, and initiates 
        // suppression based on the analog value.
        sensorValue = analogRead(firePin);
        Serial.println(sensorValue);

        if (sensorValue > flameThresh)
        {
            suppress == true;
            Stop();
            flameCenter();
            sprayFire();
        }
        else
        {
            suppress == false;
        }
    }

    /***** CODE FOR FIRE SUPRESSION *****/
    void sprayFire()
    { // Initiates spraying with the pump relay.
        pinMode(sprayPin, OUTPUT);
        while (analogRead(firePin) >= 0.25 * flameThresh)
        {
            digitalWrite(sprayPin, HIGH);
            goLeft();
            delay(100);
            goRight();
            delay(100);
        }
        digitalWrite(sprayPin, LOW);
        checkFire();
    }

    /***** CODE FOR FLAME CENTERING *****/
    void flameCenter()
    {                                      
        // Code for centering on flame source.
        sensorValue = analogRead(firePin); // take sensor value
        delay(500);
        goRight(); // turn right for quarter second.
        delay(250);
        Stop();
        delay(250);
        rSensorValue = analogRead(firePin); // take right sensor value
        goLeft();                           // turn left for quarter second.
        delay(300);
        Stop();
        delay(500);
        lSensorValue = analogRead(firePin); // take left sensor  value

        if (rSensorValue > lSensorValue)
        {
            sensorValue = analogRead(firePin);
            goRight();
            delay(250);
            Stop();
            delay(250);
            rSensorValue = analogRead(firePin);
            while (rSensorValue > sensorValue && sensorValue < flameMax)
            {
                sensorValue = analogRead(firePin);
                goRight();
                // Jogs to the right in increments until the 
                // fire sensor value stops rising.
                delay(150); 
                Stop();
                delay(150);
                rSensorValue = analogRead(firePin);
            }
            Stop();
        }
        else
        {
            sensorValue = analogRead(firePin);
            goLeft();
            delay(250);
            Stop();
            delay(250);
            int lSensorValue = analogRead(firePin);
            while (lSensorValue > sensorValue && sensorValue < flameMax)
            {
                sensorValue = analogRead(firePin);
                goLeft();
                // Jogs to the left in increments until the fire 
                // sensor value stops rising.
                delay(150); 
                Stop();
                delay(150);
                lSensorValue = analogRead(firePin);
            }
            Stop();
        }
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
        // The ping travels out and back, so to find the distance of the 
        // object we take half of the distance travelled.
        return microseconds / 29 / 2;
    }

    long getDistance(int pingPin)
    {
        // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
        // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
        pinMode(pingPin, OUTPUT);
        digitalWrite(pingPin, LOW);
        delayMicroseconds(2);
        digitalWrite(pingPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pingPin, LOW);
        // The same pin is used to read the signal from the PING))): a HIGH
        // pulse whose duration is the time (in microseconds) from the sending
        // of the ping to the reception of its echo off of an object.
        pinMode(pingPin, INPUT);
        duration = pulseIn(pingPin, HIGH);
        // convert the time into a distance
        //inches = microsecondsToInches(duration);
        cm = microsecondsToCentimeters(duration);
        return (cm);
    }

    /***** ENCODER READING *****/
    void EncoderRead1()
    {                                                        
        //Function to read the encoder state once the interrupt occurs
        if (digitalRead(2) == HIGH && digitalRead(8) == LOW) 
        {
            Count1--;
        }
        else
        { //If wheel is not going forward, reduce count
            Count1++;
        }
    }

    void EncoderRead2()
    {                                                        
        if (digitalRead(3) == HIGH && digitalRead(9) == LOW) 
        {
            Count2++;
        }
        else
        { //If wheel is not going forward, reduce count
            Count2--;
        }
    }