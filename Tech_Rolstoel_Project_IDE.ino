// I2Cdev and MPU6050 must be installed as libraries https://github.com/AritroMukherjee/MPU5060sensor
// Check serial port under tools
// Check baud, 115200, in serial monitor

#include <I2Cdev.h>
#include <stdio.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

MPU6050 mpu;
// ================================================================
// ===                    CHOOSE WHICH DATA                     ===
// ================================================================

//#define SEE_DATA_STREAM
//#define SEE_DISTANCE_TRAVELLED
//#define SEE_SPEED
#define SEE_DISTRAVSPEED

// ================================================================
// ===                      ALL VARIABLES                       ===
// ================================================================

// LED for activity
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// wheelcircumference imput
char diameter[5];        // char diameter of wheel in inches
float circumference = 0; // circumference of wheel in cm
byte number = 0;         // byte diameter of wheel in inches

// integers of ypr
int ypr_0; // yaw angle in degrees
int ypr_1; // pitch angle in degrees
int ypr_2; // roll angle in degrees

// calculate distance travelled
int prevypr_1 = 0;       // previous pitch angle measurement
int prevypr_2 = 0;       // previous roll angle measurement
int passes_1 = 0;        // passes of the pitch angle past 0
int passes_2 = 0;        // passes of the roll angle past 0
float distrav_m = 0;     // distance travelleld in meter
float prevdistrav_m = 0; // previous distance travelleld in meter

// calculate speed
unsigned long time_begin; // time begin for delta(t)
unsigned long time_end;   // time end for delta(t)
int index = 0;
float distrav_begin = 0; // distance travelled begin for delta(x)
float distrav_end = 0;   // distance travelled end for delta(x)
float average = 0;       // average speed over 3 seconds

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial)
        ;

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));                                                               // <---- uncomment to see start procedure
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));                                                             // <---- uncomment to see start procedure
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));     // <---- uncomment to see start procedure

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));                                                                       // <---- uncomment to see start procedure
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for MPU6050

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));                                                                       // <---- uncomment to see start procedure
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));                        // <---- uncomment to see start procedure
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));                                             // <---- uncomment to see start procedure
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // 1 = initial memory load failed                                                                             // (if it's going to break, usually the code will be 1)
        // 2 = DMP configuration updates failed
        //Serial.print(F("DMP Initialization failed (code "));                                                        // <---- uncomment to see start procedure
        //Serial.print(devStatus);                                                                                    // <---- uncomment to see start procedure
        //Serial.println(F(")"));                                                                                     // <---- uncomment to see start procedure
    }
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // ask for wheelcircumference in inches
    Serial.println(" ");
    Serial.println("Wat is de diameter van je rolstoel in inches?");
    while (!Serial.available())
        ; // <---- comment to be able to use serial plotter
    Serial.readBytesUntil('\n', diameter, 2);
    number = atoi(diameter);
    circumference = number * M_PI * 2.54;
    Serial.print("Je wielomtrek is ");
    Serial.print(circumference);
    Serial.println(" cm");
    Serial.println(" ");
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));                                                                        // <---- uncomment to see start procedure

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // set integers yaw, pitch and roll
        ypr_0 = ypr[0] * 180 / M_PI;
        ypr_1 = ypr[1] * 180 / M_PI;
        ypr_2 = ypr[2] * 180 / M_PI;

// print ypr angles
#ifdef SEE_DATA_STREAM
        Serial.print("ypr\t");
        Serial.print(ypr_0);
        Serial.print("\t");
        Serial.print(ypr_1);
        Serial.print("\t");
        Serial.println(ypr_2);
#endif

        // ================================================================
        // ===              CALCULATE DISTANCE TRAVELLED                ===
        // ================================================================

        // count passes past 0 pitch and roll
        if ((prevypr_1 <= 0 && ypr_1 >= 0 && prevypr_1 != ypr_1) || (prevypr_1 >= 0 && ypr_1 <= 0 && prevypr_1 != ypr_1))
        {
            passes_1 += 1;
        }
        if ((prevypr_2 <= 0 && ypr_2 >= 0 && prevypr_2 != ypr_2) || (prevypr_2 >= 0 && ypr_2 <= 0 && prevypr_2 != ypr_2))
        {
            passes_2 += 1;
        }

        // calculate distance travelled
        distrav_m = ((passes_1 + passes_2) / 4 * circumference / 100);
        if (prevdistrav_m != distrav_m)
        {
#ifdef SEE_DISTANCE_TRAVELLED
            Serial.print("Je hebt ");
            Serial.print(distrav_m);
            Serial.println(" meter afgelegd.");
#endif
        }

        // set new previous integers
        prevypr_1 = ypr_1;
        prevypr_2 = ypr_2;
        prevdistrav_m = distrav_m;

        // ================================================================
        // ===                     CALCULATE SPEED                      ===
        // ================================================================

        if (index >= 199)
        {
            // update variables
            distrav_end = distrav_m;
            time_end = millis();
            average = ((distrav_end - distrav_begin) / ((time_end - time_begin) / 1000) * 3.6);

// print speed
#ifdef SEE_SPEED
            Serial.print("Je gaat ");
            Serial.print(average);
            Serial.println(" km/h");
#endif

// print speed and distance travelled
#ifdef SEE_DISTRAVSPEED
            Serial.print("Je hebt ");
            Serial.print(distrav_m);
            Serial.print(" meter afgelegd en je gaat ");
            Serial.print(average);
            Serial.println(" km/h");
#endif

            // reset counts
            index = 0;

            // update variables
            distrav_begin = distrav_m;
            time_begin = millis();
        }
        else
        {
            // count plus 1
            index += 1;
        }

        // ================================================================
        // ===                      LED BLINKING                        ===
        // ================================================================
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
