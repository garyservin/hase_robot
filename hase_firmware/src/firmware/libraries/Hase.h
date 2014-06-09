#ifndef MBED_HASE_H
#define MBED_HASE_H

#include "mbed.h"
#include <Motor.h>
#include <QEI.h>
#include <PID.h>
#include <ros/time.h>

// Motors
// Right Motor
#define FWD_R   p24
#define REV_R   p25
#define PWM_R   p26
// Left Motor
#define PWM_L   p21
#define FWD_L   p22
#define REV_L   p23

// Right Encoder
#define QEIA_L  p15
#define QEIB_L  p14
// Left Encoder
#define QEIA_R  p12
#define QEIB_R  p13

class Hase {
public:
    /* Rate at which encoders are sampled and PID loop is updated */
    #define PID_RATE        30     // Hz
    const float PID_INTERVAL = 1.0 / PID_RATE;

    /* Odometry publishing rate */
    #define ODOM_RATE       10     // Hz
    const float ODOM_INTERVAL = 1.0 / ODOM_RATE;

    //#define DEBUG

    /* PID Parameters */
    float Kc1 = 1.6;
    float Ti1 = 0.2;
    float Td1 = 0.0;

    /* Define the robot paramters */
    int cprEncoder = 64; // Encoder ticks per revolution for motor
    int gearRatio = 30; // Gear ratio for motor gear
    int cpr = cprEncoder * gearRatio; // Encoder ticks per revolution for the Pololu 29:1 motor
    float wheelDiameter = 0.123825; // meters
    float wheelTrack = 0.23; // meters
    float ticksPerMeter = cpr / (3.141592 * wheelDiameter);

    /* Stop the robot if it hasn't received a movement command
       in this number of milliseconds */
    #define AUTO_STOP_INTERVAL 2000
    long lastMotorCommand = AUTO_STOP_INTERVAL;

    /* Setpoint Info For a Motor */
    typedef struct {
      double TargetSpeed;            // target speed in m/s
      double TargetTicksPerFrame;    // target speed in ticks per frame
      long Encoder;                  // encoder count
      long PrevEnc;                  // last encoder count
      int PrevErr;                   // last error
      int Ierror;                    // integrated error
      int output;                    // last motor setting
    }
    SetPointInfo;

    /* A struct to hold Odometry info */
    typedef struct {
      ros::Time lastOdom;            // last ROS time odometry was calculated
      ros::Time encoderStamp;        // last ROS time encoders were read
      unsigned long encoderTime;     // most recent millis() time encoders were read
      unsigned long lastEncoderTime; // last millis() time encoders were read
      unsigned long lastOdomTime;    // last millis() time odometry was calculated
      long prevLeftEnc;              // last left encoder reading used for odometry
      long prevRightEnc;             // last right encoder reading used for odometry
      float linearX;                 // total linear x distance traveled
      float linearY;                 // total linear y distance traveled
      float angularZ;                // total angular distance traveled
    }
    OdomInfo;

    typedef enum Wheel {

        LEFT_WHEEL,
        RIGHT_WHEEL

    } Wheel;

    typedef enum Motors {

        LEFT_MOTOR,
        RIGHT_MOTOR

    } Motors;

    /** Create a hase control interface
     */
    Hase();

    /** Set the speed of each wheel of the robot
     *
     * @param lspeed The speed of the left wheel as a normalised value between -1.0 and 1.0
     * @param rspeed The speed of the right wheel as a normalised value between -1.0 and 1.0
     */
    void setSpeeds(float lspeed, float rspeed);

    /** Get the count of pulses for the specified wheel of the robot
     *
     * @param wheel The wheel to obtain the pulses from
     * @return The specified wheel's encoder pulses
     */
    long getPulses(Wheel wheel);
    long getPulses(Motors motor);

    /** Get the pulses per revolution of the specified wheel of the robot
     *
     * @param wheel The wheel to obtain the pulses per revolution from
     * @return The specified wheel's pulses per revolution
     */
    int getPulsesPerRevolution(Wheel wheel);
    int getPulsesPerRevolution(Motors motor);

    /** Get the pulses per revolution of from a wheel
     *
     * @param wheel The wheel to obtain the pulses per revolution from
     * @return The converted RPM using the CPR from the specified wheel
     */
    int getRPM(Wheel wheel);
    int getRPM(Motors motor);

    /** Get the count of revolutions for the specified wheel of the robot
     *
     * @param wheel The wheel to obtain the revolutions from
     * @return The specified wheel's encoder revolutions
     */
    int getRevolutions(Wheel wheel);

    /** Convert speed to ticks
     *
     * @param float the peed in meters per second to convert
     * @return The converted ticks
     */
    int speedToTicks(float);

private:
    Motor _lmotor;
    Motor _rmotor;
    QEI _lqei;
    QEI _rqei;
    PID _lpid;
    PID _rpid;

    Ticker _readEncoderTicker;

    long _lpulses;
    long _rpulses;

    int _lpps;
    int _rpps;

    /** Callback executed via the Ticker to read encoder satus
     */
    void readEncoder();

    /** Reset the specified encoder pulse count
     *
     * @param wheel The wheel which encoder's will be reseted
     */
    void resetEncoder(Wheel wheel);

    /** Get the current state of the specified wheel of the robot
     *
     * @param wheel The wheel to obtain the current state from
     * @return The specified wheel's current state
     */
    int getCurrentState(Wheel wheel);
};

#endif
