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

// Gyro yaw axis
#define GYR_YAW p17

#define SYSLED LED1

// DEBUG
#define DEBUG_ENABLED
#define DEBUG_XBEE

// Serial debug interface
#ifdef DEBUG_XBEE
#define DBG_TX   p28
#define DBG_RX   p27
#define DEBUG_BAUDRATE    115200
#else
#define DBG_TX   USBTX
#define DBG_RX   USBRX
#define DEBUG_BAUDRATE    57600
#endif

class Hase {
public:
    // SYS led blinking rate
    #define SYSLED_RATE        5     // Hz
    const float SYSLED_INTERVAL = 1.0 / SYSLED_RATE;

    // Rate at which encoders are sampled and PID loop is updated
    #define PID_RATE        30     // Hz
    const float PID_INTERVAL = 1.0 / PID_RATE;

    // PID Parameters
    float Kc1 = 1.6;
    float Ti1 = 0.2;
    float Td1 = 0.0;

    // Define the robot paramters
    int cprEncoder = 64; // Encoder ticks per revolution for motor
    int gearRatio = 30; // Gear ratio for motor gear
    int cpr = cprEncoder * gearRatio; // Encoder ticks per revolution for the Pololu 30:1 motor (1920)
    float wheelDiameter = 0.123825; // meters
    float wheelTrack = 0.23; // meters
    float ticksPerMeter = cpr / (3.141592 * wheelDiameter); // ~4935.635851

    // Stop the robot if it hasn't received a movement command in this number of milliseconds
    #define AUTO_STOP_INTERVAL 2.0

    typedef enum Wheel {

        LEFT_WHEEL,
        RIGHT_WHEEL

    } Wheel;

    typedef enum Motors {

        LEFT_MOTOR,
        RIGHT_MOTOR

    } Motors;

    #define _DEG2RAD 0.01745331111
    #define _GYRO_SCALE 0.0005 // Gyroscope scale is 0.5 mV/dps (1x OUT)

    float gyrz_offset = 0.372727;

    /** Create a hase control interface
     */
    Hase();

    /** Set the speed of each wheel of the robot
     *
     * @param lspeed The speed of the left wheel in ticks per second
     * @param rspeed The speed of the right wheel in ticks per second
     */
    void setSpeedsTicks(float lspeed, float rspeed);

    /** Set the speed of each wheel of the robot
     *
     * @param lspeed The speed of the left wheel in meters per second
     * @param rspeed The speed of the right wheel in meters per second
     */
    void setSpeeds(float lspeed, float rspeed);

    /** Get the count of pulses for the specified wheel of the robot
     *
     * @param wheel The wheel to obtain the pulses from
     * @return The specified wheel's encoder pulses
     */
    int getPulses(Wheel wheel);
    int getPulses(Motors motor);

    /** Get the pulses per revolution of the specified wheel of the robot
     *
     * @param wheel The wheel to obtain the pulses per revolution from
     * @return The specified wheel's pulses per revolution
     */
    int getPulsesPerSecond(Wheel wheel);
    int getPulsesPerSecond(Motors motor);

    /** Get the pulses per revolution of from a wheel
     *
     * @param wheel The wheel to obtain the pulses per revolution from
     * @return The converted RPM using the CPR from the specified wheel
     */
    int getRPM(Wheel wheel);
    int getRPM(Motors motor);

    /** Get the wheel's linear speed in meters per second
     *
     * @param wheel The wheel to obtain the speed from
     * @return The specified wheel's speed (linear)
     */
    double getWheelSpeed(Wheel wheel);

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

    /** Convert ticks per second to meter per seconds
     *
     * @param int ticks per second to convert
     * @return The converted speed in meters per second
     */
    float ticksToSpeed(int ticks);

    /** Get the yaw angular velocity in radians per second
     *
     * @param wheel The wheel to obtain the speed from
     * @return The specified wheel's speed (linear)
     */
    float getYawSpeed();

    int debug(const char *fmt, ...);

private:
    Serial _debug;
    Motor _lmotor;
    Motor _rmotor;
    QEI _lqei;
    QEI _rqei;
    PID _lpid;
    PID _rpid;
    AnalogIn _gyroYaw;
    DigitalOut _sysLed;

    Ticker _readEncoderTicker;
    Ticker _sysLedTicker;
    Timer _lastMotorCommand;

    int _lpulses;
    int _rpulses;

    int _lpps;
    int _rpps;

    // Yaw rotation speed in milivolts
    float _yaw_vel;

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

    /** Calibrates the imu
     */
    void calibrateImu();

    /** Led blink callback
     */
    void sysBlink();

};

#endif
