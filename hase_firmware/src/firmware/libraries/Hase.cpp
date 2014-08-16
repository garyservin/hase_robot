#include "Hase.h"
#include <stdarg.h>

Hase::Hase():
    _debug(DBG_TX, DBG_RX),
    _lmotor(PWM_L, FWD_L, REV_L),
    _rmotor(PWM_R, FWD_R, REV_R),
    _lqei(QEIA_L, QEIB_L, NC, cprEncoder, QEI::X4_ENCODING),
    _rqei(QEIA_R, QEIB_R, NC, cprEncoder, QEI::X4_ENCODING),
    _lpid(Kc1, Ti1, Td1, PID_INTERVAL),
    _rpid(Kc1, Ti1, Td1, PID_INTERVAL)
{

    _debug.baud(DEBUG_BAUDRATE);

    _lpid.setInputLimits(-10500.0, 10500.0);
    _lpid.setOutputLimits(-1.0, 1.0);
    _lpid.setBias(0.0);
    _lpid.setMode(AUTO_MODE);
    _lpid.setSetPoint(0.0);

    _rpid.setInputLimits(-10500.0, 10500.0);
    _rpid.setOutputLimits(-1.0, 1.0);
    _rpid.setBias(0.0);
    _rpid.setMode(AUTO_MODE);
    _rpid.setSetPoint(0.0);

    _lmotor.speed(0.0);
    _rmotor.speed(0.0);

    _readEncoderTicker.attach(this, &Hase::readEncoder, PID_INTERVAL);
    _lastMotorCommand.start();
}

// @params lspeed: speed in ticks per second
// @params rspeed: speed in ticks per second
void Hase::setSpeedsTicks(float lspeed, float rspeed)
{
    _lpid.reset();
    _lpid.setSetPoint(lspeed);
    _lpid.setProcessValue(_lpps);
    _lmotor.speed(_lpid.compute());

    _rpid.reset();
    _rpid.setSetPoint(rspeed);
    _rpid.setProcessValue(_rpps);
    _rmotor.speed(_rpid.compute());

    // Reset motor timeout timer
    _lastMotorCommand.reset();
}

// @params lspeed: speed in meters per second
// @params rspeed: speed in meters per second
void Hase::setSpeeds(float lspeed, float rspeed)
{
    double vl = Hase::speedToTicks(lspeed);
    double vr = Hase::speedToTicks(rspeed);
#ifdef DEBUG_ENABLED
    Hase::debug("setSpeeds(m->ticks): %.2f->%.2f, %.2f->%.2f", lspeed, vl, rspeed, vr);
#endif
    Hase::setSpeedsTicks(vl, vr);
}

int Hase::getPulses(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        return _lqei.getPulses() / gearRatio;
    }else{
        return _rqei.getPulses() / gearRatio;
    }
}

int Hase::getPulses(Motors motor){
    if(motor == LEFT_MOTOR){
        return _lqei.getPulses();
    }else{
        return _rqei.getPulses();
    }
}

int Hase::getRevolutions(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        return _lqei.getRevolutions();
    }else{
        return _rqei.getRevolutions();
    }
}

int Hase::getCurrentState(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        return _lqei.getCurrentState();
    }else{
        return _rqei.getCurrentState();
    }
}

double Hase::getWheelSpeed(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        return _lpps / ticksPerMeter;
    }else{
        return _rpps / ticksPerMeter;
    }
}

int Hase::getPulsesPerSecond(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        return _lpps / gearRatio;
    }else{
        return _rpps / gearRatio;
    }
}

int Hase::getPulsesPerSecond(Motors motor){
    if(motor == LEFT_MOTOR){
        return _lpps;
    }else{
        return _rpps;
    }
}

void Hase::resetEncoder(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        _lqei.reset();
    }else{
        _rqei.reset();
    }
}

int Hase::getRPM(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        return ((long) _lpps * 60) / (cpr);
    }else{
        return ((long) _rpps * 60) / (cpr);
    }
}

int Hase::getRPM(Motors motor){
    if(motor == LEFT_MOTOR){
        return ((long) _lpps * 60) / _lqei.getPulsesPerRevolution();
    }else{
        return ((long) _rpps * 60) / _rqei.getPulsesPerRevolution();
    }
}

// Harcoded access to te qei interface
void Hase::readEncoder() {
    int currentPulses = 0;

    currentPulses = _lqei.getPulses();
    _lpps = (currentPulses - _lpulses) * PID_RATE;
    _lpulses = currentPulses;

    _lpid.setProcessValue(_lpps);
    float ltmp = _lpid.compute();
    _lmotor.speed(ltmp);

    currentPulses = _rqei.getPulses();
    _rpps = (currentPulses - _rpulses) * PID_RATE;
    _rpulses = currentPulses;

    _rpid.setProcessValue(_rpps);
    float rtmp = _rpid.compute();
    _rmotor.speed(rtmp);

#ifdef DEBUG_ENABLED
    Hase::debug("%i\t%.2f\t\t|\t%i\t%.2f\t", _lpulses, Hase::ticksToSpeed(_lpps), _rpulses, Hase::ticksToSpeed(_rpps));
    //Hase::debug("%i\t%i\t\t|\t%i\t%i\t", _lpulses, _lpps, _rpulses, _rpps);
    //Hase::debug("%i, %i", _lpulses, _rpps);
#endif

    // Motor timeout
    if(_lastMotorCommand >= AUTO_STOP_INTERVAL){
        Hase::setSpeedsTicks(0, 0);
    }
}

/* Convert meters per second to ticks per second */
int Hase::speedToTicks(float v) {
  return int(v * ticksPerMeter);// * ODOM_INTERVAL);
}

/* Convert ticks per second to meters per second */
float Hase::ticksToSpeed(int ticks) {
  return ticks / ticksPerMeter;
}

// Debug method
int Hase::debug(const char *fmt, ...){
    char buffer[200]= {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 200, fmt, args);
    va_end(args);

    return _debug.printf("[DEBUG] %s\r\n", buffer);
}
