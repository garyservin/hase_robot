#include "Hase.h"

Hase::Hase():
    _lmotor(PWM_L, FWD_L, REV_L),
    _rmotor(PWM_R, FWD_R, REV_R),
    _lqei(QEIA_L, QEIB_L, NC, cprEncoder, QEI::X4_ENCODING),
    _rqei(QEIA_R, QEIB_R, NC, cprEncoder, QEI::X4_ENCODING),
    _lpid(Kc1, Ti1, Td1, PID_INTERVAL),
    _rpid(Kc1, Ti1, Td1, PID_INTERVAL)
{

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

    _readEncoderTicker.attach(this, &Hase::readEncoder, ODOM_INTERVAL);
}

// @params lspeed: speed in ticks per second
// @params rspeed: speed in ticks per second
void Hase::setSpeeds(float lspeed, float rspeed)
{
    _lpid.reset();
    _lpid.setSetPoint(lspeed);
    _lmotor.speed(lspeed / 10500.0);

    _rpid.reset();
    _rpid.setSetPoint(rspeed / 10500.0);
    _rmotor.speed(rspeed);

#ifdef DEBUG
//    printf("Set point to L: %.2f\tR: %.2f\r\n", lspeed * 10500, rspeed * 10500);
#endif
}

long Hase::getPulses(Wheel wheel){
    if(wheel == LEFT_WHEEL){
        return _lpulses / gearRatio;
    }else{
        return _rpulses / gearRatio;
    }
}

long Hase::getPulses(Motors motor){
    if(motor == LEFT_MOTOR){
        return _lpulses;
    }else{
        return _rpulses;
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
    _lpps = _lqei.getPulses();
    _lqei.reset();
    _lpulses += _lpps;
    // Calculate speed in pulses per second
    _lpps *= ODOM_RATE;

    _lpid.setProcessValue(_lpps);
    float ltmp = _lpid.compute();
    _lmotor.speed(ltmp);

    _rpps = _rqei.getPulses();
    _rqei.reset();
    _rpulses += _rpps;
    // Calculate speed in pulses per second
    _rpps *= ODOM_RATE;

    _rpid.setProcessValue(_rpps);
    float rtmp = _rpid.compute();
    _rmotor.speed(rtmp);

#ifdef DEBUG
    printf("%i\t%i\t%.2f\t|\t%i\t%i\t%.2f\r\n", _lpulses, _lpps, ltmp, _rpulses, _rpps, rtmp);
#endif
}

/* Convert meters per second to ticks per time frame */
int Hase::speedToTicks(float v) {
  return int(v * ticksPerMeter * ODOM_INTERVAL);
}

