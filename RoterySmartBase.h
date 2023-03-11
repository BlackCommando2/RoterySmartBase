#include <Arduino_JSON.h>
// #include <Wire.h>
#include <PID_v1.h>
#include <Motor.h>
#include <MPU6050_tockn.h>
#include "States/States.cpp"
#include "mpu/mpu.cpp"
#include "optimizer/optimizer.cpp"
#include "virtualBase/virtualBase.cpp"
#include "odometry/odometry.cpp"
#include "encoderFeedback/encoderFeedback.h"
#include "MotorHandler/MotorHandler.cpp"
#include "feedbackHandler/feedbackHandler.cpp"
#include "PIDRatio/PIDRatio.cpp"
// #include <multi_Lidar.h>
#include "smartbase/smartbase.h"
#include "Arduino.h"

class RoterySmartBase
{
public:
    Direction *real = new Direction(); 
    Direction *PID_out = new Direction(); 
    Direction *UserIn = new Direction();
    Direction *smartBaseUserIn = new Direction();
    Direction *lidar = new Direction();
    MotorSpeeds *finalSpeeds = new MotorSpeeds();
    encoderFeedback *efx = new encoderFeedback(); 
    encoderFeedback *efy = new encoderFeedback();
    encoderFeedback *efr = new encoderFeedback();
    Motor *m1, *m2, *m3, *m4;
    UniversalEncoder *encx, *ency, *encr;
    bool virtualMode = false;
    long interval = 10000;
    long prevtime = 0;
    RoterySmartBase() {}
    void setup()
    {
        if (!virtualMode)
        {
            feedback.setup();
            mpu.setOffset(-1);
            feedback.setDirections(real);
            smartBase.disabledistanceMode(); // new
            smartBase.setLidarDirection(lidar);
            smartBase.setRealDirection(real);
            smartBase.setUserInDirections(smartBaseUserIn,UserIn);
            PID_ratio.set(real, PID_out, UserIn);
            PID_ratio.setup();
            OdometryHelper.setDirections(PID_out);
            OdometryHelper.setMotors(finalSpeeds);
            base.setMotorSpeeds(finalSpeeds);
        }
        else
        {
            vbase.set(PID_out, real);
            PID_ratio.set(real, PID_out, UserIn);
            PID_ratio.setup();
        }
    }
    void compute()
    {
        if (micros() - prevtime > interval)
        {
            if (!virtualMode)
            {
                feedback.compute();
            }
            else
            {
                vbase.feedbackCompute();
            }

            if(!smartBase.pathBase)
            {
                // Serial.println("manual");
                smartBase.distanceCompute();
            }
            else if(smartBase.pathBase)
            {
                //Serial.println("auto");
                smartBase.pathCompute();
            }
            
            PID_ratio.compute();

            if (!virtualMode)
            {
                OdometryHelper.compute();
                base.apply();
            }
            else
            {
                vbase.apply();
            }
            prevtime = micros();
        }
    }
    void setMotors(Motor *_m1, Motor *_m2, Motor *_m3, Motor *_m4) //setting all the motors
    {
        virtualMode = false;
        m1 = _m1;
        m2 = _m2;
        m3 = _m3;
        m4 = _m4;

        base.set(m1, m2, m3, m4);
    }
    void setEncoders(UniversalEncoder *_encx, UniversalEncoder *_ency, UniversalEncoder *_encr)
    {
        virtualMode = false;
        efx = new encoderFeedback(_encx);
        efy = new encoderFeedback(_ency);
        efr = new encoderFeedback(_encr);
        feedback.setEncoderXYR(efx, efy, efr);
    }
    void setDirection(Direction *_UserIn)
    {
        smartBaseUserIn = _UserIn;
        setup();
    }
    void enableVirtualMode(bool _virtualMode = true)
    {
        virtualMode = _virtualMode;
        setup();
    }
    Direction *getFeedbackRef()
    {
        return real;
    }
    Direction *getOdomRef()
    {
        return PID_out;
    }
    Direction *getUserInRef()
    {
        return UserIn;
    }
    Direction *getSmartBaseUserInRef()
    {
        return smartBaseUserIn;
    }

    Direction *getLidarDataInRef()
    {
        return lidar;
    }

    MotorSpeeds *getFinalSpeedsRef()
    {
        return finalSpeeds;
    }
};