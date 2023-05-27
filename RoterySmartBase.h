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
#include "smartbase/smartbase.h"
#include "feedbackHandler/feedbackHandler.cpp"
#include "PIDRatio/PIDRatio.cpp"
// #include <multi_Lidar.h>

#include "Arduino.h"

class RoterySmartBase
{
public:
    // smartbase *smartBase = new smartbase();
    // smartbase *tempSM = new smartbase();
    Direction *real = new Direction(); 
    Direction *PID_out = new Direction(); 
    Direction *UserIn = new Direction();
    Direction *smartBaseUserIn = new Direction();
    Direction *lidar = new Direction();
    Direction *lidarDiff = new Direction();
   Direction *tempData1 = new Direction();
    Direction *tempData2 = new Direction();
    MotorSpeeds *finalSpeeds = new MotorSpeeds();
    encoderFeedback *efx = new encoderFeedback(); 
    encoderFeedback *efy = new encoderFeedback();
    encoderFeedback *efr = new encoderFeedback();

    Motor *m1, *m2, *m3, *m4;
    UniversalEncoder *encx, *ency, *encr;
    double tempKp;
    bool virtualMode = false,DisMode= true;
    long interval = 10000;
    long prevtime = 0;
    RoterySmartBase() {}
    void setup()
    {
        if (!virtualMode)
        {
            // smartBase.pidinit();
            // feedback.setSmartBase(smartBase);
            feedback.setLidar(lidar);
            feedback.setup();
            mpu.setOffset(1);
            feedback.setDirections(real);
            smartBase.disabledistanceMode();
            smartBase.setLidar(lidar);
            smartBase.setRealDirection(real);
            smartBase.setUserInDirections(smartBaseUserIn,UserIn);
            PID_ratio.set(real, PID_out, UserIn);
            PID_ratio.setup();
            SB_ratio.set(real, PID_out, UserIn);
            SB_ratio.setup();
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
            smartBase.distanceCompute();
            // if(!smartBase.pathBase)
            // {
            //     // Serial.println("manual");
            //     smartBase.distanceCompute();
            // }
            // else if(smartBase.pathBase)
            // {
            //     //Serial.println("auto");
            //     smartBase.pathCompute();
            // }
            if(smartBase.distanceMode)
            {
                SB_ratio.compute();
            }
            else if(!smartBase.distanceMode)
            {
                PID_ratio.compute();
            }

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
    void changeDisMode()
    {
        // Serial.println(lidar->fy-50);
        if(!DisMode)
        {
            tempData1 = PID_ratio.equatePrev();
            // PID_ratio.storePrev(tempData2);
            smartBase.disabledistanceMode();
            DisMode=!DisMode;

        }
        else if(DisMode)
        {
            tempKp=smartBase.PIDOutDiff();
            SB_ratio.updateYPID(tempKp,0.0,0.0);
            SB_ratio.updateRPID(6,0.0,0.0);
            tempData2=SB_ratio.equatePrev();
            SB_ratio.storePrev(tempData1);
            smartBase.setDesiredDistance(50);
            smartBase.enabledistanceMode();
            DisMode=!DisMode;  

        }
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
    Direction *getLidarDataDiffInRef()
    {
        return lidarDiff;
    }

    MotorSpeeds *getFinalSpeedsRef()
    {
        return finalSpeeds;
    }
};