class feedbackHandler
{

public:
    Direction *directions = new Direction();
    encoderFeedback *encX = new encoderFeedback();
    encoderFeedback *encY = new encoderFeedback();
    encoderFeedback *encR = new encoderFeedback();
    smartbase *sm = new smartbase();
    Direction *feedLidar = new Direction();
    double offset = 2.06;
    int valueX,valueY,valueR;
    // bool enablePath=false;
    void setDirections(Direction *d)
    {
        this->directions = d;
    }
    void setup()
    {
        mpu.setup();
    }
    void setEncoderXYR(encoderFeedback *x, encoderFeedback *y, encoderFeedback *r)
    {
        encX = x;
        encY = y;
        encR = r;
    }

    // void setPathOn()
    // {
    //     enablePath = true;
    // }

    // void setPathOff()
    // {
    //     enablePath = false;
    // }

    // void setSmartBase(smartbase *s)
    // {
    //     this-> sm = s;
    // }

    void setLidar(Direction *lidarData)
    {
        this->feedLidar = lidarData;
    }

    void compute()
    {
        directions->fx = encX->getReadings();
        if (!smartBase.distanceMode)
        {
            directions->fy = encY->getReadings();
            // Serial.println("E:\tX:"+String(directions -> fx)+"\tY:"+String(directions -> fy )+"\tR:"+String(directions -> fr));
        }
        else if (smartBase.distanceMode)
        {
            // Serial.println("D:\tX:"+String(directions -> fx)+"\tY:"+String(directions -> fy )+"\tR:"+String(directions -> fr));
            directions->fy = feedLidar->fy;
            valueY=encY->getReadings();
            // Serial.println("feed: "+(String)directions->fy);
        }
        // directions -> fr = 1*(encR->getReadings() - encX->getReadings());
        if (!smartBase.distanceMode)
        {
          directions->fr = mpu.getReadings() * 2;
            // Serial.println("E:\tX:"+String(directions -> fx)+"\tY:"+String(directions -> fy )+"\tR:"+String(directions -> fr));
        }
        else if (smartBase.distanceMode)
        {
            // Serial.println("D:\tX:"+String(directions -> fx)+"\tY:"+String(directions -> fy )+"\tR:"+String(directions -> fr));
            directions->fr = feedLidar->fr;
            valueR=mpu.getReadings() * 2;
            // Serial.println("feed: "+(String)directions->fy);
        }
        
        // Serial.println("X:"+String(directions -> fx)+"\tY:"+String(directions -> fy )+"\tR:"+String(directions -> fr));
        // Serial.println("Original: "+ (String)mpu.getOrignalReadings());
    }

    operator String()
    {
        return "fx=" + String(directions->fx) + " fy=" + String(directions->fy) + " fr=" + String(directions->fr);
    }

} feedback;