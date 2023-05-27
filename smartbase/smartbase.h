class smartbase
{
private:
    int  lidarAvg = 0, parallelDistance = 0;
    double parallelInput, parallelSetpoint, parallelOutput, parallelKp = 3.1, parallelKi = 1.0, parallelKd = 0.0; // 2
    // 0.90,0.11,0.0
    // 1.72,2.83,0.0
    double yInput, ySetpoint, yOutput, yKp = 3.0, yKi = 1.0, yKd = 0.0; // 3,1 //1.5,0.8
    double fykp;
    int templidar1, templidar2, previousDistanceCm1 = 0, previousDistanceCm2 = 0;
    int lidarSpeed = 0, lidarSpeedOffset = 0.8, directionalOffset = -1;
    bool lidarRotate = false, stage1 = false, stage2 = false, stage3 = false;

public:
    bool distanceMode = false, pathBase = false, pathBaseDown = false, pathBaseRight = false, rotate = false;
    int desiredDistance=0;
    Direction *inputDir = new Direction();
    Direction *outputDir = new Direction();
    Direction *lidarData = new Direction();
    Direction *realReadings = new Direction();
    // Direction *smFeed = new Direction();
    PID *parallelpid;
    PID *ypid;
    int change = 20;
    int downMidDistance = 80, downShortDistance = 65, downStopDistance = 10;
    int rightMidDistance = 50, rightShortDistance = 80, rightStopDistance = 20;
    int lidarStopDistance = 30, degreeOfRotation = 2740;
    double mpuReadings = 0.0, mpuDifference = 0.0, mpuRead = 0.0;

    smartbase()
    {
        // pidinit();
    }
    void printfun()
    {
        Serial.println("printfun");
    }

    void pidinit()
    {
        parallelpid = new PID(&parallelInput, &parallelOutput, &parallelSetpoint, parallelKp, parallelKi, parallelKd, DIRECT);
        parallelpid->SetMode(AUTOMATIC);
        parallelpid->SetSampleTime(1);
        parallelpid->SetOutputLimits(-100, 100);
        ypid = new PID(&yInput, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);
        ypid->SetMode(AUTOMATIC);
        ypid->SetSampleTime(1);
        ypid->SetOutputLimits(-200, 200);
    }

    void setUserInDirections(Direction *inputdir, Direction *outputdir)
    {
        this->inputDir = inputdir;
        this->outputDir = outputdir;
    }

    void enabledistanceMode()
    {
        distanceMode = true;
        // Serial.println("DistanceMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");
    }
    void disabledistanceMode()
    {
        distanceMode = false;
        outputDir->fx = 0;
        outputDir->fy = 0;
        outputDir->fr = 0;
    }

    void setDesiredDistance(int desiredDist = 20)
    {
        this->desiredDistance = desiredDist;
    }

    void setChange(int setchange)
    {
        this->change = setchange;
    }

    void setLidar(Direction *lidarData)
    {
        this->lidarData = lidarData;
    }

    void setYLimits(int limit1 = -100, int limit2 = 100)
    {
        ypid->SetOutputLimits(limit1, limit2);
    }
    void updateX(double kp, double ki, double kd)
    {
        //     Serial.print(parallelKp);
        //     Serial.print(",");
        //     Serial.print(parallelKi);
        //     Serial.print(",");
        //     Serial.println(parallelKd);
        // Serial.print(yKp);
        // Serial.print(",");
        // Serial.print(yKi);
        // Serial.print(",");
        // Serial.println(yKd);
        // this->parallelKp=kp;
        // this->parallelKi=ki;
        // this->parallelKd=kd;
        // parallelpid->SetTunings(kp,ki,kd);
        this->yKp = kp;
        this->yKi = ki;
        this->yKd = kd;
        ypid->SetTunings(yKp, yKi, yKd);
        // Serial.print(parallelKp);
        // Serial.print(",");
        // Serial.print(parallelKi);
        // Serial.print(",");
        // Serial.println(parallelKd);
        // Serial.println(manualX);
        // Serial.print(yKp);
        // Serial.print(",");
        // Serial.print(yKi);
        // Serial.print(",");
        // Serial.println(yKd);
        // this->rightMidDistance=kp;
        // this->rightShortDistance=ki;
        // this->rightStopDistance=kd;
    }

    void setMpuRead()
    {
        // mpuRead=real->fr;
        mpuRead = realReadings->fr;
        stage1 = false;
        stage2 = false;
        stage3 = false;
        Serial.println(mpuRead);
    }

    void setRealDirection(Direction *realDirection)
    {
        this->realReadings = realDirection;
        // realReadings -> fr = lidarData -> fyd;
    }

    void setDownDistance(int midDist, int shortDist, int stopDist)
    {
        this->downMidDistance = midDist;
        this->downShortDistance = shortDist;
        this->downStopDistance = stopDist;
    }
    void distanceCompute()
    {
        // Serial.println("compute");
        if (distanceMode)
        {
            // Serial.println("distanceMode");
            // parallelInput = lidarData->fr;
            // parallelSetpoint = parallelDistance;
            // yInput = lidarData->fy;
            // ySetpoint = desiredDistance;
            // parallelpid->Compute();//
            // ypid->Compute();
            outputDir->fx = inputDir->fx;
            // outputDir->fy=0;
            // outputDir->fr=parallelOutput *-1 ;
            // outputDir->fy=yOutput;
            outputDir->fy = lidarData->fy - desiredDistance;
            outputDir->fr = 0;
            // outputDir->fr = inputDir->fr;
            // Serial.println(String(outputDir->fr)+","+String(lidarData->fr)+","+String(parallelDistance));
            // outputDir->display();
        }
        else
        {
            outputDir->fx = inputDir->fx;
            outputDir->fr = inputDir->fr;
            outputDir->fy = inputDir->fy;
            // mpuReadings=realReadings->fr;
            // Serial.println(mpuReadings);
            // outputDir->display();
        }
    }
    double PIDOutDiff(int i=2)
    {
        fykp=map(fykp,desiredDistance,lidarData->fy,i,0.75);
        fykp=fykp<0.75?0.75:fykp;
        fykp=fykp>i?i:fykp;
        return fykp;
    }
    void pathComputeDown()
    {
        // Serial.println("pathComputeDown");
        lidarAvg = lidarData->fy;
        // Serial.println(lidarAvg);
        /*   test
        parallelInput=lidarData->fr;
        parallelSetpoint=parallelDistance;
        parallelpid->compute();
        */
        if (lidarAvg > 80)
        {
            // lidarSpeed=lidarAvg*0.5*directionalOffset;
            // lidarSpeed=lidarSpeed>255?255:lidarSpeed;
            // outputDir->fx=0;
            // outputDir->fy=lidarSpeed;
            // outputDir->fr=0;

            if (lidarAvg >= 200)
            {
                outputDir->fx = 0;
                outputDir->fy = -200;
                outputDir->fr = 0;
            }
            else if (lidarAvg < 200 && lidarAvg > 175)
            {
                outputDir->fx = 0;
                outputDir->fy = -175;
                outputDir->fr = 0;
            }
            else if (lidarAvg < 175 && lidarAvg > 150)
            {
                outputDir->fx = 0;
                outputDir->fy = -150;
                outputDir->fr = 0;
            }
            else if (lidarAvg < 150 && lidarAvg > 125)
            {
                outputDir->fx = 0;
                outputDir->fy = -125;
                outputDir->fr = 0;
            }
            else if (lidarAvg < 125 && lidarAvg > 80)
            {
                outputDir->fx = 0;
                outputDir->fy = 50;
                outputDir->fr = 0;
            }

            // else if(lidarAvg<100 && lidarAvg>80)
            // {
            //     outputDir->fx=0;
            //     outputDir->fy=-50;
            //     outputDir->fr=0;
            // }
        }

        else if (lidarAvg <= 80 && lidarAvg > 60)
        {
            // formula

            // lidarSpeed=(lidarAvg-downShortDistance)*4.5;
            // lidarSpeed=lidarSpeed>255?255:lidarSpeed;
            // outputDir->fx=lidarSpeed;
            // outputDir->fy=0;
            // outputDir->fr=0;

            //! formula

            // test   could use but tuning neccessary
            //  yInput=lidarAvg;
            //  ySetpoint=75;
            //  ypid->Compute();
            //  outputDir->fx=100;
            //  outputDir->fy=yOutput;
            //  outputDir->fr=0;
            //! test
            // manual

            if (lidarAvg >= 70)
            {
                outputDir->fx = 200;
                outputDir->fy = 0;
                // outputDir->fr=parallelOutput *-1;//test
                outputDir->fr = 0;
            }
            else if (lidarAvg < 70)
            {
                outputDir->fx = 100;
                outputDir->fy = 0;
                // outputDir->fr=parallelOutput *-1;//test
                outputDir->fr = 0;
            }

            //! manual
        }

        else if (lidarAvg < 60 && lidarAvg > 10)
        {
            lidarSpeed = (lidarAvg - downStopDistance) * 4.5 * directionalOffset;
            outputDir->fx = 0;
            outputDir->fy = lidarSpeed;
            outputDir->fr = 0;
        }
        else if (lidarAvg <= 10)
        {
            outputDir->fx = 0;
            outputDir->fy = 0;
            outputDir->fr = 0;
            // pathBase=false;
        }
        outputDir->display();
    }

    void pathComputeRight()
    {
        // Serial.println("pathComputeRight");
        mpuReadings = realReadings->fr;
        mpuDifference = mpuRead - mpuReadings;
        lidarAvg = lidarData->fy;
        // Serial.println(String(mpuRead)+","+String(mpuReadings)+","+String(abs(mpuDifference))+","+String(lidarAvg));
        // Serial.println(String(stage1)+","+String(stage2)+","+String(stage3)+","+String(rotate));
        if (abs(mpuDifference) > degreeOfRotation && stage1 && !stage2)
        {
            Serial.println("stage2");
            stage2 = true;
            rotate = false;
        }
        else if (!stage1 && stage2 && abs(mpuDifference) > degreeOfRotation && !stage3)
        {
            Serial.println("stage3");
            stage3 = true;
            rotate = false;
        }

        if (lidarAvg < lidarStopDistance && !stage2 && !rotate)
        {
            Serial.println("Ball-Rack");
            outputDir->fx = 0;
            outputDir->fy = 100;
            outputDir->fr = 0;
            stage1 = true;
        }
        else if (abs(mpuDifference) <= degreeOfRotation && stage1)
        {
            Serial.println("First Rotate");
            outputDir->fx = 0;
            outputDir->fy = 0;
            outputDir->fr = 63;
            rotate = true;
        }
        else if (lidarAvg > rightMidDistance && stage2 && !rotate && !stage3)
        {
            Serial.println("After first Rotate");
            outputDir->fx = 0;
            outputDir->fy = -200;
            outputDir->fr = 0;
            stage1 = false;
            mpuRead = realReadings->fr;
        }
        else if (abs(mpuDifference) <= degreeOfRotation && !stage1 && stage2 && !stage3)
        {
            Serial.println("Second rotate");
            outputDir->fx = 0;
            outputDir->fy = 0;
            outputDir->fr = 63;
            rotate = true;
        }
        // else if(!stage1 && lidarAvg>rightMidDistance && !rotate && stage3)
        // {
        //     Serial.println("After Second Rotate");
        //     outputDir->fx=0;
        //     outputDir->fy=-200;
        //     outputDir->fr=0;
        // }
        else if (!stage1 && stage3 && !rotate && stage2)
        {
            if (lidarAvg > 100)
            {
                Serial.println("after-second");
                outputDir->fx = 0;
                outputDir->fy = -200;
                outputDir->fr = 0;
            }
            else if (lidarAvg < 100 && lidarAvg > 50)
            {
                Serial.println("second-last");
                outputDir->fx = 0;
                outputDir->fy = -100;
                outputDir->fr = 0;
            }
            else if (lidarAvg < 50 && lidarAvg > 10)
            {
                Serial.println("last");
                outputDir->fx = 0;
                outputDir->fy = -50;
                outputDir->fr = 0;
            }
            else
            {
                Serial.println("Stop");
                outputDir->fx = 0;
                outputDir->fy = 0;
                outputDir->fr = 0;
                rotate = true;
            }
        }
        // else if(!stage1 && lidarAvg<=rightMidDistance && lidarAvg>rightShortDistance && stage3)
        // {
        //     Serial.println("Last Sequence");
        //     outputDir->fx=0;
        //     outputDir->fy=-100;
        //     outputDir->fr=0;
        // }
        // else if(!stage1 && lidarAvg<rightShortDistance && lidarAvg>rightStopDistance && stage3)
        // {
        //     Serial.println("second-last");
        //     outputDir->fx=0;
        //     outputDir->fy=-50;
        //     outputDir->fr=0;
        // }
        // else if(!stage1 && lidarAvg<=rightStopDistance && stage3)
        // {
        //     Serial.println("last");
        //     outputDir->fx=0;
        //     outputDir->fy=0;
        //     outputDir->fr=0;
        //     //pathBase=false;
        // }
        // else
        // {
        //     outputDir->fx=0;
        //     outputDir->fy=0;
        //     outputDir->fr=0;
        // }
        outputDir->display();
    }

    void pathCompute()
    {
        // Serial.println("pathCompute");
        if (pathBaseDown)
        {
            pathComputeDown();
        }
        else if (pathBaseRight)
        {
            // Serial.println("pathBaseRight");
            pathComputeRight();
        }
    }
}smartBase;