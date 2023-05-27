class PIDRatio
{
public:
    Direction *target = new Direction();
    Direction *prevUserIn = new Direction();
    Direction *currUserIn = new Direction();
    Direction *prevFeedback = new Direction();
    Direction *currentFeedback = new Direction();
    Direction *speedReferenceFeedback = new Direction();
    Direction *_feedback = new Direction();
    Direction *t_feedback = new Direction();
    Direction *traveled = new Direction();
    Direction *output = new Direction();
    Direction *UserIn = new Direction();
    optimizer *opt = new optimizer();
    double dist = 0, brakeOut = 0, zero = 0;
    bool braking = false;
    double xKp = 1.0, xKi = 0, xKd = 0;
    double yKp = 1.0, yKi = 0, yKd = 0;
    double rKp = 3.0, rKi = 0, rKd = 0;//3.0
    double brakeKp = 4.0, brakeKi = 0.0, brakeKd = 0;
    int feedbackOffset = 8;
    PID *fxPID, *fyPID, *frPID, *brake;

    PIDRatio() {}
    PIDRatio(Direction *in, Direction *out, Direction *setp)
    {
        set(in, out, setp);
    }
    void set(Direction *in, Direction *out, Direction *setp)
    {
        UserIn = setp;
        _feedback = in;
        output = out;
        opt->set(currUserIn, currentFeedback);
    }
    void setup()
    {
        fxPID = new PID(&currentFeedback->fx, &output->fx, &target->fx, xKp, xKi, xKd, DIRECT);
        fyPID = new PID(&currentFeedback->fy, &output->fy, &target->fy, yKp, yKi, yKd, DIRECT);
        frPID = new PID(&currentFeedback->fr, &output->fr, &target->fr, rKp, rKi, rKd, DIRECT);
        brake = new PID(&dist, &brakeOut, &zero, brakeKp, brakeKi, brakeKd, DIRECT);
        fxPID->SetMode(AUTOMATIC);
        fxPID->SetSampleTime(1);

        fyPID->SetMode(AUTOMATIC);
        fyPID->SetSampleTime(1);

        frPID->SetMode(AUTOMATIC);
        frPID->SetSampleTime(1);

        brake->SetMode(AUTOMATIC);
        brake->SetSampleTime(1);

        fxPID->SetOutputLimits(-255, 255);
        fyPID->SetOutputLimits(-200, 200);
        frPID->SetOutputLimits(-255, 255);
        brake->SetOutputLimits(-255, 255);
    }
    
    void updateYPID(double yKp,double yKi,double yKd)
    {
        this-> yKp = yKp;
        this-> yKi = yKi;
        this-> yKd = yKd;
        fyPID->SetTunings(this->yKp,this->yKi,this->yKd);
        // Serial.println((String)this->yKp+", "+(String)this->yKi+", "+(String)this->yKd);
    }
    void updateRPID(double rKp,double rKi,double rKd)
    {
        this-> rKp = rKp;
        this-> rKi = rKi;
        this-> rKd = rKd;
        frPID->SetTunings(this->rKp,this->rKi,this->rKd);
        // Serial.println((String)this->yKp+", "+(String)this->yKi+", "+(String)this->yKd);
    }
    void updateXPID(double xKp,double xKi,double xKd)
    {
        this-> xKp = xKp;
        this-> xKi = xKi;
        this-> xKd = xKd;
        fxPID->SetTunings(this->xKp,this->xKi,this->xKd);
        // Serial.println((String)this->yKp+", "+(String)this->yKi+", "+(String)this->yKd);
    }

    void updateBrakePID(double brakeKp,double brakeKi,double brakeKd)
    {
        this-> brakeKp = brakeKp;
        this-> brakeKi = brakeKi;
        this-> brakeKd = brakeKd;
        brake->SetTunings(brakeKp,brakeKi,brakeKd);
        Serial.println((String)this->brakeKp+", "+(String)this->brakeKi+", "+(String)this->brakeKd);
    }

    void setFeedbackOffset(int off)
    {
        this-> feedbackOffset = off;
        Serial.println("feed: "+(String)this->feedbackOffset);
    }

    void compute()
    {
            // UserIn->display();
            // _feedback->display();
            UserIn->process();
            // Serial.println(yKp);
            // Braking Logic
            if (UserIn->isZero && !prevUserIn->isZero)// Detecting Brake
            {
                prevUserIn->magnitude = 1;
                prevUserIn->invertProcess();
                *currUserIn = *prevUserIn;
                braking = true;
            }
            else
            {
                //Reducing User Input
                currUserIn->fx = UserIn->fx / 1;
                currUserIn->fy = UserIn->fy / 1;
                currUserIn->fr = UserIn->fr / 1;
                braking = false;  
            }
            *prevUserIn = *currUserIn;


            *t_feedback = *_feedback / feedbackOffset; // test this // 8
            *currentFeedback = *t_feedback - *prevFeedback;
            *speedReferenceFeedback = *currentFeedback;
            currUserIn->process();
            if (!currUserIn->isZero)
            {
                // Serial.println("!isZero");
                // delay(100000);
                opt->minimize();
            }
            currUserIn->process();
            
            *prevFeedback=*t_feedback - *currentFeedback;
            if(braking){
                *traveled = *speedReferenceFeedback - *currentFeedback;
                dist = pow(pow((speedReferenceFeedback->fx - currentFeedback->fx), 2) +
                    pow((speedReferenceFeedback->fy - currentFeedback->fy), 2) +
                    pow((speedReferenceFeedback->fr - currentFeedback->fr), 2), 0.5) * 2;// * 2
                brake->Compute();
                traveled->process();
                traveled->magnitude = brakeOut;
                traveled->invertProcess();
                *currUserIn = *traveled;
            }
            *target=*currUserIn;
            fxPID->Compute();
            fyPID->Compute();
            frPID->Compute(); 
            // output->display();
    }

    Direction *equatePrev()
    {
        if(smartBase.distanceMode){
            _feedback->fy=feedback.valueY;
            _feedback->fr=feedback.valueR;
            *t_feedback = *_feedback / feedbackOffset;
            *prevFeedback= *t_feedback;
            *currentFeedback = *t_feedback - *prevFeedback;
            return currentFeedback;
        }
        else if(!smartBase.distanceMode)
        {
            *t_feedback = *_feedback / feedbackOffset;
            *prevFeedback= *t_feedback;
            *currentFeedback = *t_feedback - *prevFeedback;
            return currentFeedback;
        }
    }
    void storePrev(Direction *_current)
    {
        *currentFeedback = *_current;
    }
} PID_ratio,SB_ratio;