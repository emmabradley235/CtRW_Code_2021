package org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles;

import java.util.ArrayList;


public class SmoothProfile extends FeedforwardProfile { // creates smooth motion via a trapezoidal velocity profile
    // anytime a comment refers to position, velocity, or acceleration, that is assuming the profile output is positional, if it is say a velocity output shift everything back in your mind, the things that do velocity will really be doing acceleration, the things doing acceleration will be doing jerk, etc

    double acceleratingSquareCoefficient, stableLinearCoefficient, deceleratingSquareCoefficient;
    double K1, K2, K3; // the multiplier for the setpoint (ex. number you multiply setpoint velocity by to create the motor power required to reach that setpoint)


    boolean stabalizesAtMaxSpeed = false;
    double maxSpeed;

    double startPos;
    double posAfterAcceleration;
    double posAfterSteadySpeed;
    double posAfterDeceleration;

    double endOfAccelerationTime;
    double endOfConstantSpeedTime;
    double endOfDecelerationTime;


    public SmoothProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK, double tertiaryK, ProfileEndBehavior endBehavior){
        super( 0, endBehavior ); // call the FeedforwardProfile constructor to setup transversal time variables

        this.startPos = startPoint;
        this.maxSpeed = maxSpeed;
        this.posAfterDeceleration = endPoint;
        createVeloProfile(startPoint, endPoint, maxAccelPerSec, maxSpeed); // make the acceleration profile, which can be used to calculate velocity for the output of this profile

        this.K1 = primaryK;
        this.K2 = secondaryK;
        this.K3 = tertiaryK;
    }
    public SmoothProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK, ProfileEndBehavior endBehavior){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, secondaryK, 1.0, endBehavior);
    }
    public SmoothProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, ProfileEndBehavior endBehavior){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, 1.0, 1.0, endBehavior);
    }
    public SmoothProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK, double tertiaryK){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, secondaryK, tertiaryK, ProfileEndBehavior.MAINTAIN);
    }
    public SmoothProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, secondaryK, 1.0, ProfileEndBehavior.MAINTAIN);
    }
    public SmoothProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, 1.0, 1.0, ProfileEndBehavior.MAINTAIN);
    }



    @Override
    public double getPrimaryProfileTarget(double timestep){ // think position
        double rawPosition = 0;
        double timeSinceSectionStart = timestep;


        if (timestep <= 0) // if at or before start, return start
            rawPosition = startPos;

        else if (timestep < endOfAccelerationTime) // if at accelerating part of profile, return appropriate integral (position)
            rawPosition = acceleratingSquareCoefficient * Math.pow(timeSinceSectionStart, 2); // n*x^2

        else if (stabalizesAtMaxSpeed && timestep < endOfConstantSpeedTime) { // if at stable part of profile, return appropriate integral (position)
            timeSinceSectionStart -= endOfAccelerationTime;
            rawPosition = (stableLinearCoefficient * (timeSinceSectionStart)) + posAfterAcceleration; // n*x + posAfterAcceleration
        }

        else if(timestep < endOfDecelerationTime){ // if decelerating part
            double lastEndPos = 0;

            if(stabalizesAtMaxSpeed){ // if stabalizes at a max speed, use the end of that one
                timeSinceSectionStart -= endOfConstantSpeedTime;
                lastEndPos = posAfterSteadySpeed;
            }
            else { // if doesn't stabalize at max speed
                timeSinceSectionStart -= endOfAccelerationTime;
                lastEndPos = posAfterAcceleration;
            }

            rawPosition = (deceleratingSquareCoefficient * Math.pow(timeSinceSectionStart, 2)) + (stableLinearCoefficient * timeSinceSectionStart) + lastEndPos;
        }

        else // if at or beyond end, just return end
            rawPosition = posAfterDeceleration;


        return K1 * rawPosition;
    }


    @Override
    public double getSecondaryProfileTarget(double timestep){ // think velocity
        double rawVelo = 0;


        if (timestep <= 0) // if at or before start, return 0, as stopped at start
            rawVelo = 0;

        else if (timestep < endOfAccelerationTime) // if at accelerating part of profile, return appropriate integral (position)
            rawVelo = (acceleratingSquareCoefficient / 2) * timestep; // the derivative of the first range's function

        else if (stabalizesAtMaxSpeed && timestep < endOfConstantSpeedTime) // if at stable part of profile, return appropriate integral (position)
            rawVelo = stableLinearCoefficient; // the max speed but multiplied by 1 or -1 in accordance with the direction of the velocity

        else if(timestep < endOfDecelerationTime){ // if decelerating part
            double timeSinceSectionStart = timestep;

            if(stabalizesAtMaxSpeed){
                timeSinceSectionStart -= endOfConstantSpeedTime;
            }
            else {
                timeSinceSectionStart -= endOfAccelerationTime;
            }

            rawVelo = (deceleratingSquareCoefficient/2) * timeSinceSectionStart + stableLinearCoefficient; // the derivative of the last range's function
        }

        else // if at or beyond end, just return 0, as stopped
            rawVelo = 0;


        return K2 * rawVelo;
    }


    @Override
    public double getTertiaryProfileTarget(double timestep){ // think accel
        double rawAccel = 0;


        if (timestep <= 0) // if at or before start, return 0, as stopped at start
            rawAccel = 0;

        else if (timestep < endOfAccelerationTime) // if at accelerating part of profile, return appropriate integral (position)
            rawAccel = (acceleratingSquareCoefficient / 2); // the second derivative of the first range's function

        else if (stabalizesAtMaxSpeed && timestep < endOfConstantSpeedTime)  // if at stable part of profile, return appropriate integral (position)
            rawAccel = 0; // no acceleration when no change in velocity

        else if(timestep < endOfDecelerationTime) // if decelerating part
            rawAccel = (deceleratingSquareCoefficient/2); // the second derivative of the last range's function

        else // if at or beyond end, just return 0, as stopped
            rawAccel = 0;


        return K3 * rawAccel;
    }




    private void createVeloProfile(double startPoint, double endPoint, double maxAccelPerSec, double maxSpeed){
        double travelError = endPoint - startPoint;
        double goingUp = Math.abs(travelError) / travelError; // if travel error is positive, going up will be 1, if negative, going up will be -1, to ensure that everything is going in the right direction
        double travelDistance = Math.abs(travelError);


        // TODO: find how far need go before flat
        // we know the time of the start point is 0, and we know the max acceleration per second is the slope of the acceleration graph, so until flattening:
        // speed = maxAccelPerMsec*time
        // and we can put maxSpeed in for speed and solve for time to find when acceleration reaches max speed
        double maxAccelPerMsec = maxAccelPerSec / 1000; // (dividing maxAccelPerSec by 1000 to scale it to be accel per millisecond)
        endOfAccelerationTime = maxSpeed / maxAccelPerMsec;

        // the function of position given a time is simply the integral of the speed function, so:
        // position = (maxAccelPerMsec/2) * time^2
        // which we can plug in maxSpeedReachTime to find how far it has traveled by the time it has fully accelerated, and compare that to half the endpoint to see if it has time to fully accelerate before position gets over half way to the target
        double fullyAcceleratedDistance = (maxAccelPerMsec/2) * Math.pow(endOfAccelerationTime, 2);

        if( travelDistance/2 > fullyAcceleratedDistance){ // if travelDistance/2 is greater than the fully accelerated position, there is a flat component, so find how long it needs to be
            stabalizesAtMaxSpeed = true;

            // the accelerating portion of the profile
            acceleratingSquareCoefficient = goingUp * (maxAccelPerMsec/2); // taking the integral equation from before and storing it
             // no linear component to this integral equation as speed graph had no offset
            posAfterAcceleration = goingUp * fullyAcceleratedDistance; // store where we are at the end of this movement

            // the constant speed portion of the profile
            // finding the integral of this portion:
            // speed = maxSpeed  (the speed function)
            // position = maxSpeed * time  (the integral of the speed function), but be sure to
            double distanceLeftToHalfway = (travelDistance/2) - fullyAcceleratedDistance; // see how much distance is left to the half way point
            double flatSpeedTravelTime = 2 * (distanceLeftToHalfway/maxSpeed); // the amount of time we will spend at flat speed is the amount of time from the fully accelerated point to the midpoint times two
            stableLinearCoefficient = goingUp * (maxSpeed/1000); // this one only has a linear component (no acceleration), speed converted from in/sec to in/msec
            endOfConstantSpeedTime = endOfAccelerationTime + flatSpeedTravelTime; // and this part of the graph goes until time to slow down again
            posAfterSteadySpeed = goingUp * (2 * distanceLeftToHalfway) + posAfterAcceleration; // store where we are at the end of this movement

            // now find the integral of the deceleration portion
            // speed = -maxAccelPerMsec*time + maxSpeed   (the speed function)
            // position = -(maxAccelPerMsec/2) * time^2 + maxSpeed * time   (the integral of the speed function)
            deceleratingSquareCoefficient = goingUp * -(maxAccelPerMsec/2); // taking the integral equation from before and storing it
            endOfDecelerationTime = endOfConstantSpeedTime + endOfAccelerationTime; // and this part of the graph goes until the end
        }
        else{
            stabalizesAtMaxSpeed = false;

            // the accelerating portion of the profile
            acceleratingSquareCoefficient = goingUp * (maxAccelPerMsec/2); // taking the integral equation from before and storing it
            // no linear component to this integral equation as speed graph had no offset
            posAfterAcceleration = goingUp * fullyAcceleratedDistance; // store where we are at the end of this movement

            // now find the integral of the deceleration portion
            // speed = -maxAccelPerMsec*time + maxSpeed   (the speed function)
            // position = -(maxAccelPerMsec/2) * time^2 + maxSpeed * time   (the integral of the speed function)
            deceleratingSquareCoefficient = goingUp * -(maxAccelPerMsec/2); // taking the integral equation from before and storing it
            endOfDecelerationTime = endOfConstantSpeedTime + endOfAccelerationTime; // and this part of the graph goes until the end
        }
    }
}
