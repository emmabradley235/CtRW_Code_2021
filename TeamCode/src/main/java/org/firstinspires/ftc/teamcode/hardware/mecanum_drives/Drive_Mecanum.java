package org.firstinspires.ftc.teamcode.hardware.mecanum_drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Math.AngleMath;


public class Drive_Mecanum {
    ElapsedTime localRuntime;

    // Default speed modifier values
    private static double DEFAULT_TURN_MULTIPLIER = 1.0; // default values to use in the event no custom values are passed
    private static double DEFAULT_TRANSLATE_MULTIPLIER = 1.0;

    // Create and initialize speed modifier variables - using default values (then can be set via a constructor)
    private double turnMultiplier = DEFAULT_TURN_MULTIPLIER; // what percentage of maximum turning speed should be used as a base turning speed (50% = 0.5, etc) - a multiplier
    private double translateMultiplier = DEFAULT_TRANSLATE_MULTIPLIER; // what percentage of maximum translational speed should be used as a base translational speed (50% = 0.5, etc) - a multiplier

    // turn PID coeficients
    public static final double turnKp = 3.0;
    public static final double turnKi = 0.0;
    public static final double turnKd = 20.0;
    // turn PID local variables
    private double lastError = 0.0;
    private double lastRuntime = 0.0;
    private double integral = 0.0;


    //Motor variables
    private DcMotor driveFL, driveFR, driveBL, driveBR; // motors that are being used for mecanum driving


    // Default constructor
    public Drive_Mecanum(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        localRuntime = new ElapsedTime();

        // setup motors from passed motors
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;
    }

    // Secondary constructor that can be used to pass different speed divisor values
    public Drive_Mecanum(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR, double turnSpeed, double translateSpeed){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        localRuntime = new ElapsedTime();

        // setup motors from passed motors
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;

        // set custom values to the speed divisors
        turnMultiplier = turnSpeed;
        translateMultiplier = translateSpeed;
    }


    // Drive functions
    /**
     * Drives the robot relative to the field according to the given commands (powers) for each axis
     * @param x the x movement command, between -1 and 1
     * @param y the y movement command, between -1 and 1
     * @param r the r movement command, between -1 and 1
     * @param currentHeading the current heading of the robot in RADIANS, used for converting the field relative commands to robot relative commands so they can be applied
     */
    public void driveFieldRelative(double x, double y, double r, double currentHeading) { // this drives relative to field (+x is forward, +y is left, heading is in radians)
        // if using controller inputs, ensure you reverse the y on the stick input before passing into this method because down on the stick is positive and up is negative, and we need that to be the opposite way

        double heading = currentHeading * -1; // multiplied by -1 because that's how the code was written by me several years ago (and it works), but you could go through and alter the formulas to make it equivalent without requiring the negation :)
        heading = AngleMath.clipAngle(heading, AngleUnit.RADIANS);

        // Set up heading factor for relative to robot (convert the heading to radians, then get the sine and cosine of that radian heading)
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        // do math to adjust to make the input drive vector relative to the robot (rather than relative to field)
        double robotRelativeX = (y * cos) - (x * sin);
        double robotRelativeY = (y * sin) + (x * cos);


        driveRobotRelative(robotRelativeX, robotRelativeY, r); // send the newly robot relative commands into the robot relative drive function
    }

    /**
     * Drives the robot (relative to itself) according to the given commands (powers) for each axis
     * @param x the x movement command, between -1 and 1
     * @param y the y movement command, between -1 and 1
     * @param r the r movement command, between -1 and 1
     */
    public void driveRobotRelative(double x, double y, double r) {
        // apply the cartesian mecanum formula
        double veloFL = y + r + x;
        double veloFR = y - r - x;
        double veloBL = y + r - x;
        double veloBR = y - r + x;

        // Normalize the powers before we pass them into the motors (so that no power is outside of the range when passed in, preserving the intended ratio of the powers to each other)
        double largest_power = Math.max( Math.max( Math.abs(veloFL), Math.abs(veloFR)), Math.max(Math.abs(veloBL), Math.abs(veloBR)) ); // first find the largest of all the powers (get the max of the first two, max of the second two, then get the max of the two maxes)
        if(largest_power > 1.0){ // if the largest power value is greater than 1
            veloFL /= largest_power; // divide each power by the largest one
            veloFR /= largest_power; // resulting in the largest power being 1 (x/x = 1)
            veloBL /= largest_power; // and the rest scaled appropriately
            veloBR /= largest_power;
        }

        // set the motor powers based off of the math done previously - Make that robot go VROOOOM
        driveFL.setPower(veloFL);
        driveFR.setPower(veloFR);
        driveBL.setPower(veloBL);
        driveBR.setPower(veloBR);
    }


    /**
     * Outputs a turn command value, used for reaching a specified heading
     * @param targetHeading the RADIAN target heading for the robot
     * @param currentHeading the RADIAN current heading for the robot
     * @return the correct turn power to reach the target heading, according to the PID
     */
    public double calcTurnPIDPower(double targetHeading, double currentHeading){
        double error = targetHeading - currentHeading; // the error is the difference between where we want to be and where we are right now
        double timeDifference = localRuntime.milliseconds() - lastRuntime; // timeDifference is the time since the last runtime

        integral += error * timeDifference; // the integral is the sum of all error over time, and is used to push past unexpected resistance (as if the arm stays in a single position away from the set position for too long, it builds up over time and pushes past the resistance)
        // multiplied by the timeDifference to prevent wild variation in how much it is increase if cycle time increases/decreases for some reason
        double dError = ((error - lastError) / timeDifference); // the rate of change of the current error, this component creates a smooth approach to the set point

        double rotationPower = (turnKp * error) + (turnKi * integral) + (turnKd * dError); // multiply each term by its coefficient, then add together to get the final power

        lastError = error; // update the last error to be the current error
        lastRuntime = localRuntime.milliseconds(); // update the last runtime to be the current runtime

        return -rotationPower;
    }
}

