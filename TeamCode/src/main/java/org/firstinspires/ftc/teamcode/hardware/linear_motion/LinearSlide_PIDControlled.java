package org.firstinspires.ftc.teamcode.hardware.linear_motion;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.VeloLimiter;


public class LinearSlide_PIDControlled {
    private DcMotor slideMotor;
    private PIDController PID;
    private VeloLimiter limiter;


    public static double DEFAULT_INCH_TARGET_MARGIN = 0.1;

    private double inchesPerMotorTic;
    private boolean hasLimiter = false;

    private double targetPos = 0;
    private double lastCurrentPos = 0;


    public LinearSlide_PIDControlled(DcMotor slideMotor, double inchesPerMotorTic, PIDController PID){
        this.slideMotor = slideMotor;
        this.inchesPerMotorTic = inchesPerMotorTic;
        this.PID = PID;
    }
    public LinearSlide_PIDControlled(DcMotor slideMotor, double inchesPerMotorTic, PIDController PID, VeloLimiter limiter){
        this.slideMotor = slideMotor;
        this.inchesPerMotorTic = inchesPerMotorTic;
        this.PID = PID;
        this.limiter = limiter;
        hasLimiter = true;

        limiter.setLimitDeceleration( false );
    }

    public void runToInternalTarget(){
        runToGivenPosition(targetPos);
    }
    public void runToGivenPosition(double targetPosition){
        double currentPos = getInchPosition();

        double outputPower = PID.getOutput(currentPos, targetPosition);

        if( hasLimiter ){
            outputPower = limiter.getLimitedVelo( outputPower );
        }

        slideMotor.setPower(outputPower);
    }
    public void setTargetPosition(double targetPosition){
        this.targetPos = targetPosition;
    }

    public double getInchPosition(){
        return slideMotor.getCurrentPosition() * inchesPerMotorTic;
    }
    public double getErrorToTarget(){
        return getInchPosition() - targetPos;
    }

    public boolean isAtTarget(){
        return Math.abs(getInchPosition() - targetPos) <= DEFAULT_INCH_TARGET_MARGIN;
    }

}
