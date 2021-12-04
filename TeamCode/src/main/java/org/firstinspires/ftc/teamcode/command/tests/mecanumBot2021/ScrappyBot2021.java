package org.firstinspires.ftc.teamcode.command.tests.mecanumBot2021;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.intakes.Intake;
import org.firstinspires.ftc.teamcode.hardware.linear_motion.LinearSlide_PIDControlled;
import org.firstinspires.ftc.teamcode.hardware.servo_managers.Servo_MultiSetState;
import org.firstinspires.ftc.teamcode.hardware.servo_managers.Servo_Spinner;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utility.Odometry.TrackingWheelLocalizer2021_Scrappy;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.NamedState;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Range2d;


/*
    Welcome to the template Provider class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named Provider2020 and all code within would have to reflect that change).

    Happy coding!
 */



// The main robot data class - called provider because it provides hardware classes with the robot data that they need to function

@Config

public class ScrappyBot2021 {
    public static String robotName = "Scrappy";

    public static double bucketIntakeRot = 0.0;
    public static double bucketPrepRot = 0.7;
    public static double bucketDumpRot = 1.0;
    public static double dumpMilliseconds = 2000;

    public static double bucketDownPos = 0.0;
    public static double bucketLvl1Pos = 10.0;
    public static double bucketLvl2Pos = 15.0;
    public static double bucketLvl3Pos = 20.0;
    public static double armToIntakePosThreshold = 8.0;


    public static double motorTicsToSlideInches = 360;

    public static PIDCoefficients duckSlideCoeff = new PIDCoefficients(1.0, 0, 0);
    public static PIDCoefficients bucketSlideCoeff = new PIDCoefficients(1.0, 0, 0);

    public static Range2d bucketIActiveRange = new Range2d(-0.5, 0.5);
    public static Range2d bucketIWindupBounds = new Range2d(-1, 1);

    public static int duckSpinDuration = 2500;
    public static int duckSpinStopDuration = 700;

    public static Range2d duckSlideExtremePositions = new Range2d(0, 30);

    // Motor and servo variables

    // Drive motors
    private DcMotor driveFL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBL = null;
    private DcMotor driveBR = null;

    // Manipulator motors (motors for other things, not driving) - names are example names, they can be set for whatever application you have
    private DcMotor motorLift = null;
    private DcMotor motorIntake = null;
    private DcMotor motorDuck = null;


    // Servo Variables - names are example names, they can be set for whatever application you have
    private Servo duckSpinServo = null;
    private Servo bucketArmServo = null;



    // Sensor Variables
    // Distance Sensor Variables - names are just examples, feel free to change to whatever you like. Just note that "flight" represents the fact that they are Time of Flight sensors (think radar, but with lasers)
    //public Rev2mDistanceSensor flightFront0;
    //public Rev2mDistanceSensor flightLeft1;
    //public Rev2mDistanceSensor flightRight2;
    //public Rev2mDistanceSensor flightBack3;

    public BNO055IMU imu; // the IMU class instance

    // The all important hardware map (basically a log of what devices are plugged into what ports. Setup on the FTC Robot Controller app)
    private HardwareMap mainMap;

    // All hardware/process class instances
    public Drive_Mecanum drive;
    public LinearSlide_PIDControlled duckSlide;
    public LinearSlide_PIDControlled bucketSlide;
    public Servo_Spinner duckSpinner;
    public Servo_MultiSetState bucketArm;
    public Intake intake;
    public Localizer odometry;


    // Default constructor
    public ScrappyBot2021(HardwareMap hMap){
        initMap(hMap); // pull information from the hardware map - MUST BE DONE BEFORE

        initIMU(); // setup the IMU and calibrate the current position to be 0

        // init all hardware/processing object here
        drive = new Drive_Mecanum(driveFL, driveFR, driveBL, driveBR);
        intake = new Intake(motorIntake);
        duckSpinner = new Servo_Spinner(duckSpinServo, 0.5, 1.0, 0.0, duckSpinStopDuration, duckSpinDuration);
        bucketArm = new Servo_MultiSetState(bucketArmServo, new NamedState("Intake", bucketIntakeRot), new NamedState("Prep", bucketPrepRot), new NamedState("Dump", bucketDumpRot));
        duckSlide = new LinearSlide_PIDControlled(motorDuck, motorTicsToSlideInches, new PIDController(duckSlideCoeff));
        bucketSlide = new LinearSlide_PIDControlled(motorDuck, motorTicsToSlideInches, new PIDController(bucketSlideCoeff, bucketIActiveRange, bucketIWindupBounds));
        odometry = new TrackingWheelLocalizer2021_Scrappy(hMap);
    }
    public ScrappyBot2021(HardwareMap hMap, String robotName){
        this(hMap);

        this.robotName = robotName;
    }


    // Initialization functions

    private void initMap(HardwareMap hMap){    // setup the hardware map dependant classes (usually by grabbing their components out of the hardware map)
        mainMap = hMap;

        /* NOTE - hardware map requests for hardware that is not in the hardware map (because they are not being used this season for example),
                  should either be removed or commented out to prevent the computer looking for hardware that isn't there.

                  That said, it is ok to have hardware map requests for hardware that isn't physically plugged in to the hubs at the moment,
                  as long as it is present in the hardware map (again, configured in the FTC Robot Controller app).
                  Just be aware that any attempt to get readings from sensors or motors that aren't plugged in will not be reliable,
                  and motors that aren't plugged in will not move (Don't worry, we've all done that at least once ;)
         */


        // Grabbing motors from hardware map
        driveFL = mainMap.get(DcMotor.class, "driveFL");
        driveFR = mainMap.get(DcMotor.class, "driveFR");
        driveBL = mainMap.get(DcMotor.class, "driveBL");
        driveBR = mainMap.get(DcMotor.class, "driveBR");
        motorLift = mainMap.get(DcMotor.class, "motorLift");
        motorIntake = mainMap.get(DcMotor.class, "motorIntake");
        motorDuck = mainMap.get(DcMotor.class, "motorDuck");

        // Set motors to run with encoders (uncomment if you are, comment out if you are not)
        //driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       /* motorIntakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        // Reverse motor direction as needed
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);


        // Grabbing servos from hardware map (uncomment if you are using them, comment out if you are not)
        duckSpinServo = mainMap.get(Servo.class, "duckSpinServo");
        bucketArmServo = mainMap.get(Servo.class, "bucketArmServo");

        /*intakeDropL = mainMap.get(Servo.class, "intakeDropL");
        intakeDropR = mainMap.get(Servo.class, "intakeDropR");
        pullerDropL = mainMap.get(Servo.class, "pullerDropL");
        pullerDropR = mainMap.get(Servo.class, "pullerDropR");
        */

        // Grabbing sensors from hardware map (uncomment if you are using them, comment out if you are not)
        /*
        touchSensor0 = mainMap.get(DigitalChannel.class, "touchLift0");
        touchSensor1 = mainMap.get(DigitalChannel.class, "touchArm1");
        touchSensor2 = mainMap.get(DigitalChannel.class, "touchBlock2");
        touchSensor3 = mainMap.get(DigitalChannel.class, "touchLiftUp3");
        touchSensor4 = mainMap.get(DigitalChannel.class, "touchLeft4");
        touchSensor5 = mainMap.get(DigitalChannel.class, "touchRight5");
        touchSensor6 = mainMap.get(DigitalChannel.class, "touchBack6");
        touchSensor7 = mainMap.get(DigitalChannel.class, "touchClamp7");
        */

        // Time of flight sensor setup (uncomment if you are using them, comment out if you are not)
        /*
        flightFront0 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightFront0");
        flightLeft1  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightLeft1");
        flightRight2 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightRight2");
        flightBack3  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightBack3");
        */

        imu = mainMap.get(BNO055IMU.class, "imu");   // get Inertial Measurement Unit from the hardware map
    }

    private void initIMU(){ // create a new IMU class instance and set our IMU to that
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "DroopyIMU.json"; // see the calibration sample opmode
        parameters.loggingEnabled      =  true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }


    // External utility functions (ones that will be used outside this class)

    public void resetIMU(){ // an alternative name for init_imu, which is in this case public. The purpose of this is to more clearly convey application (you don't need to init the imu as the user, but you may want to reset it)
        initIMU();
    }

    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // get the current heading of the robot in degrees
    }
    public double getHeading(AngleUnit angleUnit){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle; // get the current heading of the robot in degrees
    }
    public String getName(){
        return robotName;
    }



    // hardware subroutine methods
    void stopRobot(){
        drive.driveRobotRelative(0, 0, 0);
        intake.spinDown();
        duckSpinner.stop();
        duckSlide.runToGivenPosition( duckSlide.getInchPosition() ); // tell PIDs to run to their current position, hopefully stoppign them
        bucketSlide.runToGivenPosition( bucketSlide.getInchPosition() );
        bucketArmServo.setPosition( bucketArmServo.getPosition() ); // same idea for the servo
    }

    void togglePeriodicDuckSpin(Servo_Spinner.SpinDirection direction){
        if( !duckSpinner.isPeroidicSpinning() || duckSpinner.getPeriodicSpinDirection() == direction)
            duckSpinner.startPeriodicSpin( direction );
        else
            duckSpinner.stopPeriodicSpin();
    }

    private boolean dumpRoutineRunning = false;
    private int dumpRoutineState = 0;
    private ElapsedTime dumpRoutineStateTimer;
    public boolean isDumpRoutineRunning(){return dumpRoutineRunning;}
    public void startDumpRoutine(){
        dumpRoutineRunning = true;
        dumpRoutineState = 0;
        dumpRoutineStateTimer = new ElapsedTime();
    }
    public void stopDumpRoutine(){
        dumpRoutineRunning = false;
    }
    public void toggleDumpRoutine(){
        if( !dumpRoutineRunning )
            startDumpRoutine();
        else
            stopDumpRoutine();
    }
    public void updateDumpRoutine(){
        if( dumpRoutineRunning ){
            switch (dumpRoutineState){
                case 0:
                    if( bucketSlide.isAtTarget() ){
                        dumpRoutineState = 1;
                        dumpRoutineStateTimer.reset();
                    }
                    break;

                case 1:
                    bucketArm.goToStateNamed("Dump");
                    if( dumpRoutineStateTimer.milliseconds() >= dumpMilliseconds){
                        dumpRoutineState = 2;
                    }

                case 2:
                    bucketArm.goToStateNamed("Intake");
                    bucketSlide.setTargetPosition(bucketDownPos);
            }
        }
    }
}

