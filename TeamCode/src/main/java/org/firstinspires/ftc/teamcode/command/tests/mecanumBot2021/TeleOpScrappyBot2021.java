package org.firstinspires.ftc.teamcode.command.tests.mecanumBot2021;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.servo_managers.Servo_Spinner;
import org.firstinspires.ftc.teamcode.utility.Math.LinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.ReplayRecorder.backend_classes.GamepadState;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Point2d;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */





@TeleOp(name = "Scrappy TeleOp", group = "@@@")
@Config

public class TeleOpScrappyBot2021 extends LinearOpMode{
    // TeleOp Variables
    public String controllerConfig = "Controller Configurations:\n" +
            "Gamepad 1 -\n" +
            " - Left stick = Robot translation\n" +
            " - Right stick left/right = Robot rotation\n" +
            " - Left trigger = Translation boost factor\n" +
            " - Right trigger = Rotation boost factor\n" +
            " - Right stick up/down = Intake power\n" +
            " - Right shoulder button = Toggle if the intake is running\n" +
            "\n" +
            "Gamepad 2 -\n" +
            " - Right bumper = Toggle duck spinner spinning automatically (goes right)\n" +
            " - Left bumper = Toggle duck spinner spinning automatically (goes left)\n" +
            " - Right trigger = Manual duck spin right\n" +
            " - Left trigger = Manual duck spin left\n" +
            " - DPad-Down = Bucket slide move to intake position\n" +
            " - DPad-Left = Bucket slide move to score lvl 1 position\n" +
            " - DPad-Right = Bucket slide move to score lvl 2 position\n" +
            " - DPad-Up = Bucket slide move to score lvl 3 position\n" +
            " - A = Dump bucket then move slide to intake position\n" +
            " - Right Stick up/down = Move out/in the duck spinner linear slide\n";

    // Robot Speed constants
    public static double LOWER_ROTATE_SPEED_MULT = 0.5; // Speed multipliers for turning (1 being 100% of power going in)
    public static double UPPER_ROTATE_SPEED_MULT = 1.0;

    public static double LOWER_TRANS_SPEED_MULT = 0.4; // Speed multipliers for transslation (1 being 100% of power going in)
    public static double UPPER_TRANS_SPEED_MULT = 1.0;

    public static double INTAKE_SPEED_MULT = 1.0; // power multiplier for the intake

    public static double DUCK_SLIDE_SPEED_MULT = 1.0;


    // Misc constants
    public static double TELEOP_DURATION_SECONDS = 90; // cut off controls after the duration is complete to prevent penalties
    public static double GAMEPAD_DEADZONE_RADIUS = 0.05;
    public static double TRIGGER_DEADZONE = 0.05;


    // Robot Classes
    private ScrappyBot2021 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock

    // Value manipulation classes
    private LinearInterpolator transMultiplierInterp;
    private LinearInterpolator rotMultiplierInterp;

    // flag variables
    private boolean canToggleIntake = true;
    private boolean canTogglePeriodicDuckSpin = true;
    private boolean canToggleDumping = true;

    // tracking variables
    private double duckSlideTarget = 0;


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new ScrappyBot2021(hardwareMap);
        runtime = new ElapsedTime();

        transMultiplierInterp = new LinearInterpolator(new Point2d(0, UPPER_TRANS_SPEED_MULT), new Point2d(1, LOWER_TRANS_SPEED_MULT));
        rotMultiplierInterp = new LinearInterpolator(new Point2d(0, UPPER_ROTATE_SPEED_MULT), new Point2d(1, LOWER_ROTATE_SPEED_MULT));


        telemetry.addData(robot.getName() + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate




        // The main run loop - write the main robot run code here
        while ( opModeIsActive() && runtime.seconds() <= TELEOP_DURATION_SECONDS ) {
            // take the gamepads and turn them into custom gamepad objects
            // (only here for future compatibility with ReplayRecorder, if not using ReplayRecorder just replace gp1 with gamepad1 in this code and remove this section)
            GamepadState gp1 = new GamepadState(gamepad1);
            GamepadState gp2 = new GamepadState(gamepad2);


            // the speed modifiers
            double transMult = transMultiplierInterp.interpolate(gp1.left_trigger);
            double rotMult = rotMultiplierInterp.interpolate(gp1.right_trigger);

            // the robot movement commands
            double xTranslatePower = gp1.left_stick_x * transMult;
            double yTranslatePower = -gp1.left_stick_y * transMult; // the y value is made negative, as up on the gamepad stick is negative and we want it to be positive
            double rotatePower = gp1.right_stick_x * rotMult;


            // intake movement control
            double intakePower = gp1.right_stick_y * INTAKE_SPEED_MULT;

            if( gp1.right_bumper && canToggleIntake ){ // if first frame of button being pressed
                robot.intake.setRunning( !robot.intake.isRunning() ); // toggle the running state
                canToggleIntake = false; // make it so it can't toggle again until the button is released
            }
            else if( !gp1.right_bumper )
                canToggleIntake = true;


            // duck spinner control
            if( gp2.right_bumper && canTogglePeriodicDuckSpin ){ // duck spinner periodic spinning toggles
                robot.togglePeriodicDuckSpin( Servo_Spinner.SpinDirection.FORWARD );
            }
            else if( gp2.left_bumper && canTogglePeriodicDuckSpin ){
                robot.togglePeriodicDuckSpin( Servo_Spinner.SpinDirection.BACKWARD );
            }
            else if( !gp2.left_bumper && !gp2.right_bumper ){
                canTogglePeriodicDuckSpin = true;
            }
            // manual duck overrides
            if( gp2.right_trigger >= TRIGGER_DEADZONE )
                robot.duckSpinner.runForwards();
            else if( gp2.left_trigger >= TRIGGER_DEADZONE )
                robot.duckSpinner.runBackwards();
            else if( robot.duckSpinner.isPeroidicSpinning() )
                robot.duckSpinner.updatePeriodicSpin();
            else
                robot.duckSpinner.stop();


            // dumping manual control
            if( gp2.a && canToggleDumping ){
                robot.toggleDumpRoutine();
                canToggleDumping = false;
            }
            else if( !gp2.a )
                canToggleDumping = true;

            robot.updateDumpRoutine();


            // manual bucket slide control
            if( gp2.dpad_down )
                robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketDownPos );
            else if( gp2.dpad_left )
                robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketLvl1Pos );
            else if( gp2.dpad_right )
                robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketLvl2Pos );
            else if( gp2.dpad_up )
                robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketLvl3Pos );


            // manual duck slide control
            duckSlideTarget += -gp2.right_stick_y * DUCK_SLIDE_SPEED_MULT;

            if( duckSlideTarget < ScrappyBot2021.duckSlideExtremePositions.min )
                duckSlideTarget = ScrappyBot2021.duckSlideExtremePositions.min;
            else if( duckSlideTarget > ScrappyBot2021.duckSlideExtremePositions.max )
                duckSlideTarget = ScrappyBot2021.duckSlideExtremePositions.max;


            // passing powers into systems
            robot.drive.driveRobotRelative(xTranslatePower, yTranslatePower, rotatePower);
            robot.intake.setIntakeRunSpeed( intakePower );
            robot.bucketSlide.runToInternalTarget();
            robot.duckSlide.runToInternalTarget();
            if( !robot.isDumpRoutineRunning() ){
                if( robot.bucketSlide.getInchPosition() <= ScrappyBot2021.armToIntakePosThreshold )
                    robot.bucketArm.goToStateNamed("Intake");
                else
                    robot.bucketArm.goToStateNamed("Prep");
            }

            telemetry.addLine(controllerConfig);
            telemetry.addData("Duck spinner direction", robot.duckSpinner.getServoSpinDirection());
            telemetry.update();
        }

        // once loop done, stop everything
        robot.stopRobot();

        telemetry.addLine("TeleOp duration complete -> robot stopped");
        telemetry.update();
    }


    /* PUT ALL FUNCTIONS HERE */

}
