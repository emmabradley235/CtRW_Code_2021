package org.firstinspires.ftc.teamcode.command.tests.mecanumBot2021;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.continuous_servos.ServoSpinner;
import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */


@TeleOp(name = "Rocker-Bogus Bot", group = "@@@")


public class TeleOpMecanumBot extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Rocker-Bogus Bot";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for transslation (1 being 100% of power going in)


    // Robot Classes
    private MecanumBot robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new MecanumBot(hardwareMap);
        runtime = new ElapsedTime();


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate




        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables
            //double xTranslatePower = AdvMath.squareAndKeepSign( gamepad1.left_stick_x ); // squaring the inputs to make sure small gamepad inputs are made smaller, while gamepad inputs of 1 stay the same (making it so that when you want to go full speed you can, but you also have more precise control at slower speeds than full)
            //double yTranslatePower = AdvMath.squareAndKeepSign( -gamepad1.left_stick_y );
            //double rotatePower = AdvMath.squareAndKeepSign( gamepad1.right_stick_x );
            double xTranslatePower = gamepad1.left_stick_x;
            double yTranslatePower = -gamepad1.left_stick_y; // the y value is made negative, as up on the gamepad stick is negative and we want it to be positive
            double rotatePower = gamepad1.right_stick_x;



            // Hardware instruction (telling the hardware what to do)
            if(gamepad1.dpad_left){ // if dpad left, run the servo that direction
                robot.duckSpinner.runForwards();
                // robot.duckSpinServo.setPosition(1.0);
            }
            else if (gamepad1.dpad_right){
                robot.duckSpinner.runBackwards(); // if dpad right, run the servo the other direction
                // robot.duckSpinServo.setPosition(0.0);
            }
            else{ // if none pressed, stop
                robot.duckSpinner.stop();
                //robot.duckSpinServo.setPosition(0.5);
            }


            robot.drive.driveRobotRelative(xTranslatePower, yTranslatePower, rotatePower);
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
