package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.Math;

@TeleOp(name = "DriveTeleop", group = "Concept")

public class DriveTeleop extends LinearOpMode  {
    // Initialize robot from another class
    HardwarePushbot robot = new HardwarePushbot();

    public void runOpMode(){
        robot.init(hardwareMap);
        int intakeState = 0; // 0 = off, 1 = on
        int vertSlideState = 0; // 0 = bottom, 1-2-3 set lines
        int depositState = 0; // 0 = ready to deposit, 1 = deposited
        int endGameState = 0; // 0 = ready, 1 = plane launched, 2 = measurement tape up, 3 = we are hanged on the truss!

        double robotSpeed = 1.0;
        double[] intakeArray = {0.0, 1.0}; // speeds of intake
        double[] intakeSpots = {1.0, 0.52}; // positions of intake servo
        double slidesCPR = 384.5;
        /*
        Notes on PID
        position = revolutions * cpr
        revolutions = desired distance / circumference

        position = desired distance * cpr / circumference
        */
        double circumferenceSlides = 3.9; // 3.9 cm, 39mm
        double[] distanceArray = {0.0, 25.0, 50.0, 75.0}; // in cm
        double[] vertSlideArray = {0.0, 0.0, 0.0, 0.0}; // is set in the for loop below
        for(int i = 0; i < 4; i++){
            vertSlideArray[i] = distanceArray[i] * slidesCPR / circumferenceSlides;
        }
        double[] depositLeftArray = {0.0, 1.0}; // TBD, will find exact later
        double[] depositRightArray = {0.0, 1.0}; // TBD, will find exact later
        double[] planeArray = {0.0, 1.0}; // TBD, will find exact later

        // Wait for the game to start (driver presses PLAY)

        // PID Initialize for ONE Slide

        // Reset the motor encoder so that it reads zero ticks
//        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Sets the starting position of the slide to the down position
//        robot.slide.setTargetPosition((int) vertSlideArray[0]);
//        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Sets the starting position of the hang to the down position


        double lastYClick = 0.0;
        double lastLeftBump = 0.0;
        double lastRightBump = 0.0;


        waitForStart();

        while (opModeIsActive()) {
            // Drive Train (REUSED CODE)
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;


            robot.motorFrontLeft.setPower(-v3*robotSpeed); // some of these might need to be negative
            robot.motorFrontRight.setPower(v4*robotSpeed);
            robot.motorBackLeft.setPower(v1*robotSpeed);
            robot.motorBackRight.setPower(-v2*robotSpeed);
            telemetry.addData("SHOWS UPDATE","blah");
            telemetry.update();
            if(gamepad1.left_trigger>0.2){
                robot.SlideLeft.setPower(-1.0);
                telemetry.addData("SHOWS UPDATE","SLIDEPRESSED");
                telemetry.update();
            }
            else{
                robot.SlideLeft.setPower(0.0);
            }

            if(gamepad1.b){
                robot.intake.setPower(-1.0);
                robot.intakeControl.setPosition(0.5); // 0.0 or 1.0
                robot.WheelLeft.setPower(1.0);
                robot.WheelRight.setPower(-1.0);

            }

            else{
                robot.intake.setPower(0.0);
                robot.intakeControl.setPosition(0.0); // 0.0 or 1.0
                robot.WheelLeft.setPower(0.0);
                robot.WheelRight.setPower(0.0);
            }

            if(gamepad1.dpad_up){

            }
            else if(gamepad1.dpad_down){

            }

            if ((gamepad1.left_bumper && gamepad2.right_bumper) || (gamepad2.left_bumper && gamepad2.right_bumper)){
                // emergency break
                break;
            }

        }
    }

    public double slidePosition(double linkage1, double linkage2, double distance){
        // linkage 1 = opposite of theta, linkage 2 = adjacent, distance = desired distance
        // input in cm, output in degrees (north of x axis)
        return Math.toDegrees(Math.acos(((distance*distance) + (linkage2*linkage2) - (linkage1*linkage1))/(2*distance*linkage2)));
    }
    public void FrontDrive(double power){
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower( power);
    }

    public void slides(double Position){

    }

    public void outtake(double Position){
        robot.depositLeft.setPosition(Position);
        robot.depositRight.setPosition(Position);

    }


}