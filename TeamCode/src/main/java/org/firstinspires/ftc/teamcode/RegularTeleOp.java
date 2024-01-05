package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.Math;

@TeleOp(name = "Regular Teleop", group = "Concept")

public class RegularTeleOp extends LinearOpMode  {
    // Initialize robot from another class
    HardwarePushbot robot = new HardwarePushbot();

    public void runOpMode(){
        robot.init(hardwareMap);
        double intakeup=1.0;
        double intakecoll=0.6;
        int intakeState = 0; // 0 = off, 1 = on
        int vertSlideState = 0; // 0 = bottom, 1-2-3 set lines
        int depositState = 0; // 0 = ready to deposit, 1 = deposited
        int wheelState = 0; // 0 = not intaking, 1 = yes intaking, 2 = reverse intaking
        int endGameState = 0; // 0 = ready, 1 = plane launched, 2 = measurement tape up, 3 = we are hanged on the truss!
        boolean slideIsUp = false; // false = down, true = up
        double slideTimerInitial = 0.0;
        int outtakePos = 0; // 0 = we didn't rotate it, 1 = we rotated it out
        boolean planeReleased = false; // false if we didn't release the plane

        double robotSpeedFast = 1.0;
        double robotSpeedSlow = 0.5;
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
            // Drive Train (REUSED CODE), except we cube the motor power to reduce it
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            if (gamepad1.x) {
                frontLeftPower  = -v3*robotSpeedFast;
                frontRightPower =  v4*robotSpeedFast;
                backLeftPower   =  v1*robotSpeedFast;
                backRightPower  = -v2*robotSpeedFast;
            }
            else {
                frontLeftPower  = -v3*robotSpeedSlow;
                frontRightPower =  v4*robotSpeedSlow;
                backLeftPower   =  v1*robotSpeedSlow;
                backRightPower  = -v2*robotSpeedSlow;
            }

            robot.motorFrontLeft.setPower(frontLeftPower); // some of these might need to be negative
            robot.motorFrontRight.setPower(frontRightPower);
            robot.motorBackLeft.setPower(backLeftPower);
            robot.motorBackRight.setPower(backRightPower);

            // First Segment Servo Deposit
            if (gamepad1.left_trigger > 0.2) {
                //Spit Pixel
                robot.wheel.setPower(1.0);
            }
            else if (gamepad1.right_trigger > 0.2) {
                // hold pixel
                robot.wheel.setPower(-1.0);
            }
            else if (wheelState == 1) {
                robot.wheel.setPower(1.0);
            }
            else if (wheelState == 2) {
                robot.wheel.setPower(-1.0);
            }
            else {
                // do nothing
                robot.wheel.setPower(0.0);
            }

            // Third Segment Intake Power
            if (gamepad2.x) {
                // Reverse Intake spit out
                wheelState = 2; // TODO: check later
                robot.intake.setPower(-1.0);
                robot.intakeControl.setPosition(intakecoll);
            }
            else if (gamepad2.b) {
                // Intake Normal
                wheelState = 1;
                robot.intake.setPower(1.0);
                robot.intakeControl.setPosition(intakecoll);
            }
            else {
                // no power intake
                wheelState = 0;
                robot.intake.setPower(0.0);
                robot.intakeControl.setPosition(intakeup);
            }


            // 4th segment intake control (THIS IS JUST A BACKUP IN CASE SOMETHING GOES WRONG, TODO: DELETE THIS)
            if (gamepad2.dpad_up) {
                // Folded up
//                robot.intakeControl.setPosition(intakecoll);
                robot.depositLeft.setPosition(0.68);  // TODO: find out actual numbers here
                robot.depositRight.setPosition(0.32); // TODO: find out actual numbers here
            }
            else if (gamepad2.dpad_down) {
                // Taking in
//                robot.intakeControl.setPosition(intakeup);
                robot.depositLeft.setPosition(0.0);  // TODO: find out actual numbers here
                robot.depositRight.setPosition(1.0); // TODO: find out actual numbers here
            }

            //5th segment Slides
            if (gamepad2.left_bumper) {
                if (slideIsUp) {
                    // if the slide was going up and is now going down, then we reset the timer
                    slideTimerInitial = System.currentTimeMillis();
                }
                else if (outtakePos == 1 && System.currentTimeMillis() - slideTimerInitial >= 315) {
                    outtakePos = 1;
                    // if we didn't rotate it in yet and more than 250 ms have passed, then rotate in the outtake thing
                    robot.depositLeft.setPosition(0.0);  // TODO: find out actual numbers here
                    robot.depositRight.setPosition(1.0); // TODO: find out actual numbers here
                }
                // Slide down
                slideIsUp = false;
                slideTimerInitial = System.currentTimeMillis();
                robot.SlideLeft.setPower(0.75);  // go down a little slowly
                robot.SlideRight.setPower(0.75); // go down a little slowly
            }
            else if (gamepad2.right_bumper) {
                // Slide up
                if (!slideIsUp) {
                    // if the slide was going down and is now going up, then we reset the timer
                    slideTimerInitial = System.currentTimeMillis();
                }
                else if (outtakePos != 1 && System.currentTimeMillis() - slideTimerInitial >= 315) {
                    outtakePos = 1;
                    // if we didn't rotate it out yet and more than 250 ms have passed, then rotate out the outtake thing
                    robot.depositLeft.setPosition(0.68);  // TODO: find out actual numbers here
                    robot.depositRight.setPosition(0.32); // TODO: find out actual numbers here
                }
                slideIsUp = true;
                robot.SlideLeft.setPower(-1.0);
                robot.SlideRight.setPower(-1.0);
            }
            else {
                // no power
                robot.SlideLeft.setPower(0.0);
                robot.SlideRight.setPower(0.0);
            }


            if (!planeReleased) {
                if(gamepad1.b) {
                    // rlease the plane
                    planeReleased = true;
                    robot.plane.setPosition(1.0);
                }
            }

            if ((gamepad1.left_bumper && gamepad1.right_bumper) || (gamepad2.left_bumper && gamepad2.right_bumper)) {
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
        robot.motorBackLeft.setPower((312.0) * power);
    }

    public void slides(double Position){

    }

    public void outtake(double Position){
        robot.depositLeft.setPosition(Position);
        robot.depositRight.setPosition(Position);

    }


}