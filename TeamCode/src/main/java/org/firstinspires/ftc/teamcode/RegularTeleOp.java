package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.Math;

@Config
@TeleOp(name = "Regular Teleop", group = "Concept")
public class RegularTeleOp extends LinearOpMode  {
    // Initialize robot from another class
    HardwarePushbot robot = new HardwarePushbot();
    public static volatile double SPEED_FAST = 1.0;
    public static volatile double SPEED_SLOW = 0.5;
    public static volatile double INTAKEUP = 1.0;
    public static volatile double INTAKECOLL = 0.6;
    public static final double slidesCPR = 384.5;
    public static DepositPositions depositLeftPositions = new DepositPositions(0.0, 0.75);
    public static DepositPositions depositRightPositions = new DepositPositions(1.0, 0.25);

    public static class DepositPositions {
        public double normal;
        public double out;

        public DepositPositions(double n, double o) {
            normal = n;
            out = o;
        }
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        int wheelState = 0; // 0 = not intaking, 1 = yes intaking
        int endGameState = 0; // 0 = ready, 1 = plane launched, 2 = measurement tape up, 3 = we are hanged on the truss!
        int depositBoxState = 0; // 0 = we didn't rotate it, 1 = we rotated it out
        boolean planeReleased = false; // false if we didn't release the plane

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

            double robotSpeed;
            if (gamepad1.x) { robotSpeed = SPEED_SLOW; }
            else            { robotSpeed = SPEED_FAST; }

            robot.motorFrontLeft.setPower(-v3*robotSpeed); // some of these might need to be negative
            robot.motorFrontRight.setPower(v4*robotSpeed);
            robot.motorBackLeft.setPower(v1*robotSpeed);
            robot.motorBackRight.setPower(-v2*robotSpeed);

            // First Segment Servo Deposit
            if (gamepad1.left_trigger > 0.2) {
                //Keep Pixel
                robot.wheel.setPower(1.0);
            }
            else if (gamepad1.right_trigger > 0.2) {
                // spit pixel
                robot.wheel.setPower(-1.0);
            }
            else if (wheelState == 1) {
                // keep pixel
                robot.wheel.setPower(1.0);
            }
            else {
                // do nothing
                robot.wheel.setPower(0.0);
            }

            // Third Segment Intake Power
            if (gamepad2.x) {
                // Reverse Intake spit out
                // this is just in case we accidentally take 3 pixels
                robot.intake.setPower(-1.0);
                robot.intakeControl.setPosition(INTAKECOLL);
            }
            else if (gamepad2.b) {
                // Intake Normal
                wheelState = 1;
                robot.intake.setPower(1.0);
                robot.intakeControl.setPosition(INTAKECOLL);
            }
            else {
                // no power intake
                wheelState = 0;
                robot.intake.setPower(0.0);
                robot.intakeControl.setPosition(INTAKEUP);
            }


            // 4th segment intake control (THIS IS JUST A BACKUP IN CASE SOMETHING GOES WRONG, TODO: DELETE THIS)
            if (gamepad2.dpad_up) {
                // Folded up
//                robot.intakeControl.setPosition(intakecoll);
                depositBoxState = 1;
                setDepositBox();
            }
            else if (gamepad2.dpad_down) {
                // Taking in
//                robot.intakeControl.setPosition(intakeup);
                depositBoxState = 0;
            }

            //5th segment Slides
            if (gamepad2.left_bumper) {
                // Slide down
                robot.SlideLeft.setPower(0.4);  // go down a little slowly
                robot.SlideRight.setPower(0.6); // go down a little slowly
            }
            else if (gamepad2.right_bumper) {
                robot.SlideLeft.setPower(-1.0);
                robot.SlideRight.setPower(-1.0);
            }
            else {
                // no power
                robot.SlideLeft.setPower(0.0);
                robot.SlideRight.setPower(0.0);
            }

            if (depositBoxState == 0) {
                resetDepositBox();
            }


//            if (!planeReleased) {
//                if(gamepad1.b) {
//                    // rlease the plane
//                    planeReleased = true;
//                    robot.plane.setPosition(1.0);
//                }
//            }

            if ((gamepad1.left_bumper && gamepad1.right_bumper)) {
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

    public void resetDepositBox() {
        /**
         * Reset the position of the deposit box thing to default
         */
        robot.depositLeft.setPosition(depositLeftPositions.normal);
        robot.depositRight.setPosition(depositRightPositions.normal);
    }

    public void setDepositBox() {
        /**
         * Set the position of the deposit box thing to be moved out and ready for outtaking.
         */
        robot.depositLeft.setPosition(depositLeftPositions.out);
        robot.depositRight.setPosition(depositRightPositions.out);
    }

}
