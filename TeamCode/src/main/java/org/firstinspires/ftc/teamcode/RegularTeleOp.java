package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;

@Config
@TeleOp(name = "Regular Teleop", group = "Concept")
public class RegularTeleOp extends LinearOpMode  {

    // Dynamic constants
    public static volatile double DRIVESPEED_FAST = 1.0; // between 0 and 1
    public static volatile double DRIVESPEED_SLOW = 0.3; // between 0 and 1
    public static volatile double SLIDESPEED = 16.0; // must be whole num
    public static volatile TwoPositions intakePositions = new TwoPositions(1.0, 0.5);
    public static volatile TwoPositions depositLeftPositions = new TwoPositions(0.0, 0.575);
    public static volatile TwoPositions depositRightPositions = new TwoPositions(1.0, 0.425);

    // Other classwide items
    private HardwarePushbot robot = new HardwarePushbot();
    private FtcDashboard dashboard;

    public static class TwoPositions {
        public double normal;
        public double out;

        public TwoPositions(double normal, double out) {
            this.normal = normal;
            this.out = out;
        }
    }

    public void runOpMode() {
        // do some initialization
        robot.init(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        Telemetry _tele = telemetry;
        MultipleTelemetry telemetry = new MultipleTelemetry(_tele, dashboard.getTelemetry());


        int endGameState = 0; // 0 = ready, 1 = plane launched, 2 = measurement tape up, 3 = we are hanged on the truss!
//        int depositBoxState = 0; // 0 = we didn't rotate it, 1 = we rotated it out
        boolean planeReleased = false; // false if we didn't release the plane
        boolean driveSlow = false;
        boolean holdingY = false;
        boolean intaking;

        Slides slides = new Slides(
                telemetry,
                robot.slideLeft,
                robot.slideRight,
                robot.slideEncoder
            );

        waitForStart();

        while (opModeIsActive()) {
            double loopIterationStartTime = System.currentTimeMillis();

//            if (gamepad2.y && !holdingY) {
//                driveSlow = !driveSlow;
//                holdingY = true;
//            } else {
//                holdingY = false;
//            }

            // Drive Train (REUSED CODE), except we cube the motor power to reduce it
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            double robotSpeed;
//            if (driveSlow) { robotSpeed = SPEED_SLOW; }
//            else           { robotSpeed = SPEED_FAST; }
            if (gamepad2.y) { robotSpeed = DRIVESPEED_SLOW; }
            else            { robotSpeed = DRIVESPEED_FAST; }

            robot.motorFrontLeft.setPower(-v3*robotSpeed); // some of these might need to be negative
            robot.motorFrontRight.setPower(v4*robotSpeed);
            robot.motorBackLeft.setPower(v1*robotSpeed);
            robot.motorBackRight.setPower(-v2*robotSpeed);


            // Third Segment Intake Power
            if (gamepad2.x) {
                // Intake Normal
                wheelKeepPixel();
                intaking = true;
                robot.intake.setPower(-1.0);
                robot.intakeControl.setPosition(intakePositions.out);
            }
            else if (gamepad2.b) {
                // Reverse Intake spit out
                // this is just in case we accidentally take 3 pixels
                intaking = false;
                robot.intake.setPower(1.0);
                robot.intakeControl.setPosition(intakePositions.out);
            }
            else {
                // no power intake
                intaking = false;
                robot.intake.setPower(0.0);
              robot.intakeControl.setPosition(intakePositions.normal);
            }

            // First Segment Servo Deposit
            if (gamepad1.left_trigger > 0.2) {
                //Keep Pixel
                wheelKeepPixel();
            }
            else if (gamepad1.right_trigger > 0.2) {
                // spit pixel
                wheelSpitPixel();
            }
            else if (!intaking) {
                // do nothing
                robot.wheel.setPower(0.0);
            }

            // 4th segment intake control (THIS IS JUST A BACKUP IN CASE SOMETHING GOES WRONG, TODO: DELETE THIS)
            if (gamepad2.dpad_right) {
                // Folded up
//                robot.intakeControl.setPosition(intakecoll);
                setDepositBox();
            }
            else if (gamepad2.dpad_left) {
                // Taking in
//                robot.intakeControl.setPosition(intakeup);
                resetDepositBox();
            }

//            //5th segment Slides
//            if (gamepad2.dpad_up) {
//                // Slide up
//                slides.moveUp(SLIDESPEED_UP);
//            }
//            else if (gamepad2.dpad_down) {
//                slides.moveDown(SLIDESPEED_DOWN);
//            }
//            else {
//                // no power
//                robot.slideLeft.setPower(0.0);
//                robot.slideRight.setPower(0.0);
//            }

            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                // we do negative since our gamepad stick is sus
                slides.move(-gamepad2.left_stick_y * SLIDESPEED);
            }
            if (gamepad2.a) { slides.moveToBottom(); }


//            if (!planeReleased) {
//                if(gamepad1.b) {
//                    // rlease the plane
//                    planeReleased = true;
//                    robot.plane.setPosition(1.0);
//                }
//            }

            // Fix the slide positions
            double slidePower = slides.updateSlides();
            slides.setSlidePower(slidePower);

            // emergency break
            if ((gamepad1.left_bumper && gamepad1.right_bumper)) { break; }

            // update all of the telemetry at the end of each loop iteration
            telemetry.addData("Loop time", System.currentTimeMillis() - loopIterationStartTime);
            telemetry.update();
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

    public void wheelKeepPixel() {
        robot.wheel.setPower(-1.0);
    }

    public void wheelSpitPixel() {
        robot.wheel.setPower(1.0);
    }

}
