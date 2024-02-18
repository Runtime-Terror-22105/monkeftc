package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.Math;

@Config
@TeleOp(name = "TeleopFaster", group = "Concept")
public class SusTeleOp extends LinearOpMode  {

    // Dynamic constants
    public static volatile double DRIVESPEED_FAST = 1.0; // between 0 and 1
    public static volatile double DRIVESPEED_SLOW = 0.3; // between 0 and 1
    public static volatile double SLIDESPEED = 21; // must be whole num
    public static volatile double MAX_TELEOP_VEL = 92.61691602936227;
    public static volatile int DEPOSIT_OUT_HEIGHT = 1000;
    public static volatile TwoPositions intakePositions = new TwoPositions(1.0, 0.5);
    public static volatile TwoPositions depositLeftPositions = new TwoPositions(0.67778, 0.0);
//    public static volatile TwoPositions depositRightPositions = new TwoPositions(1.0, 0.0);

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

        // sus driving init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, MAX_TELEOP_VEL);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        int endGameState = 0; // 0 = ready, 1 = plane launched, 2 = measurement tape up, 3 = we are hanged on the truss!
//        int depositBoxState = 0; // 0 = we didn't rotate it, 1 = we rotated it out
        boolean planeReleased = false; // false if we didn't release the plane
        boolean holdingY = false;
        boolean shouldResetDepositBox = false;
        ElapsedTime depositBoxTimer = new ElapsedTime();
        boolean ignore_automatic_depositbox = false;
        boolean intaking;
        double lastLoopTime = 0;

        ElapsedTime loopTimer = new ElapsedTime();

        Slides slides = new Slides(
                telemetry,
                robot.slideLeft,
                robot.slideRight,
                robot.slidesEncoder
        );

        waitForStart();
        resetDepositBox();

        while (opModeIsActive()) {
            // reset the timer
            loopTimer.reset();

            // Slides
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                // we do negative since our gamepad stick is sus
//                slides.move(-gamepad2.left_stick_y * SLIDESPEED * lastLoopTime);
                slides.move(-gamepad2.left_stick_y * SLIDESPEED);
                telemetry.addData("status", "raising slides");
                telemetry.addData("move amount", -gamepad2.left_stick_y * SLIDESPEED);
            }

            if (gamepad2.dpad_left) {
                slides.moveToLineOne();
            } else if (gamepad2.dpad_up) {
                slides.moveToLineTwo();
            } else if (gamepad2.dpad_right) {
                slides.moveToLineThree();
            } else if (gamepad2.dpad_down) {
                shouldResetDepositBox = true;
                depositBoxTimer.reset();
                resetDepositBox();
            }

            if (shouldResetDepositBox && depositBoxTimer.milliseconds() >= 750) {
                slides.moveToBottom();
            }

            double slidePosition = robot.slidesEncoder.getCurrentPosition();
            if (slidePosition < 15) {
                shouldResetDepositBox = false;
            } else if (!shouldResetDepositBox && !ignore_automatic_depositbox && slidePosition >= DEPOSIT_OUT_HEIGHT) {
                setDepositBox();
            } else if (!ignore_automatic_depositbox && slidePosition <= DEPOSIT_OUT_HEIGHT) {
                resetDepositBox();
            }

            // Setting the Outtake Box
            if (gamepad2.left_trigger > 0.2) {
                ignore_automatic_depositbox = true;
                resetDepositBox();
            }
            else if (gamepad2.right_trigger > 0.2) {
                ignore_automatic_depositbox = true;
                setDepositBox();
            } else {
                ignore_automatic_depositbox = false;
            }



//            if (gamepad2.y && !holdingY) {
//                driveSlow = !driveSlow;
//                holdingY = true;
//            } else {
//                holdingY = false;
//            }

            // Drive Train (REUSED CODE), except we cube the motor power to reduce it

            double robotSpeed;
            if (gamepad2.y) { robotSpeed = DRIVESPEED_SLOW; }
            else            { robotSpeed = DRIVESPEED_FAST; }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * robotSpeed,
                            -gamepad1.left_stick_x * robotSpeed,
                            Math.pow(-gamepad1.right_stick_x, 3) * robotSpeed
                    )
            );
            telemetry.addData("Right stick power", Math.pow(-gamepad1.right_stick_x, 3));
            drive.update();

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
            lastLoopTime = loopTimer.milliseconds();
            telemetry.addData("Loop time", lastLoopTime);
            telemetry.update();
        }
    }

    public void resetDepositBox() {
        /**
         * Reset the position of the deposit box thing to default
         */
        robot.depositLeft.setPosition(depositLeftPositions.normal);
//        robot.depositRight.setPosition(depositRightPositions.normal);
    }

    public void setDepositBox() {
        /**
         * Set the position of the deposit box thing to be moved out and ready for outtaking.
         */
        robot.depositLeft.setPosition(depositLeftPositions.out);
//        robot.depositRight.setPosition(depositRightPositions.out);
    }

    public void wheelKeepPixel() {
        robot.wheel.setPower(-1.0);
    }

    public void wheelSpitPixel() {
        robot.wheel.setPower(1.0);
    }

}
