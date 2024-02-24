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
    public static volatile TwoPositions depositLeftPositions = new TwoPositions(0.86, 0.1);
    public static volatile TwoPositions depositRightPositions = new TwoPositions(0.14, 0.9);

    // Other classwide items
    private HardwarePushbot robot = new HardwarePushbot();
    private FtcDashboard dashboard;

    // idk why this is classwide
    boolean depositBoxIsOut = false;
    int wheelState = 0; // 0 is not spinning, 1 is keeping, -1 is spitting

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
        PidDriveTrain follower = new PidDriveTrain(
                hardwareMap,
                telemetry
        );

        wheelState = 0;
        depositBoxIsOut = false;
        boolean shouldResetDepositBox = false;
        ElapsedTime depositBoxTimer = new ElapsedTime();
        boolean ignore_automatic_depositbox = false;
        boolean intaking;
        boolean planeReleased = false;
//        double lastLoopTime = 0;

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

            // region Update Slide Positions
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                // we do negative since our gamepad stick is sus
//                slides.move(-gamepad2.left_stick_y * SLIDESPEED * lastLoopTime);
                slides.move(-gamepad2.left_stick_y * SLIDESPEED);
//                telemetry.addData("move amount", -gamepad2.left_stick_y * SLIDESPEED);
            }

            if (gamepad2.dpad_left) {
                slides.moveToLineOne();
            } else if (gamepad2.dpad_up) {
                slides.moveToLineTwo();
            } else if (gamepad2.dpad_right) {
                slides.moveToLineThree();
            } else if (gamepad2.dpad_down) {
                // start timer to move to the bottom
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
            } else if (depositBoxIsOut && !ignore_automatic_depositbox && slidePosition <= DEPOSIT_OUT_HEIGHT) {
                resetDepositBox();
            }

            // endregion

            // region Outtake Box
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
            // endregion

            // region Driving

            double robotSpeed;
            if (gamepad2.y) { robotSpeed = DRIVESPEED_SLOW; }
            else            { robotSpeed = DRIVESPEED_FAST; }

            if(gamepad2.right_bumper) {
                // HEADING LOCK!!! cool backdrop stuff
                follower.setTargetPosition(0, 0, 0, 1000000, 1000000, 0.02);
                // 0.5 degrees max off, 0.01 rad = 0.5 deg.
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * robotSpeed,
                                -gamepad1.left_stick_x * robotSpeed,
                                follower.powerH()
                        )
                );
            }
            else {
                // Regular driving
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * robotSpeed,
                                -gamepad1.left_stick_x * robotSpeed,
                                Math.pow(-gamepad1.right_stick_x, 3) * robotSpeed
                        )
                );
            }



            // endregion

            // region Intake
            if (gamepad2.x) {
                // Intake Normal
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
            // endregion

            // region Outtake Box Wheel
            if (intaking || gamepad1.left_trigger > 0.2) {
                //Keep Pixel
                wheelKeepPixel();
            }
            else if (gamepad1.right_trigger > 0.2) {
                // spit pixel
                wheelSpitPixel();
            }
            else if ((wheelState != 0) || !intaking) {
                // do nothing
                wheelState = 0;
                robot.wheel.setPower(0.0);
            }
            // endregion

            // region Plane
            if (!planeReleased) {
                if(gamepad2.a) {
                    // rlease the plane
                    planeReleased = true;
                    robot.plane.setPosition(1.0);
                }
            }
            // endregion

            // region Set Power to Slides
            double slidePower = slides.updateSlides();
            slides.setSlidePower(slidePower);
            // endregion

            // region Emergency Break
            if ((gamepad1.left_bumper && gamepad1.right_bumper)) { break; }
            // endregion

            // update all of the telemetry at the end of each loop iteration
//            telemetry.addData("Loop time", loopTimer.milliseconds());
            telemetry.addData("Deposit left position", robot.depositLeft.getPosition());
            telemetry.update();
        }
    }

    public void resetDepositBox() {
        /**
         * Reset the position of the deposit box thing to default
         */
        if (depositBoxIsOut) {
            depositBoxIsOut = false;
            robot.depositLeft.setPosition(depositLeftPositions.normal);

            robot.depositRight.setPosition(depositRightPositions.normal);
        }
    }

    public void setDepositBox() {
        /**
         * Set the position of the deposit box thing to be moved out and ready for outtaking.
         */
        if (!depositBoxIsOut) {
            depositBoxIsOut = true;
            robot.depositLeft.setPosition(depositLeftPositions.out);
            robot.depositRight.setPosition(depositRightPositions.out);
        }
    }

    public void wheelKeepPixel() {
        if (wheelState != 1) {
            wheelState = 1;
            robot.wheel.setPower(-1.0);
        }
    }

    public void wheelSpitPixel() {
        if (wheelState != -1) {
            wheelState = -1;
            robot.wheel.setPower(1.0);
        }
    }

}
