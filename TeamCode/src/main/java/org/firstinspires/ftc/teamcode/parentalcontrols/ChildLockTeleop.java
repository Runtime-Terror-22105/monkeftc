package org.firstinspires.ftc.teamcode.parentalcontrols;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PidDriveTrain;
import org.firstinspires.ftc.teamcode.Slides;
import org.firstinspires.ftc.teamcode.util.TwoPositions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sussyteleop.TeleopMecanumDrive;

import java.lang.Math;

@Config
@TeleOp(name = "Child Lock Teleop", group = "Child Lock")
public class ChildLockTeleop extends LinearOpMode  {

    // Dynamic constants
    public static volatile double DRIVESPEED_FAST = 0.4; // between 0 and 1
    public static volatile double DRIVESPEED_SLOW = 0.3; // between 0 and 1
    public static volatile double SLIDESPEED = 21; // must be whole num
    public static volatile double MAX_TELEOP_VEL = Math.pow(10, 6); // 92.61691602936227
    public static volatile double MAX_TELEOP_ACCEL = Math.pow(10, 6); // 92.61691602936227
    public static volatile TwoPositions intakePositions = new TwoPositions(1.0, 0.65);
    // Other classwide items
    private HardwareChildLockBot robot = new HardwareChildLockBot();
    private FtcDashboard dashboard;

    public void runOpMode() {
        // do some initialization
        robot.init(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        Telemetry _tele = telemetry;
        MultipleTelemetry telemetry = new MultipleTelemetry(_tele, dashboard.getTelemetry());

        // sus driving init
        TeleopMecanumDrive drive = new TeleopMecanumDrive(hardwareMap, MAX_TELEOP_VEL, MAX_TELEOP_ACCEL);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PidDriveTrain follower = new PidDriveTrain(
                hardwareMap,
                telemetry
        );
        follower.updatePos();

        boolean intaking;
        boolean planeReleased = false;

        Slides slides = new Slides(
                telemetry,
                robot.slideLeft,
                robot.slideRight,
                robot.slidesEncoder
        );

        waitForStart();

        while (opModeIsActive()) {
            // region Update Slide Positions
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                // we do negative since our gamepad stick is sus
                slides.move(-gamepad2.left_stick_y * SLIDESPEED);
            }

            double slidePosition = robot.slidesEncoder.getCurrentPosition();
            if (gamepad2.dpad_left) {
                slides.moveToLineOne();
            } else if (gamepad2.dpad_up) {
                slides.moveToLineTwo();
            } else if (gamepad2.dpad_right) {
                slides.moveToLineThree();
            } else if (gamepad2.dpad_down && slidePosition >= 750) {
                slides.moveToBottom();
            }

            // endregion

            // region Driving

            double robotSpeed;
            if (gamepad2.y || gamepad1.y) { robotSpeed = DRIVESPEED_SLOW; }
            else                          { robotSpeed = DRIVESPEED_FAST; }
            double heading_power = Math.pow(-gamepad1.right_stick_x, 3) * robotSpeed;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * robotSpeed,
                            -gamepad1.left_stick_x * robotSpeed,
                            heading_power
                    )
            );



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
                robot.intake.setPower(0.7);
                robot.intakeControl.setPosition(intakePositions.out);
            }
            else {
                // no power intake
                intaking = false;
                robot.intake.setPower(0.0);
                robot.intakeControl.setPosition(intakePositions.normal);
            }
            // endregion

            // region Plane
            if (!planeReleased) {
                if(gamepad2.a) {
                    // release the plane
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
            telemetry.update();
        }
    }

}
