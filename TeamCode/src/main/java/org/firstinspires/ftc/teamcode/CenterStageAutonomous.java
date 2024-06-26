/**
 * Generic class with shared methods for the auto.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.util.TwoPositions;

public class CenterStageAutonomous {
    // intake heights, outtake position
    public static double resetIntakeHeight = 1.0;
    public static double intakeHeight1 = 0.8;
    public static double intakeHeight2 = 0.73;
    public static double intakeHeight3 = 0.63;
    public static volatile TwoPositions depositLeftPositions = new TwoPositions(1.0, 0.35);
    public static volatile TwoPositions depositRightPositions = new TwoPositions(0.0, 0.65);

    private RobotSleep theSleep;
    public Slides slides;

    private HardwareMap hardwareMap;
    private HardwarePushbot robot;
    private Telemetry telemetry;
    private FtcDashboard dashboard;
    private OpenCvWebcam camera;
    private VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;
    private ObjectPositionPipeline detector;
//    private CenterStageProcessor imgProcessor;

    public enum WhatColorToDetect {
        RED,
        BLUE
    }

    public static class coord {
        public double x;
        public double y;
        public double heading;
        public double maxErrorX;
        public double maxErrorY;
        public double maxErrorH;

        coord(double x, double y, double heading, double maxErrorX, double maxErrorY, double maxErrorH) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.maxErrorX = maxErrorX;
            this.maxErrorY = maxErrorY;
            this.maxErrorH = maxErrorH;
        }
    }

    public CenterStageAutonomous(HardwareMap hwMap, HardwarePushbot bot, Telemetry t, FtcDashboard dash, RobotSleep sleepFunc) {
        this.hardwareMap = hwMap;
        this.robot = bot;
        this.telemetry = t;
        this.dashboard = dash;
        this.theSleep = sleepFunc;

        slides = new Slides(
                this.telemetry,
                this.robot.slideLeft,
                this.robot.slideRight,
                this.robot.slidesEncoder
        );
    }


    // region miscellaneous stuff

    @FunctionalInterface
    public interface RobotSleep {
        void sleep(long milliseconds);
    }

    private void sleep(long milliseconds) {
        theSleep.sleep(milliseconds);
    }

    // endregion


    // region camera stuff

    public void initCamera(WhatColorToDetect colorToDetect) {
        // get webcam id and webcam
        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(robot.camera, cameraMonitorViewId);

        detector = new ObjectPositionPipeline(telemetry);
        switch (colorToDetect) {
            case RED:  detector.setDetectRed(true);  break;
            case BLUE: detector.setDetectRed(false); break;
        }
        camera.setPipeline(detector);

        camera.setMillisecondsPermissionTimeout(5000); // how much time for getting permission until timeout

        // open the webcam
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // when cam opens, start streaming
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                // put a telemetry saying there was an error maybe
                telemetry.addData("CRITICAL", "Failed to open camera. Code " + String.valueOf(errorCode));
                telemetry.update();
            }
        });





        //        imgProcessor = new CenterStageProcessor(telemetry);
//        imgProcessor = new CenterStageProcessor(dashboard, telemetry);
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
////                .addProcessor(imgProcessor)
//                .setCamera(robot.camera)
////                .setCamera(BuiltinCameraDirection.BACK)
//                .setCameraResolution(new Size(320, 240))
//                .build();

    }

    public ObjectPositionPipeline.Location getPropLocation() {
        return this.detector.getPropLocation();
    }

    public void stopCameraStreaming() {
        this.camera.stopStreaming();
    }

    // endregion


    // region outtake stuff
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

    public void wheelKeepPixel(int milliseconds) {
        robot.wheel.setPower(-1.0);
        sleep(milliseconds);
        robot.wheel.setPower(0.0);
    }

    public void wheelSpitPixel(int milliseconds) {
        robot.wheel.setPower(1.0);
        sleep(milliseconds);
        robot.wheel.setPower(0.0);
    }


    // endregion


    // region movement stuff

    public void moveForward(int milliseconds, double power) {
        /**
         * Move forward for some time.
         * @param milliseconds - How long to move forward for, in milliseconds
         * @param power - The power to use for the motors
         */
        this.moveDrivetrain(
                power,
                power,
                power,
                power
        );
        this.sleep(milliseconds);
        this.CancelPowerRobot();
    }

    public void moveBackward(int milliseconds, double power) {
        /**
         * Move backward for some time. Simply a convenience function which calls moveForward().
         * @param milliseconds - How long to move backward for, in milliseconds
         * @param power - The power to use for the motors
         */
        this.moveForward(milliseconds, -power);
    }

    public void strafeLeft(int milliseconds, double power) {
        /**
         * Strafe left for some time.
         * @param milliseconds - How long to strafe for, in milliseconds
         * @param power - The power to use for the motors
         */
        this.moveDrivetrain(
                -power,
                 power,
                 power,
                -power
        );
        this.sleep(milliseconds);
        this.CancelPowerRobot();
    }

    public void spinLeft(int milliseconds, double power) {
        /**
         * Table for how to turn 90 degrees at different powers
         * | power | milliseconds
         * | 1.0   | 229
         * | .75   | 347
         * | .50   | 650
         * | .40   | 932
         * | .30   | 1540
         */
//        power=-power;
        this.moveDrivetrain(
                -power,
                power,
                -power,
                power
        );
        this.sleep(milliseconds);
        this.CancelPowerRobot();
    }

    public void spinRight(double power) {
//        power=-power;
        this.moveDrivetrain(
                power,
                -power,
                power,
                -power
        );
//        robot.motorFrontLeft.setPower(power);
//        robot.motorFrontRight.setPower(power);
//        robot.motorBackLeft.setPower(-power);
//        robot.motorBackRight.setPower(-power);
    }

    public void spinRight(int milliseconds, double power) {
        spinLeft(milliseconds, -power);
    }

//    public void LeftSlantDrive(double power) {
//        power=-power;
//        robot.motorFrontLeft.setPower(0.0);
//        robot.motorFrontRight.setPower(power);
//        robot.motorBackRight.setPower(0.0);
//        robot.motorBackLeft.setPower(power);
//    }
//
//    public void RightSlantDrive(double power) {
//        power = -power;
//        robot.motorFrontLeft.setPower(power);
//        robot.motorFrontRight.setPower(0.0);
//        robot.motorBackRight.setPower(-power);
//        robot.motorBackLeft.setPower(0.0);
//    }
//
//    public void RightSlantRearDrive(double power) {
//        robot.motorFrontLeft.setPower(0.0);
//        robot.motorFrontRight.setPower(power);
//        robot.motorBackRight.setPower(0.0);
//        robot.motorBackLeft.setPower(power);
//    }
//
//    public void LeftSlantRearDrive(double power) {
//        robot.motorFrontLeft.setPower(power);
//        robot.motorFrontRight.setPower(0.0);
//        robot.motorBackRight.setPower(-power);
//        robot.motorBackLeft.setPower(0.0);
//    }

    public void CancelPowerRobot() {
        this.moveDrivetrain(0, 0, 0, 0);
    }

    // endregion


    // region pixel stuff

    public void intake(int milliseconds) {
        /**
         * @param milliseconds - How long to intake for, in milliseconds
         */
        intake(milliseconds, 1.0);
    }

    public void intake(int milliseconds, double power) {
        /**
         * @param milliseconds - How long to intake for, in milliseconds
         * @param power - The power to use for the motors
         */
        robot.intake.setPower(-power);
        robot.intakeControl.setPosition(0.5);
        robot.wheel.setPower(power);
        sleep(milliseconds);
        robot.intake.setPower(power);
        robot.intakeControl.setPosition(0.5);
        robot.wheel.setPower(-power);
    }

    public void reverseIntake(int milliseconds, double power) {
        /**
         * Reverse the intake.
         * @param milliseconds - How long to reverse the intake for, in milliseconds
         * @param power - The power to use
         */
        robot.intake.setPower(-power);
        robot.intakeControl.setPosition(0.5);
        sleep(milliseconds);
        robot.intakeControl.setPosition(1.0);
    }

    public void intakeOn(int mode) {
        robot.wheel.setPower(-1.0);
        this.resetDepositBox();
        if(mode == 1){
            robot.intakeControl.setPosition(intakeHeight1);
        }
        if(mode == 2){
            robot.intakeControl.setPosition(intakeHeight2);
        }
        if(mode == 3){
            robot.intakeControl.setPosition(intakeHeight3);
        }
        sleep(550);

        robot.intake.setPower(-1.0);
        sleep(1500);

    }

    public void intakeOff() {
        robot.intakeControl.setPosition(resetIntakeHeight);
        robot.wheel.setPower(0.0);
        robot.intake.setPower(0.0);
    }

    private void moveDrivetrain(
            double frontLeft,
            double frontRight,
            double rearLeft,
            double rearRight
    ) {
        /**
         * Sets power for the drivetrain motors. This function exists because there is some sus
         * negative stuff.
         */
        telemetry.addData("Front left power", frontLeft);
        telemetry.addData("Front right power", -frontRight);
        telemetry.addData("Back left power", -rearLeft);
        telemetry.addData("Back right power", rearRight);
        telemetry.update();
        this.robot.motorFrontLeft.setPower(frontLeft); // some of these might need to be negative
        this.robot.motorFrontRight.setPower(-frontRight);
        this.robot.motorBackLeft.setPower(-rearLeft);
        this.robot.motorBackRight.setPower(rearRight);
    }

    //    public void outtake(int milliseconds, double power) {
//        /**
//         * @param milliseconds - How long to outtake for, in milliseconds
//         * @param power - The power to use for the servos
//         */
//        robot.intake.setPower(power);
//        sleep(milliseconds);
//        robot.intake.setPower(0);
//    }

    // endregion

}
