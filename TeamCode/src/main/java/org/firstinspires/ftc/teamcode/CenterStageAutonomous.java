/**
 * Generic class with shared methods for the auto.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CenterStageAutonomous {
    public final int RIGHTANGLETURNTIME = 200;

    private RobotSleep theSleep;

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

    public CenterStageAutonomous(HardwareMap hwMap, HardwarePushbot bot, Telemetry t, FtcDashboard dash, RobotSleep sleepFunc) {
        this.hardwareMap = hwMap;
        this.robot = bot;
        this.telemetry = t;
        this.dashboard = dash;
        this.theSleep = sleepFunc;
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
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
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
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
//                .addProcessor(imgProcessor)
                .setCamera(robot.camera)
//                .setCamera(BuiltinCameraDirection.BACK)
                .setCameraResolution(new Size(320, 240))
                .build();

    }

    public ObjectPositionPipeline.Location getPropLocation() {
        return detector.getPropLocation();
    }

    public void stopCameraStreaming() {
        camera.stopStreaming();
    }

    // endregion


    // region movement stuff

    public void moveForward(int milliseconds, double power) {
        /**
         * Move forward for some time.
         * @param milliseconds - How long to move forward for, in milliseconds
         * @param power - The power to use for the motors
         */
        FrontDrive(-power);
        sleep(milliseconds);
        CancelPowerRobot();
    }

    public void moveBackward(int milliseconds, double power) {
        /**
         * Move backward for some time.
         * @param milliseconds - How long to move forward for, in milliseconds
         * @param power - The power to use for the motors
         */
        moveForward(1000, -power);
    }

    public void turnLeft(double power) {
        spinLeft(power);
        long sleeptime = (long)(RIGHTANGLETURNTIME/power); // tweak this value with trial and error
        sleep(sleeptime);
        CancelPowerRobot();
    }

    public void turnRight(double power) {
        spinRight(power);
        long sleeptime = (long)(RIGHTANGLETURNTIME/power);
        sleep(sleeptime);
        CancelPowerRobot();
    }

    public void spinLeft(double power) {
//        power=-power;
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
        robot.motorBackRight.setPower(power);
    }

    public void spinRight(double power) {
//        power=-power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackLeft.setPower(-power);
        robot.motorBackRight.setPower(-power);
    }

    public void FrontDrive(double power) {
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorBackRight.setPower(-power);
    }

    public void LeftSlantDrive(double power) {
        power=-power;
        robot.motorFrontLeft.setPower(0.0);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(0.0);
        robot.motorBackLeft.setPower(power);
    }

    public void RightSlantDrive(double power) {
        power = -power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(0.0);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(0.0);
    }

    public void RightSlantRearDrive(double power) {
        robot.motorFrontLeft.setPower(0.0);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(0.0);
        robot.motorBackLeft.setPower(power);
    }

    public void LeftSlantRearDrive(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(0.0);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(0.0);
    }

    public void RearDrive(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }

    public void SlideRight(double power) {
//        power = -power;
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackLeft.setPower(-power);
        robot.motorBackRight.setPower(-power);

    }

    public void SlideLeft(double power) {
//        power = -power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorBackRight.setPower(power);
    }

    public void CancelPowerRobot() {
        double power = 0.0;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }

    // endregion


    // region pixel stuff

    public void intake(int milliseconds) {
        /**
         * @param milliseconds - How long to intake for, in milliseconds
         */
        robot.intake.setPower(-1.0);
        robot.intakeControl.setPosition(0.5);
        robot.wheel.setPower(1.0);
        sleep(milliseconds);
        robot.intake.setPower(1.0);
        robot.intakeControl.setPosition(0.5);
        robot.wheel.setPower(-1.0);
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
        intake(milliseconds, power);
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
