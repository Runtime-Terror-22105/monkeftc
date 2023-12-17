package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Processors.TmpProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="BLUE Auto", group="Concept")
public class BlueAuto extends LinearOpMode {
    private OpenCvWebcam camera;
    private VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;
    private TmpProcessor imgProcessor;
    private ObjectPositionPipeline detector;
    private FtcDashboard dashboard;
    HardwarePushbot robot = new HardwarePushbot();

    public enum WhatColorToDetect {
        RED,
        BLUE
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        initCamera(hardwareMap, WhatColorToDetect.BLUE);

        waitForStart();
        ObjectPositionPipeline.Location location = detector.getLocation();
        stopLocationDetection();



        // see where the prop is
        switch (location) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case MIDDLE:
                // ...
                break;
        }

    }

    public void initCamera(HardwareMap hwMap, WhatColorToDetect colorToDetect) {
        // get webcam id and webcam
        int cameraMonitorViewId = hwMap
                .appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hwMap.appContext.getPackageName()
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
        imgProcessor = new TmpProcessor(telemetry);
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
                .setCameraResolution(new Size(320, 240))
                .build();

    }

    public void stopLocationDetection() {
        camera.stopStreaming();
    }

    public void spin(double power) {
        power=-power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
    }

    public void FrontDrive(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower((312.0/435.0) * power);
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
        power = -power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(-power);

    }

    public void SlideLeft(double power) {
        power = -power;
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);

    }

    public void CancelPowerRobot() {
        double power=0.0;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }

    public void robotsleep(long time) {
        sleep(time);
    }

}
