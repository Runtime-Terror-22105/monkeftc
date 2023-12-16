package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Camera Test", group="Concept")
public class Auto extends LinearOpMode {
    private OpenCvWebcam camera;
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // get live viewport
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
        ObjectPositionPipeline detector = new ObjectPositionPipeline(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000); // how much time for getting permission until timeout

        //
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // when cam opens, start streaming
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // put a telemetry saying there was an error maybe
                telemetry.addData("CRITICAL", "Failed to open camera. Code " + String.valueOf(errorCode));
            }
        });

        waitForStart();

        // get to the area with the prop, this will depend on where the robot starts
        // ...

        // see where the prop is
//        switch (detector.getLocation()) {
//            case LEFT:
//                // ...
//                break;
//            case RIGHT:
//                // ...
//                break;
//            case MIDDLE:
//                // ...
//                break;
//        }

        //

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
