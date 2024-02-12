package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Front Auto", group="Blue Auto")
public class AutoBlueFront extends LinearOpMode {
    private FtcDashboard dashboard;
    private CenterStageAutonomous auto;
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        auto = new CenterStageAutonomous(
                hardwareMap,
                robot,
                telemetry,
                dashboard,
                (milliseconds) -> sleep(milliseconds)
        );

        auto.initCamera(CenterStageAutonomous.WhatColorToDetect.RED);
        waitForStart();

        ObjectPositionPipeline.Location location = auto.getPropLocation();
        auto.stopCameraStreaming();

        auto.strafeRight(800, 0.7);
        auto.spinRight(500, 0.5);

        // see where the prop is
        switch (location) {
            case LEFT:
                //      Deposit the Purple Pixel       //
                auto.spinRight(300, 0.5);
                auto.reverseIntake(725, -0.5);
                auto.spinLeft(300, 0.5);
                break;
            case RIGHT:
                auto.spinRight(300, -0.5);
                auto.reverseIntake(725, -0.5);
                auto.spinLeft(300, -0.5);
                break;
            case MIDDLE:
                auto.reverseIntake(725, -0.5);
                break;
        }
        auto.spinRight(725, 0.5);

        auto.moveForward(1300, 0.5);

//        auto.slides.setTargetPosition(1200);
//        auto.slides.updateSlidesAuto();
////        while (!auto.slides.destinationIsReached()) {
////            auto.slides.setSlidePower(auto.slides.updateSlides());
////        }
//        auto.setDepositBox();
//        sleep(500);
//        switch (location){
//            case LEFT:
//                auto.strafeRight(300, 0.5);
//                auto.wheelSpitPixel();
//                sleep(200);
//                auto.strafeRight(300, -0.5);
//                break;
//            case MIDDLE:
//                auto.strafeLeft(150, 0.5);
//                auto.wheelSpitPixel();
//                sleep(200);
//                auto.strafeLeft(150, -0.5);
//                break;
//            default:
//                auto.wheelSpitPixel();
//                sleep(200);
//                break;
//        }
//        auto.resetDepositBox();
//        sleep(500);
//        auto.slides.setTargetPosition(0);
//        auto.slides.updateSlidesAuto();

        sleep(1000);
        auto.strafeRight(1250, 0.8);
        sleep(1000);
    }

}
