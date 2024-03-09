package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Auto", group="Concept")
public class AutoTest extends LinearOpMode {
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

        auto.initCamera(CenterStageAutonomous.WhatColorToDetect.BLUE);
        waitForStart();

        ObjectPositionPipeline.Location location = auto.getPropLocation();
        auto.stopCameraStreaming();
        switch (location) {
            case LEFT:
                telemetry.addData("FINAL Location", "Left");
            case MIDDLE:
                telemetry.addData("FINAL Location", "Middle");
            case RIGHT:
                telemetry.addData("FINAL Location", "Right");
        }
        telemetry.update();

        auto.strafeLeft(850, 0.7);
        auto.spinLeft(630, 0.5);
        auto.intake(500, -0.4);
//        auto.moveForward(500, 0.2);

//        //      Deposit the Purple Pixel       //
//        auto.strafeLeft(1200, 0.5);
//        auto.turnLeft(0.5);
//        auto.intake(300, -0.4); // reverse the intake very slowly
//
//        //      Drive to the Backboard      //
//        auto.turnRight(0.5);
//        auto.moveForward(2500, 0.5);
//
//        //      Park      //
//        auto.moveForward(500, 0.5);

        auto.slides.setTargetPosition(1100);
//        auto.slides.updateSlidesAuto();
//        while (!auto.slides.destinationIsReached()) {
//            auto.slides.setSlidePower(auto.slides.updateSlides());
//        }
        auto.setDepositBox();
//        auto.wheelSpitPixel();



        auto.slides.moveToBottom();

    }
}
