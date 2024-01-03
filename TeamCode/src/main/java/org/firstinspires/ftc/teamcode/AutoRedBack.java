//package org.firstinspires.ftc.teamcode;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@Autonomous(name="Red Back Auto", group="Red Auto")
//public class AutoRedBack extends LinearOpMode {
//    private FtcDashboard dashboard;
//    private CenterStageAutonomous auto;
//    HardwarePushbot robot = new HardwarePushbot();
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        auto = new CenterStageAutonomous(
//                hardwareMap,
//                robot,
//                telemetry,
//                dashboard,
//                (milliseconds) -> sleep(milliseconds)
//        );
//
//        auto.initCamera(CenterStageAutonomous.WhatColorToDetect.RED);
//        waitForStart();
//
//        ObjectPositionPipeline.Location location = auto.getLocation();
//        auto.stopLocationDetection();
//
//        // see where the prop is
//        switch (location) {
//            case LEFT:
//                //      Deposit the Purple Pixel       //
//                auto.moveForward(1000, 0.5);
//                auto.turnLeft(0.5);
//                auto.moveForward(10, 0.5);
//                auto.intake(300, -0.4); // reverse the intake slowly
//
//                //      Drive to the Backboard      //
//                auto.turnRight(0.5);
//                auto.moveForward(4000, 0.5);
//                auto.turnRight(0.5);
//                auto.moveForward(900, 0.5);
//                auto.turnLeft(0.5);
//
//                //      Park      //
//                auto.moveForward(500, 0.5);
//                break;
//            case RIGHT:
//                //      Deposit the Purple Pixel       //
//                auto.moveForward(1000, 0.5);
//                auto.turnRight(0.5);
//                auto.moveForward(10, 0.5);
//                auto.intake(300, -0.4); // reverse the intake very slowly
//
//                //      Drive to the Backboard      //
//                auto.turnLeft(0.5);
//                auto.moveForward(1500, 0.5);
//                auto.turnRight(0.5);
//                auto.moveForward(3800, 0.5);
//                auto.turnRight(0.5);
//                auto.moveForward(900, 0.5);
//                auto.turnLeft(0.5);
//
//                //      Park      //
//               auto. moveForward(500, 0.5);
//                break;
//            case MIDDLE:
//                //      Deposit the Purple Pixel       //
//                auto.moveForward(1200, 0.5);
//                auto.SlideLeft(0.5); // TODO: Combine this + next two statements into one function
//                sleep(100);
//                auto.CancelPowerRobot();
//                auto.intake(300, -0.4); // reverse the intake very slowly
//
//                //      Drive to the Backboard      //
//                auto.SlideLeft(0.5); // TODO: Combine this + next two statements into one function
//                sleep(100);
//                auto.CancelPowerRobot();
//                auto.moveForward(1500, 0.5);
//                auto.turnRight(0.5);
//                auto.moveForward(3800, 0.5);
//                auto.turnRight(0.5);
//                auto.moveForward(900, 0.5);
//                auto.turnLeft(0.5);
//
//                //      Park      //
//                auto.moveForward(500, 0.5);
//                break;
//        }
//
//    }
//}
