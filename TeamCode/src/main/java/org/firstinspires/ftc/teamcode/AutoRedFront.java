package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Front Auto", group="Red Auto")
public class AutoRedFront extends LinearOpMode {
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

        // see where the prop is
        switch (location) {
            case LEFT:
                //      Deposit the Purple Pixel       //
                auto.moveForward(1000, 0.5);
                auto.turnLeft(0.5);
                auto.moveForward(10, 0.5);
                auto.intake(300, -0.4); // reverse the intake slowly

                //      Drive to the Backboard      //
                auto.turnRight(0.5);
                auto.turnRight(0.5);
                auto.moveForward(2500, 0.5);

//                //      Search for the left apriltag      //
//                if (scanAprilTag() == 4) {
//                    auto.turnRight(0.5);
//                    auto.moveForward(100, 0.5);
//                    auto.turnLeft(0.5);
////                    auto.moveRight(50);
//                }
//                else if (scanAprilTag() == 6) {
//                    auto.turnLeft(0.5);
//                    auto.moveForward(100, 0.5);
//                    auto.turnRight(0.5);
////                    auto.moveLeft(50);
//                }
//                else if (scanAprilTag() == 5) {
//                    auto.moveForward(200, 0.5);
//                    auto.outtake(400, 0.5);
//                }

                //      Park      //
                auto.moveForward(500, 0.5);
                break;
            case RIGHT:
                //      Deposit the Purple Pixel       //
                auto.moveForward(1000, 0.5);
                auto.turnRight(0.5);
                auto.intake(300, -0.4); // reverse the intake very slowly

                //      Drive to the Backboard      //
                auto.moveForward(1500, 0.5);

                //      Park      //
                auto.moveForward(500, 0.5);
                break;
            case MIDDLE:
                //      Deposit the Purple Pixel       //
                auto.moveForward(1200, 0.5);
                auto.intake(300, -0.4); // reverse the intake very slowly

                //      Drive to the Backboard      //
                auto.turnRight(0.5);
                auto.moveForward(2500, 0.5);

                //      Park      //
                auto.moveForward(500, 0.5);
                break;
        }

    }
}
