package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Back Auto", group="Red Auto")
public class AutoBlueBack extends LinearOpMode {
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

        auto.moveBackward(500, 0.5);

    }

}
