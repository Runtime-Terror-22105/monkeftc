package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CenterStageAutonomous.coord;

@Config
@Autonomous(name="PID TESTINGGGG", group="TESTINGG")
public class pidtesting extends LinearOpMode {
    private FtcDashboard dashboard;
    private CenterStageAutonomous auto;
    HardwarePushbot robot = new HardwarePushbot();
    public static int numPoints = 1;
    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;
    public static double maxError = 0;

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
        auto.stopCameraStreaming(); // camera stuff, copied from earlier
        PidDriveTrain follower = new PidDriveTrain(
                hardwareMap,
                telemetry
        );
//        int p = 0;

        while(opModeIsActive()) {
            follower.setTargetPosition(x, y, heading, maxError, maxError, maxError);

            if(follower.reached()) {
                telemetry.addData("status", "reached");
                telemetry.update();
            }
            else {
                double x = follower.powerY();
                double y = follower.powerX();
                double rx = follower.powerH();
                // update power based on PID, similar method to telop
                robot.motorFrontLeft.setPower(-(y + x + rx));
                robot.motorBackLeft.setPower(y - x + rx);
                robot.motorFrontRight.setPower(y - x - rx);
                robot.motorBackRight.setPower(-(y + x - rx));

            }
            telemetry.addData("errorX", follower.errorX);
            telemetry.addData("errorY", follower.errorY);
            telemetry.addData("errorHeading", follower.errorH);
            telemetry.update();
        }

    }

}
