package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CenterStageAutonomous.coord;

@Autonomous(name="PID to Point", group="PID to Point")
public class PidToPoint extends LinearOpMode {
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
        auto.stopCameraStreaming(); // camera stuff, copied from earlier
        coord[] points = new coord[1000];
        Slides slides = new Slides(
                telemetry,
                robot.slideLeft,
                robot.slideRight,
                robot.slidesEncoder
        );
        PidDriveTrain follower = new PidDriveTrain(
                hardwareMap,
                telemetry
        );
        int numPoints = 0;
        int p = 0;

        while(opModeIsActive()){ // continuous while loop for program
            follower.setTargetPosition(points[p].x, points[p].y, points[p].heading, points[p].maxError, points[p].maxError, points[p].maxError);

            if(follower.reached()){
                // reached position;
                p++;
                if(p == numPoints){ // auto done
                    break;
                }
                switch(p){
                    // add code here to do different things based on p value
                    // i.e start spinning intake, move slides up, etc.
                    case 1:
                        // code
                        break;
                }
            }
            else{
                double x = follower.powerX();
                double y = follower.powerY();
                double rx = follower.powerH();
                // update power based on PID, similar method to telop
                robot.motorFrontLeft.setPower(y + x + rx);
                robot.motorBackLeft.setPower(y - x + rx);
                robot.motorFrontRight.setPower(y - x - rx);
                robot.motorBackRight.setPower(y + x - rx);

            }

            double slidePower = slides.updateSlides();
            slides.setSlidePower(slidePower); // also update slides power, copied from teleop
        }

    }

}
