package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CenterStageAutonomous.coord;

@Autonomous(name="2+0 Blue Front Auto", group="concept")
public class AutoBlueFront extends LinearOpMode {
    private FtcDashboard dashboard;
    private CenterStageAutonomous auto;
    HardwarePushbot robot = new HardwarePushbot();
//    public static volatile TwoPositions depositLeftPositions = new TwoPositions(0.73, 0.0);
//    public static volatile TwoPositions depositRightPositions = new TwoPositions(0.27, 1.0);
    public static double resetIntakeHeight = 1.0;
    public static double intakeHeight1 = 0.79;
    public static double intakeHeight2 = 0.72;
    public static double intakeHeight3 = 0.70;

    public static int depositOnePixel = 1000;

    public coord[] points = new coord[100];
    public static double backDistance = -34.6;
    public static coord leftDepo = new coord(backDistance, 20.536, 4.71239, 1, 0.8, Math.toRadians(3));
    public static coord rightDepo = new coord(backDistance, 39.8456, 4.71239, 1, 0.8, Math.toRadians(3));
    public static coord centerDepo = new coord(backDistance, 30.351, 4.71239, 1, 0.8, Math.toRadians(3));

    // may need to adjust these intake positions
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
                this::sleep
        );
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

        auto.initCamera(CenterStageAutonomous.WhatColorToDetect.BLUE);
        waitForStart();

        ObjectPositionPipeline.Location location = auto.getPropLocation();
        auto.stopCameraStreaming(); // camera stuff, copied from earlier
        int[] code = new int[100];
        /*
        Code represents action to do for that position
        0: Nothing
        1: Intake & Wheel for pixel 5
        2: Intake & Wheel for pixel 3-4
        3: Intake & Wheel for pixel 1-2
        4: Slides Up (LOW) and intake off up and down
        5: Slides Up (HIGH) and intake off
        6. Deposit pixel (wait).
        7: Deposit pixel (wait), outatke in, and slides down.
        8: Sleep for a little bit.
        */

        int p = 0;
        int c = 0;
        switch(location) {
            case LEFT:
                points[0] = new coord(-5, 23.941, 5.83376302, 2, 2, Math.toRadians(3)); // place on purple pixel mark
                break;
            case MIDDLE:
                points[0] = new coord(0, 25.8979, 0, 2, 2, Math.toRadians(3)); // place on purple pixel mark
                break;
            case RIGHT:
                points[0] = new coord(0, 12.8324, 0.27, 2, 2, Math.toRadians(3)); // place on purple pixel mark
                points[1] = new coord(6.6758, 23.8324, 0.7125856101, 2, 2, Math.toRadians(3)); // place on purple pixel mark
                code[0] = 69;
                c = 1;
                break;
        }

        code[c] = 12;
        c++;
        points[c] = new coord(-1.8901, 8.865, 0, 2, 2, Math.toRadians(3)); // place on purple pixel marknew coord(0, 29.8979, 0, 2, 2, Math.toRadians(3)); // place on purple pixel mark
        code[c] = 20;
        c++;

        switch(location) {
            case LEFT:
                points[c] = leftDepo;
                break;
            case MIDDLE:
                points[c] = centerDepo;
                break;
            case RIGHT:
                points[c] = rightDepo;
                break;
        }
        code[c] = 6;
        c++;

        points[c] = new coord(-32.22, -0.489, 4.71239, 0.5, 0.5, Math.toRadians(1));
        code[c] = -100;
        c++;
        points[c] = new coord(-32.22, -0.489, 4.71239, 0.5, 0.5, Math.toRadians(1));
        code[c] = -100000;
        c++;


        int numPoints = c;
        int isSus = 2;
        auto.resetDepositBox();
        while (opModeIsActive()) { // continuous while loop for program
            double slidePower = slides.updateSlides();
            slides.setSlidePower(slidePower); // also update slides power, copied from teleop
            telemetry.addData("current point", p);
            telemetry.addData("current code", code[p]);



            if (p != numPoints) {
                follower.setTargetPosition(
                        points[p].y,
                        points[p].x,
                        points[p].heading,
                        1.8,
                        1.8,
                        points[p].maxErrorH
                );
            }

            if (isSus == 2 && p != numPoints && follower.reached()) {
                // I did this to try and be fast
                // If note, we can just do it always
                    robot.motorBackLeft.setPower(0);
                    robot.motorBackRight.setPower(0);
                    robot.motorFrontLeft.setPower(0);
                    robot.motorFrontRight.setPower(0);



                if(!slides.reached){
                    continue; // we don't want to move to the next point while slides are still moving
                    // as something could mess up
                }


                telemetry.addData("reached lol", "hi rahul");
                // reached position, do task
                     /*
            Code represents action to do for that position
            0: Nothing except intake up
            1: Intake & Wheel for pixel 5
            2: Intake & Wheel for pixel 3-4
            3: Intake & Wheel for pixel 1-2
            4: Intake off, Slides up to set line 1
            5. Deposit pixel and move slides to BELOW set line 1 (for yellow)
            6: Deposit pixel and do sussy ahh resetting
            7: Deposit pixel and reset regularly (wayy better...)
            */
                isSus = 2;
                switch(code[p]) { // will code this after
                    case 12:
                        auto.resetDepositBox();
                        slides.setTargetPosition(1300);
                        sleep(600);
                    case 20:
                        slides.setTargetPosition(800);
                        auto.setDepositBox();
                    case -1:
                        intakeOff();
                        break;
                    case 0:
                        intakeOff();
                        break;
                    case 1:
                        intakeOn(1);
                        break;
                    case 2:
                        intakeOn(2);
                        break;
                    case 3:
                        intakeOn(3);
                        break;
                    case 4:
                        intakeOff();
                        slides.moveToLineOne();
                        break;
                    case 5:
                        wheelSpitPixel(depositOnePixel);
                        slides.setTargetPosition(1000);
                        // approximate height for proper yellow pixel placement
                        break;
                    case 6:
                        wheelSpitPixel(depositOnePixel);
                        slides.setTargetPosition(1600);
                        // Basically, we first have to go UP and then backdown..
                        // Very weird unfortunate consequence of bad planning of camera location
                        break;
                    case 7:
                        wheelSpitPixel(2 * depositOnePixel);
                        break;
                    case -100:
                        slides.moveToBottom();
                        auto.resetDepositBox();
                        sleep(600);
                        break;
                }
                p++;
            }
            else if (p != numPoints){
                double xTemp = follower.powerY();
                double yTemp = follower.powerX();
                double rx = follower.powerH();
                double angle = follower.curH;
                // Ok I don't know if we should x and y here, so let's try this first
                // If it doesn't work, switch the sins and cosines
                double x = xTemp * Math.cos(angle) - yTemp * Math.sin(angle);
                double y = xTemp * Math.sin(angle) + yTemp * Math.cos(angle);

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                // update power based on PID, similar method to telop
                robot.motorFrontLeft.setPower(   -(y + x + rx) / denominator);
                robot.motorBackLeft.setPower(    (y - x + rx) / denominator);
                robot.motorFrontRight.setPower( (y - x - rx) / denominator);
                robot.motorBackRight.setPower(-(y + x - rx) / denominator);
            }


            telemetry.update();
        }
    }

//    public void resetDepositBox() {
//        robot.depositLeft.setPosition(depositLeftPositions.normal);
//        robot.depositRight.setPosition(depositRightPositions.normal);
//
//    }
//
//    public void setDepositBox() {
//        robot.depositLeft.setPosition(depositLeftPositions.out);
//        robot.depositRight.setPosition(depositRightPositions.out);
//    }

    public void intakeOn(int mode) {
        robot.wheel.setPower(-1.0);
        auto.resetDepositBox();
        if(mode == 1){
            robot.intakeControl.setPosition(intakeHeight1);
        }
        if(mode == 2){
            robot.intakeControl.setPosition(intakeHeight2);
        }
        if(mode == 3){
            robot.intakeControl.setPosition(intakeHeight3);
        }
        sleep(550);

        robot.intake.setPower(-1.0);
        sleep(1500);

    }

    public void intakeOff() {
        robot.intakeControl.setPosition(resetIntakeHeight);
        robot.wheel.setPower(0.0);
        robot.intake.setPower(0.0);
    }

    public void wheelSpitPixel(int milliseconds) {
        robot.wheel.setPower(1.0);
        sleep(milliseconds);
        robot.wheel.setPower(0.0);
    }

}