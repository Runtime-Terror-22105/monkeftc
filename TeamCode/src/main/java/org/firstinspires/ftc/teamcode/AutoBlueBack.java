package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CenterStageAutonomous.coord;
import org.firstinspires.ftc.teamcode.util.TwoPositions;

@Autonomous(name="2+5 Blue Back Auto", group="concept")
public class AutoBlueBack extends LinearOpMode {
    private FtcDashboard dashboard;
    private CenterStageAutonomous auto;
    HardwarePushbot robot = new HardwarePushbot();
    public static volatile TwoPositions depositLeftPositions = new TwoPositions(0.79, 0.0);
    public static volatile TwoPositions depositRightPositions = new TwoPositions(0.21, 1.0);
    public static double resetIntakeHeight = 1.0;
    public static double intakeHeight1 = 0.795;
    public static double intakeHeight2 = 0.75;
    public static double intakeHeight3 = 0.72;

    public coord[] points = new coord[100];
    public coord leftDepo = new coord(-80, 12.045, 4.71239, 2, 1.25, Math.toRadians(3));
    public coord rightDepo = new coord(-80, 25.938, 4.71239, 2, 1.25, Math.toRadians(3));
    public coord centerDepo = new coord(-80, 19.381, 4.71239, 2, 1.25, Math.toRadians(3));

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
                (milliseconds) -> sleep(milliseconds)
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

        auto.initCamera(CenterStageAutonomous.WhatColorToDetect.RED);
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
                points[0] = new coord(-6.4503, 28.3385466, 5.465502567, 2, 2, Math.toRadians(3)); // place on purple pixel mark
                points[1] = new coord(-0.28603, 19.313, 0, 2, 2, Math.toRadians(5)); // move back a little to reset
                break;
            case MIDDLE:
                points[0] = new coord(-0.17, 29, 0, 2, 2, Math.toRadians(3)); // place on purple pixel mark
                points[1] = new coord(-0.28603, 19.313, 0, 2, 2, Math.toRadians(5)); // move back a little to reset
                break;
            case RIGHT:
                points[0] = new coord(10.3336, 25.514, 0, 2, 2, Math.toRadians(3)); // place on purple pixel mark
                points[1] = new coord(-0.28603, 19.313, 0, 2, 2, Math.toRadians(5)); // move back a little to reset
                points[2] = new coord(-0.2177, 50.84, 0, 2, 2, Math.toRadians(3)); // move back a little to reset
                code[2] = 0;
                c = 1;
                break;
        }

        code[0] = 0;
        c++;
        code[1] = 0;
        c++;

        // Move forward to stack!
        points[c] = new coord(12, 50.68, 4.71239, 2, 2, Math.toRadians(3));
        code[c] = 10; // Start intake as we have reached the stack
        c++;
        points[c] = new coord(19.5397, 50.68, 4.71239, 2, 2, Math.toRadians(3));
        code[c] = 1; // Start intake as we have reached the stack
        c++;

        // Now, code cycles!
        // Number of i loops = number of cycles
        // i = 0, 2+1, i = 1, 2+3, i = 2, 2+5!!!
        for (int i = 0; i <= 2; i++) {
            // First do the path  to get to the backdrop.
            // First go to under the stage door (USE HIGH ERROR)
            points[c] = new coord(-59.5049, 43.588, 4.71239, 2, 2, Math.toRadians(3));
            code[c] = 5; // Slides up and down
            c++;
            // Then, you crossed the stage door, so change the desired position.
            // Depending on positions, change deposit.
            if(i == 0){ // first location, so need to be careful
                switch (location) {
                    case LEFT:
                        points[c] = rightDepo;
                        code[c] = 6;
                        c++;
                        points[c] = leftDepo;
                        code[c] = 11;
                        c++;
                        break;
                    case MIDDLE:
                        points[c] = rightDepo;
                        code[c] = 6;
                        c++;
                        points[c] = centerDepo;
                        code[c] = 11;
                        c++;
                        break;
                    case RIGHT:
                        points[c] = centerDepo;
                        code[c] = 6;
                        c++;
                        points[c] = rightDepo;
                        code[c] = 11;
                        c++;
                        break;
                }
            }
            else {
                // just do right depot as fastest
                points[c] = rightDepo;
                code[c] = 7;
                c++;
            }

            if(i == 2){
                break;
            }

            // Now we have placed pixels. We must go back to stack.
            // First get to parallel, use large error for no slow down

            points[c] = new coord(-40.4, 64.002, 4.71239, 2, 2, Math.toRadians(5));
            code[c] = 10; // Nothing to change
            c++;

            // Now turn on intake and go to stack!

            points[c] = new coord(12, 50.68, 4.71239, 2, 2, Math.toRadians(3));
            code[c] = 10; // Start intake as we have reached the stack
            c++;
            points[c] = new coord(19.5397, 50.68, 4.71239, 2, 2, Math.toRadians(3));
            code[c] = i + 1; // Start intake as we have reached the stack
            c++;
            // REPEAT
        }

        int numPoints = c;

        while (opModeIsActive()) { // continuous while loop for progra
            if (p != numPoints) {
                follower.setTargetPosition(
                        points[p].y,
                        points[p].x,
                        points[p].heading,
                        points[p].maxErrorX,
                        points[p].maxErrorY,
                        points[p].maxErrorH
                );
            }

            if (p != numPoints && follower.reached()) {
                robot.motorBackLeft  .setPower(0);
                robot.motorBackRight .setPower(0);
                robot.motorFrontLeft .setPower(0);
                robot.motorFrontRight.setPower(0);
                telemetry.addData("reached lol", "hi rahul");
                // reached position, do task
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
                switch(code[p]) { // will code this after
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
                        intakeOff();
                        slides.moveToLineOne();
                        break;
                    case 6:
                        setDepositBox();
                        sleep(700);
                        wheelSpitPixel(300);
                        sleep(1000);
                        break;
                    case 7:
                        setDepositBox();
                        sleep(700);
                        wheelSpitPixel(300);
                        sleep(500);
                        resetDepositBox();
                        sleep(1000);
                        slides.moveToBottom();
                        resetDepositBox();
                        break;
                    // Aadit pls code this later on to make it proper
                    // Later on we can just make it so it resets while raising I was too lazy rn to do it
                    // To optimize for time
                    case 8:
                        sleep(300);
                        break;

                    case 10:
                        robot.intakeControl.setPosition(resetIntakeHeight);
                        break;
                    case 11:
                        setDepositBox();
                        sleep(700);
                        wheelSpitPixel(300);
                        sleep(500);
                        resetDepositBox();
                        resetDepositBox();
                        sleep(800);
                        slides.moveToBottom();
                        resetDepositBox();
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

            double slidePower = slides.updateSlides();
            slides.setSlidePower(slidePower); // also update slides power, copied from teleop
            telemetry.update();
        }
    }

    public void resetDepositBox() {
        robot.depositLeft.setPosition(depositLeftPositions.normal);
        robot.depositRight.setPosition(depositRightPositions.normal);

    }

    public void setDepositBox() {
        robot.depositLeft.setPosition(depositLeftPositions.out);
        robot.depositRight.setPosition(depositRightPositions.out);
    }

    public void intakeOn(int mode) {
        robot.wheel.setPower(-1.0);
        resetDepositBox();
        if(mode == 1){
            robot.intakeControl.setPosition(intakeHeight1);
        }
        if(mode == 2){
            robot.intakeControl.setPosition(intakeHeight2);
        }
        if(mode == 3){
            robot.intakeControl.setPosition(intakeHeight3);
        }
        sleep(700);

        robot.intake.setPower(-1.0);
        sleep(2000);

    }
    public void intakeOff() {
        robot.intakeControl.setPosition(resetIntakeHeight);
        robot.intake.setPower(0.0);
    }
    public void wheelSpitPixel(int milliseconds) {
        robot.wheel.setPower(1.0);
        sleep(milliseconds);
        robot.wheel.setPower(0.0);
    }

}
