package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CenterStageAutonomous.coord;
import org.firstinspires.ftc.teamcode.util.TwoPositions;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="2+5 Blue Back Auto", group="concept")
public class AutoBlueBack extends LinearOpMode {
    private FtcDashboard dashboard;
    private CenterStageAutonomous auto;
    HardwarePushbot robot = new HardwarePushbot();
    public static volatile TwoPositions depositLeftPositions = new TwoPositions(0.79, 0.0);
    public static volatile TwoPositions depositRightPositions = new TwoPositions(0.21, 1.0);
    public static double resetIntakeHeight = 1.0;
    public static double intakeHeight1 = 0.80;
    public static double intakeHeight2 = 0.73;
    public static double intakeHeight3 = 0.71;

    public static int depositOnePixel = 590;

    public coord[] points = new coord[100];
    public coord leftDepo = new coord(-82.07697, 15.01, 4.71239, 2, 0.8, Math.toRadians(3));
    public coord rightDepo = new coord(-82.07697, 26.702, 4.71239, 2, 0.8, Math.toRadians(3));
    public coord centerDepo = new coord(-82.07697, 20.9381, 4.71239, 2, 0.8, Math.toRadians(3));

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
        ElapsedTime depositBoxTimer = new ElapsedTime();

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
                points[0] = new coord(-0.17, 26, 0, 2, 2, Math.toRadians(3)); // place on purple pixel mark
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
        code[c] = 0; // Nothing new
        c++;
        points[c] = new coord(19.2397, 50.68, 4.71239, 2, 0.8, Math.toRadians(3));
        code[c] = 1; // Start intake as we have reached the stack
        c++;

        // Now, code cycles!
        // Number of i loops = number of cycles
        // i = 0, 2+1, i = 1, 2+3, i = 2, 2+5!!!
        for (int i = 0; i <= 3; i++) {
            // First do the path  to get to the backdrop.
            // First go to under the stage door (USE HIGH ERROR)
            points[c] = new coord(-59.5049, 48, 4.71239, 10, 3, Math.toRadians(3));
            code[c] = 4; // Slides up and down
            c++;
            // Then, you crossed the stage door, so change the desired position.
            // Depending on positions, change deposit.
            if(i == 0){ // first location, so need to be careful
                switch (location) {
                    case LEFT:
                        points[c] = rightDepo;
                        code[c] = 5;
                        c++;
                        points[c] = leftDepo;
                        code[c] = 6;
                        c++;
                        break;
                    case MIDDLE:
                        points[c] = rightDepo;
                        code[c] = 5;
                        c++;
                        points[c] = centerDepo;
                        code[c] = 6;
                        c++;
                        break;
                    case RIGHT:
                        points[c] = centerDepo;
                        code[c] = 5;
                        c++;
                        points[c] = rightDepo;
                        code[c] = 6;
                        c++;
                        break;
                }
            }
            else {
                // just do center depot cause it requires least accuracy
                points[c] = centerDepo;
                code[c] = 7;
                c++;
            }

            if(i == 3){
                break;
            }

            // Now we have placed pixels. We must go back to stack.
            // First get to parallel, use large error for no slow down

            points[c] = new coord(-40.4, 69.002, 4.71239, 6, 6, Math.toRadians(6));
            code[c] = -1; // Nothing to change
            c++;

            // Now turn on intake and go to stack!
            if(i != 2) {
                points[c] = new coord(12, 50.68, 4.71239, 2, 2, Math.toRadians(3));
                code[c] = 0;
                c++;
                points[c] = new coord(19.2397, 50.68, 4.71239, 2, 0.8, Math.toRadians(3));
                code[c] = i + 1; // Start intake as we have reached the stack
                c++;
            }
            else{
                points[c] = new coord(19.7397, 50.68, 4.71239, 2, 2, Math.toRadians(3));
                code[c] = 0;
                c++;
                points[c] = new coord(19.2397, 38.61, 4.71239, 1, 0.8, Math.toRadians(3));
                code[c] = 1; // Start intake as we have reached the stack
                c++;
                points[c] = new coord(19.7397, 50.68, 4.71239, 2, 2, Math.toRadians(3));
                code[c] = 1; // Start intake as we have reached the stack
                c++;
            }
            // REPEAT
        }

        int numPoints = c;
        boolean goingDown = false;
        boolean startedGoingDown = false;
        int isSus = 2;
        boolean override = false;
        while (opModeIsActive()) { // continuous while loop for program
            double slidePower = slides.updateSlides();
            slides.setSlidePower(slidePower); // also update slides power, copied from teleop
            if(!override && isSus == 1 && slides.reached){
                goingDown = true;
            }

            if(isSus == 2 && !goingDown && robot.slidesEncoder.getCurrentPosition() >= 1000 && slides.getTargetPosition() != 0){
                setDepositBox();
            }
            if(goingDown && !startedGoingDown){
                depositBoxTimer.reset();
                resetDepositBox();
                startedGoingDown = true;
            }
            if(goingDown && startedGoingDown && depositBoxTimer.milliseconds() >= 575){
                slides.moveToBottom();
                isSus = 2;
                goingDown = false;
                startedGoingDown = false;
                override = true;
            }



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



            if (isSus == 2 && p != numPoints && follower.reached()) {
                // I did this to try and be fast
                // If note, we can just do it always
                if(!(code[c] == -1 || code[c] == 4) || !slides.reached) {
                    robot.motorBackLeft.setPower(0);
                    robot.motorBackRight.setPower(0);
                    robot.motorFrontLeft.setPower(0);
                    robot.motorFrontRight.setPower(0);
                    // sometimes, we don't want to slow down. This is in like cases where
                    //
                }

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
                        isSus = 1;
                        slides.moveToLineOne();
                        // Basically, we first have to go UP and then backdown..
                        // Very weird unfortunate consequence of bad planning of camera location
                        break;
                    case 7:
                        wheelSpitPixel(2 * depositOnePixel);
                        goingDown = true;
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
