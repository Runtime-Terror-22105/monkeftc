package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Testing")
public class AutoTest extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();
    public final int RIGHTANGLETURNTIME = 200; // tweak this value with trial and error

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        turnLeft(0.5);
    }

    public void moveForward(int milliseconds, double power) {
        /**
         * Move forward for some time.
         * @param milliseconds - How long to move forward for, in milliseconds
         * @param power - The power to use for the motors
         */
        FrontDrive(power);
        sleep(milliseconds);
        CancelPowerRobot();
    }

    public void turnLeft(double power) {
        spin(power);
        long sleeptime = (long)(RIGHTANGLETURNTIME/power); // tweak this value with trial and error
        sleep(sleeptime);
        CancelPowerRobot();
    }

    public void turnRight(double power) {
        spin(power);
        long sleeptime = (long)(RIGHTANGLETURNTIME/power);
        sleep(sleeptime);
        CancelPowerRobot();
    }

    public void intake(int milliseconds, double power) {
        /**
         * @param milliseconds - How long to intake for, in milliseconds
         * @param power - The power to use for the motors
         */
        robot.intake.setPower(power);
        sleep(milliseconds);
        robot.intake.setPower(0);
    }

    public void outtake(int milliseconds, double power) {
        /**
         * @param milliseconds - How long to outtake for, in milliseconds
         * @param power - The power to use for the servos
         */
        robot.intake.setPower(power);
        sleep(milliseconds);
        robot.intake.setPower(0);
    }

    public void spin(double power) {
        power=-power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
    }

    public void FrontDrive(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
    }

    public void LeftSlantDrive(double power) {
        power=-power;
        robot.motorFrontLeft.setPower(0.0);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(0.0);
        robot.motorBackLeft.setPower(power);
    }

    public void RightSlantDrive(double power) {
        power = -power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(0.0);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(0.0);
    }

    public void RightSlantRearDrive(double power) {
        robot.motorFrontLeft.setPower(0.0);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(0.0);
        robot.motorBackLeft.setPower(power);
    }

    public void LeftSlantRearDrive(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(0.0);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(0.0);
    }

    public void RearDrive(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }

    public void SlideRight(double power) {
        power = -power;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(-power);

    }

    public void SlideLeft(double power) {
        power = -power;
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);

    }

    public void CancelPowerRobot() {
        double power = 0.0;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }

    public void robotsleep(long time) {
        sleep(time);
    }
}
