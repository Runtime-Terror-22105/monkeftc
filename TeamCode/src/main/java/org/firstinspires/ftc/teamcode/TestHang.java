package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Hang", group="Concept")
public class TestHang extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        FrontDrive(0.5);
        sleep(1800);
        CancelPowerRobot();
        //robot.intake.setPower(0.8);
        sleep(750);
        robot.intake.setPower(0.0);
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
        robot.motorBackLeft.setPower((312.0/435.0) * power);
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
        double power=0.0;
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }

    public void robotsleep(long time) {
        sleep(time);
    }
}

