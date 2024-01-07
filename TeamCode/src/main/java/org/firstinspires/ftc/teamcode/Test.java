package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="Testing")
public class Test extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();
//    MultipleTelemetry telemetry;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.init(hardwareMap);
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorBackRight"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFrontRight"));
        waitForStart();

        while (opModeIsActive()) {
            // Drive Train (REUSED CODE), except we cube the motor power to reduce it
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            double robotSpeed;
            if (gamepad1.x) { robotSpeed = 1.0; }
            else            { robotSpeed = 0.5; }

            robot.motorFrontLeft.setPower(-v3*robotSpeed); // some of these might need to be negative
            robot.motorFrontRight.setPower(v4*robotSpeed);
            robot.motorBackLeft.setPower(v1*robotSpeed);
            robot.motorBackRight.setPower(-v2*robotSpeed);


            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
            telemetry.addData("Back Encoder", frontEncoder.getCurrentPosition());
            telemetry.update();
        }

    }
}
