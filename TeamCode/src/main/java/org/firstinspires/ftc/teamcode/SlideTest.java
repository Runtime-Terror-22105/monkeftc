package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="Slide Test")
public class SlideTest extends LinearOpMode {
    public static volatile double Kp = 0.03;
    public static volatile double Ki = 0;
    public static volatile double Kd = 0.0001;

    public static volatile double targetPosition = 0;

    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();
    HardwarePushbot robot = new HardwarePushbot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            double integralSum = 0;
            double lastError = 0;

            double error = 100;
            while (Math.abs(error) >= 10) {
                telemetry.addData("reference", targetPosition);
                telemetry.addData("actual pos", -robot.motorFrontLeft.getCurrentPosition());
                telemetry.update();

                // obtain the encoder position
                double encoderPosition = -robot.motorFrontLeft.getCurrentPosition();
                // calculate the error
                error = targetPosition - encoderPosition;

                // rate of change of the error
                double derivative = (error - lastError) / timer.seconds();

                // sum of all error over time
                integralSum = integralSum + (error * timer.seconds());

                double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                robot.slideLeft.setPower(-out);
                robot.slideRight.setPower(out);

                lastError = error;

                // reset the timer for next time
                timer.reset();

            }
        }
    }
}
