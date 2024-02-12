package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test the turning")
@Config
public class MyTurnTest extends LinearOpMode {
    public static volatile int turn_time = 1000;
    public static volatile double power = 1.0;
    private FtcDashboard dashboard;
    private CenterStageAutonomous auto;
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();

        while (opModeIsActive()) {
            double predicted_turn_time = 1054*Math.pow(0.00440918, power) - 392.963*power + 615.264;
            telemetry.addData("Predicted turn time (ms)", predicted_turn_time);
            if (gamepad1.y) {
                telemetry.addData("Status", "turning...");
                telemetry.addData("Current turn time (ms)", turn_time);
                telemetry.update();

                auto.spinLeft(turn_time, power);
            } else {
                telemetry.addData("Status", "Press Y to test the turn");
                telemetry.addData("Current turn time (ms)", turn_time);
            }
            telemetry.update();
        }
    }
}
