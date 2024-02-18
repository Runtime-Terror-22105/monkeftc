package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Test2 extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Encoder value", robot.slidesEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
