package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Front Auto", group="Red Auto")
public class PIDControls extends LinearOpMode {
    HardwarePushbot robot= new HardwarePushbot();
    double kp=0;
    double Ki=0;
    double kd=0;
    double IntegralSum=0;
    double lastError=0;

    @Override
    public void runOpMode() {

    }


}