package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides {
    public static volatile double Kp = 0;
    public static volatile double Ki = 0;
    public static volatile double Kd = 0;

    private double targetPosition;

    private MultipleTelemetry telemetry;
    private DcMotor slideLeft;
    private DcMotor slideRight;

    public Slides(MultipleTelemetry telemetry, DcMotor slideLeft, DcMotor slideRight) {
        this.telemetry  = telemetry;
        this.slideLeft  = slideLeft;
        this.slideRight = slideRight;

    }

    public void updateSlides() {

    }

    public void move(double targetPosition) {
        this.targetPosition = targetPosition;
    }

}
