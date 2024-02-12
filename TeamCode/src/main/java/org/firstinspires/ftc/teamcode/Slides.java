package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Encoder.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Slides {
    // slide constants
    public static int SLIDES_MIN_HEIGHT = 0; // measured in counts
    public static int SLIDES_MAX_HEIGHT = 3000; // to be determined via experimentation

    // pid constants
    public static volatile double Kp = 0.03;
    public static volatile double Ki = 0;
    public static volatile double Kd = 0.0001;

    // idk
    private int targetPosition;
    private ElapsedTime timer = new ElapsedTime();

    // pid temp vars
    private double integralSum;
    private double lastError;
    private double error;

    // instance vars
    private Telemetry telemetry;
    private DcMotor slideLeft;
    private DcMotor slideRight;
    private Encoder slidesEncoder;

    public Slides(Telemetry telemetry, DcMotor slideLeft, DcMotor slideRight, Encoder slidesEncoder) {
        this.telemetry     = telemetry;
        this.slideLeft     = slideLeft;
        this.slideRight    = slideRight;
        this.slidesEncoder = slidesEncoder;
        this.slidesEncoder.setDirection(REVERSE);

        _resetTempVars();
    }

    public double updateSlides() {
        /**
         * NOTE: You must run this function each loop iteration. It will move the slides to
         * wherever they should be with PID.
         */
        if (Math.abs(error) >= 10) {
            telemetry.addData("reference", targetPosition);
            telemetry.addData("actual pos", slidesEncoder.getCurrentPosition());

            // obtain the encoder position
            this.slidesEncoder.getCurrentPosition();
            double encoderPosition = slidesEncoder.getCurrentPosition();
            // calculate the error
            error = targetPosition - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            // calculate the power, limit it between 0 and 1
            double outUnclamped = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("slide power", out);

            lastError = error;

            // reset the timer for next time
            timer.reset();
            return out;
        }
        else {
            _resetTempVars();
            return 0;
        }
    }


    public void updateSlidesAuto() {
        /**
         * NOTE: You must run this function each loop iteration. It will move the slides to
         * wherever they should be with PID.
         */
        error = 15;
        while (Math.abs(error) < 12) {
            telemetry.addData("reference", targetPosition);
            telemetry.addData("actual pos", slidesEncoder.getCurrentPosition());

            // obtain the encoder position
            this.slidesEncoder.getCurrentPosition();
            double encoderPosition = slidesEncoder.getCurrentPosition();
            // calculate the error
            error = targetPosition - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            // calculate the power, limit it between 0 and 1
            double outUnclamped = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("slide power", out);

            lastError = error;

            // reset the timer for next time
            timer.reset();
        }
        _resetTempVars();
    }

    public void move(double moveAmt) {
        /**
         * Increase/decrease the target position of the slides by some amount of counts. This is a
         * RELATIVE move.
         * @param moveAmt - The amount of clicks to increase/decrease the position by.
         */
        double newTargetPos = Math.min(Math.max(this.targetPosition + moveAmt, SLIDES_MIN_HEIGHT), SLIDES_MAX_HEIGHT);
        this.setTargetPosition((int) Math.round(newTargetPos));
    }

    public void moveToBottom() {
        /**
         * Move the slides down to the minimum height.
         */
        setTargetPosition(SLIDES_MIN_HEIGHT);
    }

    public int getTargetPosition() {
        /**
         * Gets the current target position of the slides.
         * @return targetPosition - The target position of the slides.
         */
        return this.targetPosition;
    }

    public void setTargetPosition(int targetPosition) {
        /**
         * Sets the **absolute** position of the slides.
         * @param targetPosition - The new target position for the slides.
         */
        this.targetPosition = targetPosition;
    }

    private void _resetTempVars() {
        this.integralSum = 0;
        this.lastError = 0;
        this.error = 100; // random number
    }

    public void setSlidePower(double power) {
        this.slideLeft.setPower(-power);
        this.slideRight.setPower(power);
    }

}
