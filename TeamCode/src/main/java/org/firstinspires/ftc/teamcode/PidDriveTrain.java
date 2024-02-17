package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class PidDriveTrain {
    // pid constants - x
    public static volatile double KpX = 0;
    public static volatile double KiX = 0;
    public static volatile double KdX = 0;
    // pid constants - y
    public static volatile double KpY = 0;
    public static volatile double KiY = 0;
    public static volatile double KdY = 0;
    // pid constants - heading
    public static volatile double KpH = 0;
    public static volatile double KiH = 0;
    public static volatile double KdH = 0;

    // target positions
    private double targetPositionX;
    private double targetPositionY;
    private double targetPositionH;

    private ElapsedTime timerX = new ElapsedTime();
    private ElapsedTime timerY = new ElapsedTime();
    private ElapsedTime timerH = new ElapsedTime();

    // PID Variables - x
    private double integralSumX;
    private double lastErrorX;
    private double errorX;
    // PID Variables - y
    private double integralSumY;
    private double lastErrorY;
    private double errorY;
    // PID Variables - heading
    private double integralSumH;
    private double lastErrorH;
    private double errorH;

    // instance vars
    private Telemetry telemetry;
    private SampleMecanumDrive drive;

    // error required. depending on how accurate we want each point to be
    private double maxErrorX;
    private double maxErrorY;
    private double maxErrorH;


    // reached the positions??
    public boolean reachedX;
    public boolean reachedY;
    public boolean reachedH;

    // power required
    public double x;
    public double y;
    public double h;

    // current position on field
    public double curX;
    public double curY;
    public double curH;


    public PidDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.drive = new SampleMecanumDrive(hardwareMap);

        _resetTempVars();
    }

    public void updatePower() {
        /**
         * NOTE: You must run this function each loop iteration. It will move the slides to
         * wherever they should be with PID.
         */
        calculateError();
        double xPower = 0.0;
        double yPower = 0.0;
        double hPower = 0.0;
        boolean xReached = false;
        boolean yReached = false;
        boolean hReached = false;
        // calculate x power needed
        if (errorX >= maxErrorX) {
            // calculate the error
            // error = targetPosition - encoderPosition; commented cause i call calculateError();

            // rate of change of the error
            double derivativeX = (errorX - lastErrorX) / timerX.seconds();

            // sum of all error over time
            integralSumX = integralSumX + (errorX * timerX.seconds());

            double outUnclamped = (KpX * errorX) + (KiX * integralSumX) + (KdX * derivativeX);
            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("x powerr", out);

            lastErrorX = errorX;

            // reset the timer for next time
            timerX.reset();
            xPower = out;
        }
        else {
            xReached = true;
        }

        // calculate y power needed
        if (errorY >= maxErrorY) {
            // calculate the error
            // error = targetPosition - encoderPosition; commented cause i call calculateError();

            // rate of change of the error
            double derivativeY = (errorY - lastErrorY) / timerY.seconds();

            // sum of all error over time
            integralSumY = integralSumY + (errorY * timerY.seconds());

            double outUnclamped = (KpY * errorY) + (KiY * integralSumY) + (KdY * derivativeY);
            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("y power", out);

            lastErrorY = errorY;

            // reset the timer for next time
            timerY.reset();
            yPower = out;
        }
        else {
            yReached = true;
        }

        // calculate heading power needed
        if (errorH >= maxErrorH) {
            // calculate the error
            // error = targetPosition - encoderPosition; commented cause i call calculateError();

            // rate of change of the error
            double derivativeH = (errorH - lastErrorH) / timerH.seconds();

            // sum of all error over time
            integralSumH = integralSumH + (errorH * timerH.seconds());

            double outUnclamped = (KpH * errorH) + (KiH * integralSumH) + (KdH * derivativeH);
            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("h power", out);

            lastErrorH = errorH;

            // reset the timer for next time
            timerH.reset();
            hPower = out;
        }
        else {
            hReached = true;
        }
        reachedX = xReached;
        reachedY = yReached;
        reachedH = hReached;

        if(reachedX && reachedY && reachedH){
            // we reached the position we wanted to get to
            // not sure if we should do this if all positions are reached
            // or when each individual position is reached
            // may need to change
            _resetTempVars();
        }

        x = xPower;
        y = yPower;
        h = hPower;
    }

    public double powerX(){
        return x;
    }
    public double powerY(){
        return y;
    }
    public double powerH(){
        return h;
    }

    public boolean reached(){
        return reachedX && reachedY && reachedH;
    }

    public void updatePos(){
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        curX = poseEstimate.getX();
        curY = poseEstimate.getY();
        curH = poseEstimate.getHeading();
    }


    public void setTargetPosition(double x, double y, double heading, double xError, double yError, double hError) {
        /**
         * Sets the **absolute** position of the slides.
         * @param targetPosition - The new target position for the slides.
         */
        this.targetPositionX = x;
        this.targetPositionY = y;
        this.targetPositionH = heading;
        this.maxErrorX = xError; // max follower radius basically, sometimes we don't need to be too accuarate
        this.maxErrorY = yError; // i.e placing purple
        this.maxErrorH = hError; // but some more accurate like for yellow
        calculateError();

    }
    public void calculateError(){
        updatePos(); // update position on field
        errorX = Math.abs(targetPositionX - curX);
        errorY = Math.abs(targetPositionY - curY);
        errorH = Math.abs(targetPositionH - curH);
    }
    private void _resetTempVars() {
        // PID Variables - x
        this.integralSumX = 0;
        this.lastErrorX = 0;
        this.errorX = 0;
        // PID Variables - y
        this.integralSumY = 0;
        this.lastErrorY = 0;
        this.errorY = 0;
        // PID Variables - heading
        this.integralSumH = 0;
        this.lastErrorH = 0;
        this.errorH = 0;

        this.maxErrorX = 0;
        this.maxErrorY = 0;
        this.maxErrorH = 0;

        // reached the positions??
        this.reachedX = false;
        this.reachedY = false;
        this.reachedH = false;

        // power required
        this.x = 0.0;
        this.y = 0.0;
        this.h = 0.0;
        // NOTE:
        // before, in slides we set error to random value but that's sus
        // I hoped to fix this with calculate error function.
        // If that doesn't work we can do this
    }

}