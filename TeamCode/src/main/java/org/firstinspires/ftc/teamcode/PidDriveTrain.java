package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class PidDriveTrain {
    // region pid constants - x
    public static volatile double KpX = 0.035;
    public static volatile double KiX = 0.001;
    public static volatile double KdX = 0.00255;
    // endregion

    // region pid constants - y
    public static volatile double KpY = -0.15;
    public static volatile double KiY = -0.0005;
    public static volatile double KdY = -0.01;
    // endregion

    // region pid constants - heading
    public static volatile double KpH = 2.05;
    public static volatile double KiH = 0;
    public static volatile double KdH = 0.1;
    // endregion

    // region target positions
    private double targetPositionX;
    private double targetPositionY;
    private double targetPositionH;
    // endregion

    // region timers
    private ElapsedTime timerX = new ElapsedTime();
    private ElapsedTime timerY = new ElapsedTime();
    private ElapsedTime timerH = new ElapsedTime();
    // endregion

    // PID Variables - x
    private double integralSumX;
    private double lastErrorX;
    public double errorX;

    // PID Variables - y
    private double integralSumY;
    private double lastErrorY;
    public double errorY;

    // PID Variables - heading
    private double integralSumH;
    private double lastErrorH;
    public double errorH;

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
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _resetTempXVars();
        _resetTempYVars();
        _resetTempHVars();
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
//        boolean xReached = false;
//        boolean yReached = false;
//        boolean hReached = false;
        // calculate x power needed
        if (Math.abs(errorX) >= maxErrorX) {
            // calculate the error
            // error = targetPosition - encoderPosition; commented cause i call calculateError();

            // rate of change of the error
            double derivativeX = (errorX - lastErrorX) / timerX.seconds();

            // sum of all error over time
            integralSumX = integralSumX + (errorX * timerX.seconds());

            double outUnclamped = (KpX * errorX) + (KiX * integralSumX) + (KdX * derivativeX);
            double out = outUnclamped;
//            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("x powerr", out);

            lastErrorX = errorX;

            // reset the timer for next time
            timerX.reset();
            xPower = out;
        }
        else {
//            xReached = true;
            reachedX = true;
            _resetTempXVars();
        }

        // calculate y power needed
        if (Math.abs(errorY) >= maxErrorY) {
            // calculate the error
            // error = targetPosition - encoderPosition; commented cause i call calculateError();

            // rate of change of the error
            double derivativeY = (errorY - lastErrorY) / timerY.seconds();

            // sum of all error over time
            integralSumY = integralSumY + (errorY * timerY.seconds());

            double outUnclamped = (KpY * errorY) + (KiY * integralSumY) + (KdY * derivativeY);
            double out = outUnclamped;
//            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("y power", out);

            lastErrorY = errorY;

            // reset the timer for next time
            timerY.reset();
            yPower = out;
        }
        else {
//            yReached = true;
            reachedY = true;
            _resetTempYVars();
        }

        // calculate heading power needed
        if (Math.abs(errorH) >= maxErrorH) {
            // calculate the error
            // error = targetPosition - encoderPosition; commented cause i call calculateError();

            // rate of change of the error
            double derivativeH = (errorH - lastErrorH) / timerH.seconds();

            // sum of all error over time
            integralSumH = integralSumH + (errorH * timerH.seconds());

            double outUnclamped = (KpH * errorH) + (KiH * integralSumH) + (KdH * derivativeH);
            double out = outUnclamped;
//            double out = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1
            telemetry.addData("h power", out);

            lastErrorH = errorH;

            // reset the timer for next time
            timerH.reset();
            hPower = out;
        }
        else {
//            hReached = true;
            reachedH = true;
            _resetTempHVars();
        }
//        reachedX = xReached;
//        reachedY = yReached;
//        reachedH = hReached;

//        if(reachedX && reachedY && reachedH){
        // we reached the position we wanted to get to
        // not sure if we should do this if all positions are reached
        // or when each individual position is reached
        // may need to change
        // _resetTempVars();
//        }

        x = xPower;
        y = yPower;
        h = hPower;
    }

    public double powerX(){
        return x;
    }
    public double powerY() {
        return y;
    }
    public double powerH(){
        return h;
    }

    public boolean reached(){
        return reachedX && reachedY && reachedH;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    public void updatePos() {
        drive.update();
//        drive.updatePoseEstimate();
        Pose2d poseEstimate = drive.getPoseEstimate();
        curX = -poseEstimate.getX();
        curY = -poseEstimate.getY();
        curH = poseEstimate.getHeading();

        curH = angleWrap(curH);

    }

    public double getCurrentHeading(){
        // this should only be called after we've updated powers
        // so like its fine cause curH won't be sus
        return curH;
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
        updatePower();
    }

    public void calculateError() {
        updatePos(); // update position on field
        telemetry.addData("curX", curX);
        telemetry.addData("curY", curY);
        telemetry.addData("curHeading", curH);
        errorX = targetPositionX - curX;
        errorY = targetPositionY - curY;

        // For error h, there are 2 ways: either we can turn clockwise or counter clockwise
        // Always just take the fastest way cause that's best!
        // if its less than -180 just increase by 360 deg
        errorH = angleWrap(targetPositionH - curH);

    }

//    private void _resetTempVars() {
//        // PID Variables - x
//        this.integralSumX = 0;
//        this.lastErrorX = 0;
//        this.errorX = 0;
//        // PID Variables - y
//        this.integralSumY = 0;
//        this.lastErrorY = 0;
//        this.errorY = 0;
//        // PID Variables - heading
//        this.integralSumH = 0;
//        this.lastErrorH = 0;
//        this.errorH = 0;
//
//        this.maxErrorX = 0;
//        this.maxErrorY = 0;
//        this.maxErrorH = 0;
//
//        // reached the positions??
//        this.reachedX = false;
//        this.reachedY = false;
//        this.reachedH = false;
//
//        // power required
//        this.x = 0.0;
//        this.y = 0.0;
//        this.h = 0.0;
//        // NOTE:
//        // before, in slides we set error to random value but that's sus
//        // I hoped to fix this with calculate error function.
//        // If that doesn't work we can do this
//    }

    private void _resetTempXVars() {
        // PID Variables - x
        this.integralSumX = 0;
        this.lastErrorX = 0;
        this.errorX = 0;

        // reached the positions??
//        this.reachedX = true;

        // power required
        this.x = 0.0;
    }

    private void _resetTempYVars() {
        // PID Variables - x
        this.integralSumY = 0;
        this.lastErrorY = 0;
        this.errorY = 0;

        // reached the positions??
//        this.reachedY = true;

        // power required
        this.y = 0.0;
    }

    private void _resetTempHVars() {
        // PID Variables - x
        this.integralSumH = 0;
        this.lastErrorH = 0;
        this.errorH = 0;

        // reached the positions??
//        this.reachedH = true;

        // power required
        this.h = 0.0;
    }

}
