// basics of opencv: https://docs.opencv.org/3.4/d6/d6d/tutorial_mat_the_basic_image_container.html
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ObjectPositionPipeline extends OpenCvPipeline {
    // public static volatile double DETECTION_THRESHOLD = 0.5;
    // ROI = region of interest, aka the rectangle we're drawing
    // you should fine tune this
    public static volatile Rect ROI_Left   = new Rect(new Point( 0, 0), new Point(107, 240));
    public static volatile Rect ROI_Middle = new Rect(new Point(110, 0), new Point(213, 240));
    public static volatile Rect ROI_Right  = new Rect(new Point(220, 0), new Point(320, 240));

    // The two colors we're using
    public static volatile Scalar redColor  = new Scalar(  0,   0, 255);
    public static volatile Scalar blueColor = new Scalar(  0, 255,   0);

    public boolean DETECT_RED = true;
//    public double leftAvg;
//    public double middleAvg;
//    public double rightAvg;
    private Telemetry telemetry;

    public static double MIN_VALUES = 100;
    public static double MAX_VALUES = 255;
    public static double MIN_SATURATION = 100;
    public static double MAX_SATURATION = 255;
    public static double MIN_BLUE_HUE = 180;
    public static double MAX_BLUE_HUE = 240;
    public static double MIN_RED_LOW_HUE = 0;
    public static double MAX_RED_LOW_HUE = 30;
    public static double MIN_RED_HIGH_HUE = 336;
    public static double MAX_RED_HIGH_HUE = 360;
    // opencv uses hue from 0-180 instead of 0-360 for some reason so I divide by 2
    static Scalar MIN_BLUE     = new Scalar(MIN_BLUE_HUE/2,     MIN_SATURATION, MIN_VALUES);
    static Scalar MAX_BLUE     = new Scalar(MAX_BLUE_HUE/2,     MAX_SATURATION, MAX_VALUES);
    static Scalar MIN_RED_LOW  = new Scalar(MIN_RED_LOW_HUE/2,  MIN_SATURATION, MIN_VALUES);
    static Scalar MAX_RED_LOW  = new Scalar(MAX_RED_LOW_HUE/2,  MAX_SATURATION, MAX_VALUES);
    static Scalar MIN_RED_HIGH = new Scalar(MIN_RED_HIGH_HUE/2, MIN_SATURATION, MIN_VALUES);
    static Scalar MAX_RED_HIGH = new Scalar(MAX_RED_HIGH_HUE/2, MAX_SATURATION, MAX_VALUES);


    public static enum Location {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location propLocation = Location.MIDDLE;


    public ObjectPositionPipeline(Telemetry t) {
        this.telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        // convert rgb image to hsv (hue, saturation, value) image
        // why hsv? hsv makes it easy to differentiate colors in different lighting conditions.
        //          we basically ignore saturation since it is the "intensity"
        //          we check for what the hue is since it is the "type" of color
        //          we check for value since it is "brightness", we don't want to accidentally detect black or white or something else
        Mat mat = new Mat(); // our working copy of the image, mat = matrix
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        // If something goes wrong, return
        if (mat.empty()) {
            this.telemetry.addData("processFrame func", "something wnet wrong");
            this.telemetry.update();
            return input;
        }

//        // extract either the red or blue color channel
//        Mat oneColor = new Mat();
//        Scalar propColor;
//        if (this.DETECT_RED) {
//            propColor = redColor;
//            Core.extractChannel(mat, oneColor, 2);
//        }
//        else {
//            propColor = blueColor;
//            Core.extractChannel(mat, oneColor, 1);
//        }
//
//        // split the matrix into submatrices
//        Mat leftMat = oneColor.submat(ROI_Left);
//        Mat middleMat = oneColor.submat(ROI_Middle);
//        Mat rightMat = oneColor.submat(ROI_Right);
//
//        Scalar leftAvgScalar = Core.mean(leftMat);
//        Scalar middleAvgScalar = Core.mean(middleMat);
//        Scalar rightAvgScalar = Core.mean(rightMat);
//
//        leftAvg = leftAvgScalar.val[0];
//        middleAvg = middleAvgScalar.val[0];
//        rightAvg = rightAvgScalar.val[0];
//
////        // camera can't see right when robot is on the blue side and camera can't see left on red side
////        if (leftAvg <= DETECTION_THRESHOLD && middleAvg <= DETECTION_THRESHOLD && rightAvg <= DETECTION_THRESHOLD) {
////            if (DETECT_RED) { leftAvg  = 100000; }
////            else            { rightAvg = 100000; }
////            this.telemetry.addData("Prop did not meet detection threshold", "cri");
////        }
//
//        this.telemetry.addData("leftAvg", leftAvg);
//        this.telemetry.addData("middleAvg", middleAvg);
//        this.telemetry.addData("rightAvg", rightAvg);
//        if (leftAvg >= rightAvg && leftAvg >= middleAvg) {
//            propLocation = Location.LEFT;
//            Imgproc.rectangle(mat, ROI_Left, propColor);
//            this.telemetry.addData("Prop location:", "left");
//        }
//        else if (rightAvg >= middleAvg) {
//            propLocation = Location.RIGHT;
//            Imgproc.rectangle(mat, ROI_Right, propColor);
//            this.telemetry.addData("Prop location:", "right");
//        }
//        else {
//            propLocation = Location.MIDDLE;
//            Imgproc.rectangle(mat, ROI_Middle, propColor);
//            this.telemetry.addData("Prop location:", "middle");
//        }
//        this.telemetry.update();
//

        if (DETECT_RED) {
            // check if red one is there, check both high and low range in spectrum
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MIN_RED_LOW, MAX_RED_LOW, mat1);
            Core.inRange(mat2, MIN_RED_HIGH, MAX_RED_HIGH, mat2);
            Core.bitwise_or(mat1, mat2, mat);
        }
        else {
            // check if blue one is there
            Core.inRange(mat, MIN_BLUE, MAX_BLUE, mat);
        }

        // make submatrices for each ROI
        Mat left = mat.submat(ROI_Left);
        Mat middle = mat.submat(ROI_Middle);
        Mat right = mat.submat(ROI_Right);

        // Find which area has the most stuff captured by the mask
        double leftValue = Core.sumElems(left).val[0];
        double middleValue = Core.sumElems(middle).val[0];
        double rightValue = Core.sumElems(right).val[0];

        telemetry.addData("Left raw value:", leftValue);
        telemetry.addData("Middle raw value:", middleValue);
        telemetry.addData("Right raw value:", rightValue);

        // free memory used by the submatrixes, no point in having these big matrices doing nothing slowing down the code
        left.release();
        middle.release();
        right.release();

        if (leftValue >= rightValue && leftValue >= middleValue) {
            propLocation = Location.LEFT;
            telemetry.addData("Prop location:", "left");
        }
        else if (rightValue >= middleValue) {
            propLocation = Location.RIGHT;
            telemetry.addData("Prop location:", "right");
        }
        else {
            propLocation = Location.MIDDLE;
            telemetry.addData("Prop location:", "middle");
        }
        telemetry.update();


        /* Draw rectangles to visualize prop location. */
        // grayscale to rgb
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255, 255, 255);
        Scalar propColor = new Scalar(  0,   0, 255);

        Imgproc.rectangle(mat, ROI_Left, propLocation == Location.LEFT ? pixelColor:propColor);
        Imgproc.rectangle(mat, ROI_Middle, propLocation == Location.MIDDLE ? pixelColor:propColor);
        Imgproc.rectangle(mat, ROI_Right, propLocation == Location.RIGHT ? pixelColor:propColor);


        return mat;
    }

    public Location getPropLocation() {
        return propLocation;
    }

    public void setDetectRed(boolean shouldDetectRed) {
        this.DETECT_RED = shouldDetectRed;
    }
}