// basics of opencv: https://docs.opencv.org/3.4/d6/d6d/tutorial_mat_the_basic_image_container.html
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ObjectPositionPipeline extends OpenCvPipeline {
    public static boolean DETECT_RED = true;
    public static double MIN_VALUES = 100;
    public static double MAX_VALUES = 255;
    public static double MIN_SATURATION = 100;
    public static double MAX_SATURATION = 255;
    public static double MIN_BLUE_HUE = 100;
    public static double MAX_BLUE_HUE = 115;
    public static double MIN_RED_LOW_HUE = 0;
    public static double MAX_RED_LOW_HUE = 25;
    public static double MIN_RED_HIGH_HUE = 160;
    public static double MAX_RED_HIGH_HUE = 255;

    Telemetry telemetry;
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location location = Location.MIDDLE;

    // ROI = region of interest, aka the rectangle we're drawing
    // you should fine tune this
    static final Rect ROI_Left   = new Rect(new Point( 10, 100), new Point(105, 200));
    static final Rect ROI_Middle = new Rect(new Point(120, 100), new Point(205, 200));
    static final Rect ROI_Right  = new Rect(new Point(220, 100), new Point(310, 200));

    public ObjectPositionPipeline(Telemetry t) {
        telemetry = t;
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

        // If something goes wrong, return
        if (mat.empty()) {
            telemetry.addData("processFrame func", "something wnet wrong");
            telemetry.update();
            return input;
        }

        // create s
        Scalar MIN_BLUE = new Scalar(MIN_BLUE_HUE, MIN_SATURATION, MIN_VALUES);
        Scalar MAX_BLUE = new Scalar(MAX_BLUE_HUE, MAX_SATURATION, MAX_VALUES);
        Scalar MIN_RED_LOW = new Scalar(MIN_RED_LOW_HUE, MIN_SATURATION, MIN_VALUES);
        Scalar MAX_RED_LOW = new Scalar(MAX_RED_LOW_HUE, MAX_SATURATION, MAX_VALUES);
        Scalar MIN_RED_HIGH = new Scalar(MIN_RED_HIGH_HUE, MIN_SATURATION, MIN_VALUES);
        Scalar MAX_RED_HIGH = new Scalar(MAX_RED_HIGH_HUE, MAX_SATURATION, MAX_VALUES);

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

        // make submatrices, I don't understand this part yet
        Mat left = mat.submat(ROI_Left);
        Mat middle = mat.submat(ROI_Middle);
        Mat right = mat.submat(ROI_Right);

        // I don't yet understand this but
        // % white can be determined by adding props, dividing by area
        // grayscale image only has one channel so we take [0] only
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
            location = Location.LEFT;
            telemetry.addData("Prop location:", "left");
        }
        else if (rightValue >= middleValue) {
            location = Location.RIGHT;
            telemetry.addData("Prop location:", "right");
        }
        else {
            location = Location.MIDDLE;
            telemetry.addData("Prop location:", "middle");
        }
        telemetry.update();


        /* Draw rectangles to visualize prop location. */
        // grayscale to rgb
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255, 255, 255);
        Scalar propColor = new Scalar(  0,   0, 255);

        Imgproc.rectangle(mat, ROI_Left, location == Location.LEFT ? pixelColor:propColor);
        Imgproc.rectangle(mat, ROI_Middle, location == Location.MIDDLE ? pixelColor:propColor);
        Imgproc.rectangle(mat, ROI_Right, location == Location.RIGHT ? pixelColor:propColor);


        return input;
    }

    public Location getLocation() {
        return location;
    }

    public void setDetectRed(boolean shouldDetectRed) {
        DETECT_RED = shouldDetectRed;
    }
}