/* Notes

Basics of OpenCV: https://docs.opencv.org/3.4/d6/d6d/tutorial_mat_the_basic_image_container.html
Example of filtering: https://stackoverflow.com/questions/36693348/java-opencv-core-inrange-input-parameters
Example code: https://github.com/FTCLib/FTCLib/blob/master/core/vision/src/main/java/com/arcrobotics/ftclib/vision/UGContourRingPipeline.kt

The reason why the camera detection was not working well was likely that the color to detect 
for the team prop was not tuned well, so it was detecting too many different colors and getting 
confused about which section had the most of the correct blue/red color since how it works is 
that it simply filters sections that have the correct range of colors.

Two fixes:
- Tune the color ranges to detect
- Do a Gaussian blur at the start since if the image is blurred/smoothed out then the colors 
  might be more obvious (it seems that this is used a lot in computer vision)
*/
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

        // If something goes wrong, return
        if (mat.empty()) {
            this.telemetry.addData("processFrame func", "something went wrong");
            this.telemetry.update();
            return input;
        }

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
        // We do the [0] since the image is grayscale now
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