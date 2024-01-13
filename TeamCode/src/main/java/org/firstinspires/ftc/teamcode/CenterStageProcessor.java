//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Bitmap;
//import android.graphics.Canvas;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import org.firstinspires.ftc.robotcore.external.function.Continuation;
//import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.core.Mat;
//
//import java.util.concurrent.atomic.AtomicReference;
//
//public class CenterStageProcessor implements VisionProcessor, CameraStreamSource {
//    private final AtomicReference<Bitmap> lastFrame =
//            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
//    private Telemetry telemetry;
//    private FtcDashboard dashboard;
//
//    public CenterStageProcessor(FtcDashboard dashboard, Telemetry telemetry) {
//        this.dashboard = dashboard;
//        this.telemetry = telemetry;
//    }
//
//    /**
//     * This is for the camera streaming.
//     * @param width
//     * @param height
//     * @param calibration
//     */
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
//    }
//
//    /**
//     * This is for the camera streaming, called by the app to retrieve the frames I think.
//     * @param continuation frame bitmap consumer continuation
//     */
//    @Override
//    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
//        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
//    }
//
//    /**
//     * Process the frame.
//     * @param frame
//     * @param captureTimeNanos
//     * @return
//     */
//    @Override
//    public Object processFrame(Mat frame, long captureTimeNanos) {
//        return null;
//    }
//
//    /**
//     * Draw on the frame.
//     * @param canvas
//     * @param onscreenWidth
//     * @param onscreenHeight
//     * @param scaleBmpPxToCanvasPx
//     * @param scaleCanvasDensity
//     * @param userContext
//     */
//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//
//    }
//}
