package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.*;

import java.util.ArrayList;

public class TestThresholdProcessor implements VisionProcessor {
    public Scalar lower = new Scalar(0, 137.4, 164.3);
    public Scalar upper = new Scalar(255, 172.8, 255);

    public int pov_x = 0;
    public int pov_y = 0;

    public ColorSpace colorSpace = ColorSpace.Lab;

    private Mat mat = new Mat();
    private Mat ret = new Mat();

    private Telemetry telemetry = null;

    public TestThresholdProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        ret.release();
        ret = new Mat();

        try {
            pov_x = frame.width() / 2;
            pov_y = frame.height() / 2 + 220;

            // convert RBB to Lab
            Imgproc.cvtColor(frame, mat, colorSpace.cvtCode);

            // convert to b/w by removing all colors not in range
            Mat mask = new Mat(mat.rows(), mat.cols(), mat.type());
            //frame.copyTo(mask);
            Core.inRange(mat, lower, upper, mask);

            // Set background to black
            Core.bitwise_and(frame, frame, ret, mask);

            // Blur to remove noise
            Imgproc.GaussianBlur(mask, mask, new org.opencv.core.Size(3, 3), 0);

            // Find contours
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            telemetry.addData("Contours", contours.size());

            // Create a list of each bounding rect
            ArrayList<Rect> rects = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                rects.add(Imgproc.boundingRect(contour));
            }

            // Draw rectangle around each countour and circle in center of each bounding box
            ArrayList<Point2d> centers = new ArrayList<>();
            for (Rect rect : rects) {
                int x = rect.x + rect.width / 2;
                int y = rect.y + rect.height / 2;
                centers.add(new Point2d(x, y));
                Imgproc.rectangle(ret, rect, new Scalar(0, 255, 0), 2);
                Imgproc.circle(ret, new Point(x, y), 5, new Scalar(0, 0, 255), 5);
            }

            // Draw circle in center of frame
            Imgproc.circle(ret, new Point(pov_x, pov_y), 5, new Scalar(255, 0, 0), 5);

            ret.copyTo(frame);
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        }

        telemetry.update();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
