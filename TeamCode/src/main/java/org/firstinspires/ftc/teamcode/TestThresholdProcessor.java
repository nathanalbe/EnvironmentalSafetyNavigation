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
    public Scalar lower = new Scalar(29.8, 119, 36.8);
    public Scalar upper = new Scalar(46.8, 229.5, 181.3);

    public int pov_x = 0;
    public int pov_y = 0;

    public ColorSpace colorSpace = ColorSpace.HSV;

    // Mask
    private Mat mat = new Mat();

    // Return - objects
    private Mat ret = new Mat();

    // Minimum area on the screen for box
    public int rect_threshold = 1000;

    // Data logger
    private Telemetry telemetry = null;

    // Sets the data logger
    public TestThresholdProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // Dont know
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    // Get the frame, and gets the time.
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Frees the memory of the frame, from frame
        ret.release();

        // New Frame with the size
        ret = new Mat();

        try {
            // Gets the center of the x, and a bit above the bottom of the screen
            pov_x = frame.width() / 2;
            pov_y = frame.height() - 5;

            // convert RGB to HSV
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

            // Remove all rects whose area is less than rect_threshold
            for (int i = 0; i < rects.size(); i++) {
                Rect rect = rects.get(i);
                if (rect.area() < rect_threshold) {
                    rects.remove(i);
                    i--;
                }
            }

            // Draw rectangle around each contour and circle in center of each bounding box
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

            // Draw line from pov_x/pov_y to each center
            Point2d pov = new Point2d(pov_x, pov_y);
            ArrayList<DistanceRep> distances = new ArrayList<>();
            for (Point2d center : centers) {
                distances.add(new DistanceRep(pov, center));
                //Imgproc.line(ret, new Point(pov_x, pov_y), center.toPoint(), new Scalar(255, 0, 0), 2);
            }

            // Sort distances by distance
            distances.sort((a, b) -> (int) (a.get_distance() - b.get_distance()));

            // Draw the 3 closest lines, closest in red, 2nd closest in green, 3rd closest in blue
            for (int i = 0; i < distances.size() && i < 3; i++) {
                DistanceRep distance = distances.get(i);
                Scalar color = new Scalar(0, 0, 0);
                if (i == 0) // closest
                    color = new Scalar(255, 0, 0);
                else if (i == 1) // 2nd closest
                    color = new Scalar(0, 255, 0);
                else if (i == 2) // 3rd closest
                    color = new Scalar(0, 0, 255);
                Imgproc.line(ret, distance.get_start_point().toPoint(), distance.get_end_point().toPoint(), color, 2);
            }

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
