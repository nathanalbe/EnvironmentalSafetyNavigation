package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class ContourTest implements VisionProcessor {

    private Telemetry telemetry = null;

    public ContourTest(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int blur = 1;
    public int canny_threshold1 = 25;
    public int canny_threshold2 = 75;
    public int filter_size = 5;
    public int min_area = 2500;
    public int max_area = 500000;
    public int dilationSize = 12;

    public final float minDistance = 300f;
    public double prop = 0.182353;

    public int bar_width = 50;
    public int bar_height = 120;

    private final Point2d pov = new Point2d(0, 0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        pov.setPoint(frame.width() / 2, frame.height() - 10);
        try {
            // Convert the input frame to grayscale
            Mat grayFrame = new Mat();
            Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_RGBA2GRAY);

            // Apply Gaussian blur with a larger kernel for noise reduction
            if (blur != 0)
                Imgproc.GaussianBlur(grayFrame, grayFrame, new Size(blur, blur), 0); // Adjust kernel size as needed

            // Perform edge detection using Canny with adjusted threshold values
            Mat edges = new Mat();
            Imgproc.Canny(grayFrame, edges, canny_threshold1, canny_threshold2); // Adjust threshold values as needed
            grayFrame.release();

            // Apply morphological operations with adjusted kernel sizes and shapes
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(filter_size, filter_size));
            Imgproc.dilate(edges, edges, kernel);
            Imgproc.erode(edges, edges, kernel);
            kernel.release();

            // Find contours in the binary image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Dilate the contours to make them thicker
            Mat dilatedEdges = new Mat();
            Mat dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilationSize, dilationSize));
            Imgproc.dilate(edges, dilatedEdges, dilateKernel);
            dilateKernel.release();
            Core.bitwise_or(dilatedEdges, edges, dilatedEdges);
            Core.bitwise_not(dilatedEdges, dilatedEdges);
            edges.release();

            // Find contours in the dilated binary image
            List<MatOfPoint> dilatedContours = new ArrayList<>();
            Imgproc.findContours(dilatedEdges, dilatedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            dilatedEdges.release();

            // Draw lines around detected contours on the original frame
            ArrayList<DistanceRep> distances = new ArrayList<>();
            for (MatOfPoint contour : dilatedContours) {
                double contourArea = Imgproc.contourArea(contour);
                if (contourArea < min_area || contourArea > max_area) {
                    continue;
                }
                // Find the bounding box of the contour
                Imgproc.fillPoly(frame, Collections.singletonList(contour), new Scalar(29, 148, 47, 0.05));
                // draw center of each contour
                Moments moments = Imgproc.moments(contour);
                Point center = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());
                Imgproc.circle(frame, center, 5, new Scalar(0, 0, 0), 5);
                distances.add(new DistanceRep(center, pov));
            }

            //compare distances
            ////distances.sort(Comparator.comparingDouble(DistanceRep::get_distance));
            //compare distances by start point y value
            distances.sort((o1, o2) -> (int) -(o1.get_start_point().y - o2.get_start_point().y));

            for (int i = 0; i < distances.size() && i < 3; i++) {
                DistanceRep point = distances.get(i);
                Scalar color = new Scalar(0, 0, 0);
                if (i == 0) // closest
                    color = new Scalar(229, 15, 87);
                else if (i == 1) // 2nd closest
                    color = new Scalar(75, 196, 245);
                else if (i == 2) // 3rd closest
                    color = new Scalar(170, 39, 251);
                Imgproc.line(frame, point.get_start_point().toPoint(),
                        point.get_end_point().toPoint(), color, 10);
                telemetry.addData(i + ": Distance " + (point.get_distance()) + "in, " +
                        "Height: " + point.get_height() + ", Width" + point.get_width(), "");
            }

            DistanceRep leftPoint = closestLeft(distances);
            DistanceRep rightPoint = closestRight(distances);

            float left = 0;
            if (leftPoint.get_distance() * prop <= minDistance && leftPoint.get_start_point().get_y() > pov.y / 2.0) {
                left = weightedDistance(leftPoint) + weightedYPost(leftPoint);
            } else {
                left = 0;
            }

            float right = 0;
            if (rightPoint.get_distance() * prop <= minDistance && rightPoint.get_start_point().get_y() > pov.y / 2.0) {
                right = weightedDistance(rightPoint) + weightedYPost(rightPoint);
            } else {
                right = 0;
            }

            telemetry.addData("left: ", left);
            telemetry.addData("right: ", right);

            displayBars(left, right, frame);

        } catch (Exception e) {
            telemetry.addData("Exception", e.getMessage());
        }

        telemetry.update();
        return null;
    }

    private float weightedDistance(DistanceRep point) {
        float distance =
                (float) ((Math.abs(minDistance - (point.get_distance() * prop)) / minDistance) * 0.40f);
        if (point.get_distance() * prop < 50) {
            return 0.4f;

        }
        return distance;
    }

    private float weightedYPost(DistanceRep point) {
        float weight = (float) (Math.abs(pov.y - point.get_start_point().get_y()) / pov.y) * 0.6f;
        if (point.get_start_point().get_y() >= 800) {
            return 0.6f;
        }
        return weight;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
    }

    private DistanceRep closestRight(ArrayList<DistanceRep> closest_points) {
        int index = -1;
        DistanceRep closest = new DistanceRep(new Point2d(0, 0), new Point2d(0, 0), 0, 0);
        for (int i = 0; i < closest_points.size(); i++) {
            if (isRight(closest_points.get(i))) {
                closest = closest_points.get(i);
                index = i;
                break;
            }
        }

        if (index == -1) {
            return closest;
        }

        for (int i = index; i < closest_points.size(); i++) {
            if (isRight(closest_points.get(i))) {
                if (closest_points.get(i).get_distance() < closest.get_distance()) {
                    closest = closest_points.get(i);
                }
            }
        }
        return closest;
    }

    private DistanceRep closestLeft(ArrayList<DistanceRep> closest_points) {
        int index = -1;
        DistanceRep closest = new DistanceRep(new Point2d(0, 0), new Point2d(0, 0), 0, 0);
        for (int i = 0; i < closest_points.size(); i++) {
            if (isLeft(closest_points.get(i))) {
                closest = closest_points.get(i);
                index = i;
                break;
            }
        }

        if (index == -1) {
            return closest;
        }

        for (int i = index; i < closest_points.size(); i++) {
            if (isLeft(closest_points.get(i))) {
                if (closest_points.get(i).get_distance() < closest.get_distance()) {
                    closest = closest_points.get(i);
                }
            }
        }

        return closest;
    }

    private boolean isLeft(DistanceRep point) {
        return point.get_start_point().get_x() <= pov.x;
    }

    private boolean isRight(DistanceRep point) {
        return point.get_start_point().get_x() >= pov.x;
    }

    private void displayBars(float left, float right, Mat frame) {
        //left bar
        int left_bar_x = frame.width() - bar_width * 2;
        int left_bar_fill_height = (int) (left * bar_height);
        int left_bar_fill_y = bar_height - left_bar_fill_height;
        Imgproc.rectangle(frame, new Point(left_bar_x, 0), new Point(left_bar_x + bar_width, bar_height), new Scalar(0, 0, 0, 0.0), -1);

        //right bar
        int right_bar_x = frame.width() - bar_width;
        int right_bar_fill_height = (int) (right * bar_height);
        int right_bar_fill_y = bar_height - right_bar_fill_height;
        Imgproc.rectangle(frame, new Point(right_bar_x, 0), new Point(right_bar_x + bar_width, bar_height), new Scalar(0, 0, 0, 0.0), -1);

        // fill left bar
        Imgproc.rectangle(frame, new Point(left_bar_x, left_bar_fill_y), new Point(left_bar_x + bar_width, bar_height), new Scalar(255, 0, 0, 1.0), -1);

        // fill right bar
        Imgproc.rectangle(frame, new Point(right_bar_x, right_bar_fill_y), new Point(right_bar_x + bar_width, bar_height), new Scalar(0, 0, 255, 1.0), -1);
    }
}
