package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

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
    public int min_area = 1000;
    public int dilationSize = 18;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the input frame to grayscale
        Mat grayFrame = new Mat();
        Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_RGBA2GRAY);

        // Apply Gaussian blur with a larger kernel for noise reduction
        if (blur != 0) Imgproc.GaussianBlur(grayFrame, grayFrame, new Size(blur, blur), 0); // Adjust kernel size as needed

        // Perform edge detection using Canny with adjusted threshold values
        Mat edges = new Mat();
        Imgproc.Canny(grayFrame, edges, canny_threshold1, canny_threshold2); // Adjust threshold values as needed

        // Apply morphological operations with adjusted kernel sizes and shapes
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(filter_size, filter_size));
        Imgproc.dilate(edges, edges, kernel);
        Imgproc.erode(edges, edges, kernel);

        // Find contours in the binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Dilate the contours to make them thicker
        Mat dilatedEdges = new Mat();
        Mat dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilationSize, dilationSize));
        Imgproc.dilate(edges, dilatedEdges, dilateKernel);
        Core.bitwise_or(dilatedEdges, edges, dilatedEdges);
        Core.bitwise_not(dilatedEdges, dilatedEdges);

        // Find contours in the dilated binary image
        List<MatOfPoint> dilatedContours = new ArrayList<>();
        Imgproc.findContours(dilatedEdges, dilatedContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw lines around detected contours on the original frame
        for (MatOfPoint contour : dilatedContours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea < min_area) {
                continue;
            }

            // Find the bounding box of the contour
            Imgproc.fillPoly(frame, Collections.singletonList(contour), new Scalar(0, 255, 0)); // You can adjust the color here
        }

        // Release resources
        grayFrame.release();
        edges.release();
        kernel.release();
        dilatedEdges.release();
        dilateKernel.release();

        telemetry.update();
        return null;
    }




    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
