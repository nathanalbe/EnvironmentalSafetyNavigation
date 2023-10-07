package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.Comparator;

public class HaarCascadeObjectProcessor implements VisionProcessor {
    private CascadeClassifier objectCascade = new CascadeClassifier("/Users/nathanalbe/downloads/haarcascade_fullbody.xml");
    private MatOfRect detectedObjects = new MatOfRect();
    private Telemetry telemetry;

    public int pov_x = 0;
    public int pov_y = 0;
    public double prop = 0.182353;

    public HaarCascadeObjectProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(int width, int height, CameraCalibration calibration) {
    }

    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame to grayscale (required for Haar cascades)
        Mat gray = new Mat();
        try {
            pov_x = frame.width() / 2;
            pov_y = frame.height() - 5;
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

            // Detect objects using the Haar cascade
            objectCascade.detectMultiScale(gray, detectedObjects, 1.1,        // scaleFactor (adjust as needed)
                    3,          // minNeighbors (adjust as needed)
                    0,          // flags
                    new Size(100, 100),  // minSize (adjust as needed)
                    new Size(500, 500) );

            // Draw rectangles around detected people
            Rect[] objectsArray = detectedObjects.toArray();

            for (Rect rect : objectsArray) {
                // Draw a rectangle around the detected person
                Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(0, 255, 0), 2);
            }

            Imgproc.circle(frame, new Point(pov_x, pov_y), 5, new Scalar(255, 0, 0), 5);
            // Display the number of people detected
            telemetry.addData("People Detected", objectsArray.length);
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        }

        return null;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
