package org.firstinspires.ftc.teamcode.geometry;

import org.opencv.core.Point;

public class Line2d {
    Point2d start_point;
    Point2d end_point;
    double distance;
    double height;
    double width;

    public Line2d(Point start_point, Point2d end_point) {
        this.start_point = new Point2d(start_point);
        this.end_point = end_point;
        this.distance = this.start_point.distance(end_point);
        this.height = 0;
        this.width = 0;
    }

    public Line2d(Point2d start_point, Point2d end_point) {
        this.start_point = start_point;
        this.end_point = end_point;
        this.distance = start_point.distance(end_point);
        this.height = 0;
        this.width = 0;
    }

    public Line2d(Point2d start_point, Point2d end_point, double height, double width) {
        this.start_point = start_point;
        this.end_point = end_point;
        this.distance = start_point.distance(end_point);
        this.height = height;
        this.width = width;
    }

    public Point2d get_start_point() {
        return this.start_point;
    }

    public Point2d get_end_point() {
        return this.end_point;
    }

    public double get_distance() {
        return this.distance;
    }

    public double get_height() {
        return this.height;
    }

    public double get_width() {
        return this.width;
    }
}
