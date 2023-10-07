package org.firstinspires.ftc.teamcode;

public class DistanceRep {
    Point2d start_point;
    Point2d end_point;
    double distance;
    double height;
    double width;

    public DistanceRep(Point2d start_point, Point2d end_point, double height, double width) {
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
