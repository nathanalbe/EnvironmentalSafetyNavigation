package org.firstinspires.ftc.teamcode;

public class DistanceRep {
    Point2d start_point;
    Point2d end_point;
    double distance;

    public DistanceRep(Point2d start_point, Point2d end_point) {
        this.start_point = start_point;
        this.end_point = end_point;
        this.distance = start_point.distance(end_point);
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
}
