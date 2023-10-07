package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

public class Point2d {
    public double x;
    public double y;

    public Point2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point2d() {
        this.x = 0;
        this.y = 0;
    }

    public Point2d(Point p) {
        x = p.x;
        y = p.y;
    }

    public void set_x(double x) {
        this.x = x;
    }

    public void set_y(double y) {
        this.y = y;
    }

    public void setPoint(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public void setPoint(Point2d other) {
        this.x = other.x;
        this.y = other.y;
    }

    public double get_x() {
        return this.x;
    }

    public double get_y() {
        return this.y;
    }

    public Point2d add(Point2d other) {
        return new Point2d(this.x + other.x, this.y + other.y);
    }

    public Point2d sub(Point2d other) {
        return new Point2d(this.x - other.x, this.y - other.y);
    }

    public Point2d mul(double scalar) {
        return new Point2d(this.x * scalar, this.y * scalar);
    }

    public Point2d div(double scalar) {
        return new Point2d(this.x / scalar, this.y / scalar);
    }

    public double distance (Point2d other) {
        return Math.sqrt(Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2));
    }

    public Point toPoint() {
        return new Point(this.x, this.y);
    }
}
