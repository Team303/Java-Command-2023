package com.team303.lib.math;

public class Point2D {
    public double x;
    public double y;
    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public int xAsInt() {
        return (int)this.x;
    }

    public int yAsInt() {
        return (int)this.y;
    }

    public double xAsDouble() {
        return this.x;
    }

    public double yAsDouble() {
        return this.y;
    }
}
