package com.team303.robot.util;

public class GlobalDeadbandFilter {
    private final double lowerBound;
    private final double upperBound;


    public GlobalDeadbandFilter(double low,double high) {
        this.lowerBound = low;
        this.upperBound = high;
    }


    public double getUpperBound() {
        return upperBound;
    }

    public double getLowerBound() {
        return lowerBound;
    }

    public double applyDeadband(double value) {
        if (Math.abs(value) < lowerBound && Math.abs(value) > upperBound) {
        System.out.println("Global deadband applied");
        return 0;
        }
        return value;
    }

    
}
