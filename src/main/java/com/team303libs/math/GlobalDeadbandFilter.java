package com.team303libs.math;

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

    public double applyDeadband(double value, double lowerBound, double upperBound, double scale) {
        if (Math.abs(value) < lowerBound && Math.abs(value) > upperBound) {
        System.out.println("Global deadband applied");
        return 0;
        }
        return scale * Math.signum(value) * (Math.abs(value)-lowerBound) / (upperBound-lowerBound);
    }
    public double applyDeadband(double value, double[] bounds) {
        return applyDeadband(value,bounds[0],bounds[1],1);
    }
    public double applyDeadband(double value, double lowerBound) {
        return applyDeadband(value,lowerBound,1,1);
    }
    public double applyDeadband(double value, double lowerBound, double scale) {
        return applyDeadband(value,lowerBound,1,scale);
    }

    
}
