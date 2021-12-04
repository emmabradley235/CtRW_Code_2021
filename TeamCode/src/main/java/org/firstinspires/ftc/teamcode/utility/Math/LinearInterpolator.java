package org.firstinspires.ftc.teamcode.utility.Math;


import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Point2d;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.TimestampedValue;


public class LinearInterpolator {
    double slope;
    double offset;
    Point2d startPoint;
    Point2d endPoint;


    public LinearInterpolator(double startTime, double startValue, double endTime, double endValue){ // constructor, creates a line to interpolate from using points given
        if( startTime - endTime != 0 ){
            this.slope = (endValue - startValue) / (endTime - startTime); // rise over run baybeee
        }

        double projectedStartValue = startTime * slope; // see what value no offset gives
        this.offset = startValue - projectedStartValue; // then compare that to the actual value, and set the offset equal to the difference

        this.startPoint = new Point2d(startTime, startValue);
        this.endPoint = new Point2d(endTime, endValue);
    }
    public LinearInterpolator(TimestampedValue startValue, TimestampedValue endValue){
        this(startValue.timestamp, startValue.value, endValue.timestamp, endValue.value);
    }
    public LinearInterpolator(Point2d startPoint, Point2d endPoint){
        this(startPoint.x, startPoint.y, endPoint.x, endPoint.y);
    }


    public double interpolate(double timestep){
        return (slope*timestep) + offset; // apply  y = mx + b   (with x = timestep)
    }

    public double getSlope(){return slope;}
    public double getOffset(){return offset;}

    public String toString(){
        return "LinearInterp( Slope = " + slope + ", Offset = " + offset + " )";
    }
}
