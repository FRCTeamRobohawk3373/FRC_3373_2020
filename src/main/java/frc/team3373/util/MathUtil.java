package frc.team3373.util;

public class MathUtil {

    /**
     * Rounds a double to a number of decimal point.
     * 
     * @param val       Value to round.
     * @param precision Number of decimal points to round to.
     */
    public static double round(double val, int precision) {
        final double exp = Math.pow(10, precision);
        return Math.round(val * exp) / exp;
    }

    /**
     * Constrains an int between a min and max.
     * 
     * @param val the input value.
     * @param min the minimum value.
     * @param max the maxium value.
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Constrains a double between a min and max.
     * 
     * @param val the input value. the input value.
     * @param min the minimum value.
     * @param max the maxium value.
     */
    public static int clamp(int val, int min, int max) {
        return Math.max(min, Math.min(max, val));
    }
    
    /**
     * Returns true if value is between min and max
     * 
     * @param val the input value. the input value.
     * @param min the minimum value.
     * @param max the maxium value.
     */
    public static boolean inRange(double val, double min, double max) {
        return val > min && val < max;
    }

    /**
     * gets a value in between two values based on a percentage
     * @param start starting value
     * @param end ending value
     * @param t percent in between
     */
    public static double lerp(double start, double end, double t) {
		return start + (end - start) * t;
	}

    /**
     * gets a value in between two values based on a percentage
     * @param start starting value
     * @param end ending value
     * @param query the lineraly interpolated value
     */
	public static double inverseLerp(double start, double end, double query) {
		return inverseLerp(start, end, query, true);
	}

    /**
     * gets a value in between two values based on a percentage
     * @param start starting value
     * @param end ending value
     * @param query the lineraly interpolated value
     * @param shouldClamp whether or not the finale value should be clamped between 0-1.0
     */
	public static double inverseLerp(double start, double end, double query, boolean shouldClamp) {
		double t = (query - start) / (end - start);
		if (shouldClamp) {
			return clamp(t, 0.0, 1.0);
		} else {
			return t;
		}
	}

    /**
     * nomalizes the value between 0 to 1 for a given range.
     * 
     * @param val the input value.
     * @param min the minimum value.
     * @param max the maxium value.
     */
    public static double normalise(double val, double min, double max) {
        val = clamp(val, min, max);
        return (val - min) / (max - min);
    }

    /**
     * Calculates an always positive remainder.
     * 
     * @param x numerator
     * @param y denominator
     */
    public static double floorMod(double x, double y) {
        return x - (Math.floor(x / y) * y);
    }

    /**
     * converts the range of polar corodanates from (-PI, PI) to (0, 2PI)
     * @param value the value in radians from (-PI, PI)
     */
    public static double translatePIRange(double value){
        if(value<0){
			value+=2*Math.PI;
		}
		return value;
    }

    /**
     * Returns true if it is optimal for an object to rotate clockwise to reach a
     * destination angle
     * 
     * @param angleStart       Current angle of an object along a rotational axis in
     *                         degrees.
     * @param angleDestination Destination angle of that object in degrees.
     */
    public static boolean isClockwiseRotationNearer(double angleStart, double angleDestination) {
        angleStart = floorMod(angleStart, 360);
        angleDestination = floorMod(angleStart, 360);
        if (angleStart > angleDestination + 180) {
            angleStart -= 360;
        } else if (angleStart < angleDestination - 180) {
            angleStart += 360;
        }
        return angleStart > angleDestination;
    }

}