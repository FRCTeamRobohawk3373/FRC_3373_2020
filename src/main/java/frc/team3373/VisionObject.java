package frc.team3373;

public class VisionObject {
    public double x;
    public double y;
    public double distance;
    public double rAngle;
    public double tAngle;

    public VisionObject(double positionX, double positionY, double distance, double robotAngle, double targetAngle) {
        x = positionX;
        y = positionY;
        this.distance = distance;
        rAngle = robotAngle;
        tAngle = targetAngle;
    }

    public String toString() {
        return "Target at ("+x+","+y+") "+distance+"in, Robot Angle: "+rAngle+" degrees, Target Angle: "+tAngle+" degrees";
        //return "X: " + x + "\nY: " + y + "\nDistance: " + distance + "\nRobot Angle: " + rAngle + "\nTarget Angle: "
        //        + tAngle;
    }

}
