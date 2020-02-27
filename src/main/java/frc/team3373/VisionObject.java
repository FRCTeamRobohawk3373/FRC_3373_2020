package frc.team3373;

public class VisionObject {
    public double x;
    public double y;
    public double distance;
    public double rotation;

    public VisionObject(double positionX, double positionY, double distance, double rotation) {
        x = positionX;
        y = positionY;
        this.distance = distance;
        this.rotation = rotation;
    }

    public void print(){
		System.out.println("Target at (" + x + "," + y + ") " + distance + "in and " + rotation + " degrees.");
    }
    
}