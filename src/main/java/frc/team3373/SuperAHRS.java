package frc.team3373;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperAHRS extends AHRS {
	// Reset function, get 0-360 rotation, bump detection (get bump), get altitude (starts at zero, positive numbers up)
	
	private double previousAccelerationZ;
	private boolean hasBumped;

	private double targetAngle;

	private static SuperAHRS instance;

	public static SuperAHRS getInstance(){
		if(instance == null){
			instance = new SuperAHRS(SPI.Port.kMXP);
		}

		return instance;
	}
	
	SuperAHRS (SPI.Port port) {
		super(port);
		previousAccelerationZ = super.getWorldLinearAccelZ();
		hasBumped = false;
	}

	public float getRotation() {
		float rotation;
		rotation=(360-super.getYaw())%360;
		
		/*if (super.getYaw() >= 0) {
			rotation = super.getYaw();
		} else {
			rotation = (180 - Math.abs(super.getYaw())) + 180;
		}*/
		return rotation;
	}
	public float getRawRotation() {
		return super.getYaw();
	}
	
	private double getZJerk() {
		double currentAccel = super.getWorldLinearAccelZ();
		double deltaAccel = currentAccel - previousAccelerationZ;
		previousAccelerationZ = currentAccel;
		return deltaAccel/.01;
	}
	
	public boolean hasHitBump (double amount) {
		if(Math.abs(this.getZJerk())>amount)
			hasBumped = true;
		return hasBumped;
	}
	
	public void resetBump() {
		hasBumped = false;
	}

	public void setTargetAngle(double angle) {
		targetAngle = angle;
	}

	@Override
	public double pidGet() {
		SmartDashboard.putNumber("pidInput", (super.getYaw() - targetAngle) + 0.5);
		return (super.getYaw() - targetAngle) + 0.5;
	}
}