package frc.team3373;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.team3373.Constants;

public class Shooter {

    CANSparkMax motor;

    


    public Shooter() {
        motor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);

    }

    public void setSpeedFromDistance(double inches) {
 
    }

    public void update() {

    }

}
        