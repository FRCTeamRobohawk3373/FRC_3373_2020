package frc.team3373;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;


public class Climber {

    final double poleMotorSpeed = 0.1;
    final double winchMotorSpeed = 0.1;
    final double threshold = 5;

    CANSparkMax poleMotor, winchMotor;
    CANEncoder poleEncoder, winchEncoder;

    PIDController pidtest;

    double zeroInchesAt = 0;
    double conversion = 0;
    double gotoInches = 10;
 
    public Climber() {
        System.out.println("Climber class created");
        poleMotor = new CANSparkMax(0, MotorType.kBrushless);
        winchMotor = new CANSparkMax(1, MotorType.kBrushless);
        poleEncoder = new CANEncoder(poleMotor);
        winchEncoder = new CANEncoder(winchMotor);
        pidtest = new PIDController(0, 0, 0);
        
    }

    public void gotoInches(double inches) {
        gotoInches = inches;
    }

    public void setZeroInches(int ticks) {
        zeroInchesAt = ticks;
    }

    public void setPoleConversion(double conversion) {// todo check if this works
        poleEncoder.setPositionConversionFactor(conversion);
    }

    public void setWinchConversion(double conversion) {// todo check if this works
        poleEncoder.setPositionConversionFactor(conversion);
    }

    public void update() {
        double polePos = poleEncoder.getPosition();
        double winchPos = winchEncoder.getPosition();

        if (polePos > gotoInches+threshold) {
            poleMotor.set(poleMotorSpeed);
        } else if (polePos < gotoInches-threshold) {
            poleMotor.set(-poleMotorSpeed);
        }

        if (winchPos > gotoInches+threshold) {
            winchMotor.set(winchMotorSpeed);
        } else if (winchPos < gotoInches-threshold) {
            winchMotor.set(-winchMotorSpeed);
        }


    }
}