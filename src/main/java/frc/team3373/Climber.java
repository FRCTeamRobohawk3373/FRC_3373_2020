package frc.team3373;

import com.revrobotics.CANSparkMax;

public class Climber {

    CANSparkMax poleMotor, winchMotor;

    double zeroInchesAt = 0;
    double conversion = 0;
 
    public Climber() {
        poleMotor = new CANSparkMax(0);
        winchMotor = new CANSparkMax(0);
    }

    public void gotoInches(double inches) {

    }

    public void setZeroInches(int ticks) {
        zeroInchesAt = ticks;
    }

    public void setTickToInch(double conversion) {
        this.conversion = conversion;
    }
}