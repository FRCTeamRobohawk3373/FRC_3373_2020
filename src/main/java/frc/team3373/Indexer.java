package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
    private WPI_TalonSRX ball4;
    private WPI_TalonSRX ball5;
    private int zeroPos4;
    private int zeroPos5;
    private boolean manual;

    public Indexer(int ball4Index, int ball5Index) {
        ball4 = new WPI_TalonSRX(ball4Index);
        ball5 = new WPI_TalonSRX(ball5Index);
        zeroPos4 = ball4.getSensorCollection().getAnalogInRaw();
        zeroPos5 = ball5.getSensorCollection().getAnalogInRaw();
        ball4.setNeutralMode(NeutralMode.Brake);
        ball5.setNeutralMode(NeutralMode.Brake);

        manual = false;

        SmartDashboard.putBoolean("Manual", manual);
        SmartDashboard.putNumber("Zero Pos 4", zeroPos4);
        SmartDashboard.putNumber("Zero Pos 5", zeroPos5);
    }

    public int getBall4Pos() {
        return ball4.getSensorCollection().getAnalogInRaw();
    }

    public int getBall5Pos() {
        return ball5.getSensorCollection().getAnalogInRaw();
    }

    public boolean toggleControl() {
        manual = manual ? false : true;
        SmartDashboard.putBoolean("Manual", manual);
        return manual;
    }

    public boolean rotate4(double speed) {
        if (manual) {
            ball4.set(speed);
            return true;
        }
        return false;
    }

    public void zero() {
        zeroPos4 = getBall4Pos();
        zeroPos5 = getBall5Pos();
        SmartDashboard.putNumber("Zero Pos 4", zeroPos4);
        SmartDashboard.putNumber("Zero Pos 5", zeroPos5);
    }
}
