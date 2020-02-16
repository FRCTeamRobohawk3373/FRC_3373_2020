package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
    private final int scale = 1988;
    private WPI_TalonSRX ball4;
    private WPI_TalonSRX ball5;
    private int ball4Zero;
    private int ball5Zero;
    private boolean manual;

    public Indexer(int ball4Index, int ball5Index) {
        ball4 = new WPI_TalonSRX(ball4Index);
        ball5 = new WPI_TalonSRX(ball5Index);
        ball4Zero = ball4.getSensorCollection().getAnalogInRaw();
        ball5Zero = ball5.getSensorCollection().getAnalogInRaw();
        ball4.setNeutralMode(NeutralMode.Brake);
        ball5.setNeutralMode(NeutralMode.Brake);

        manual = true;

        SmartDashboard.putBoolean("Manual", manual);
        SmartDashboard.putNumber("Zero Pos 4", ball4Zero);
        SmartDashboard.putNumber("Zero Pos 5", ball5Zero);
    }

    public int getAbsBall4Pos() {
        return ball4.getSensorCollection().getQuadraturePosition();
    }

    public int getRelBall4Pos() {
        int mod = (getAbsBall4Pos() % scale) - ball4Zero;
        return (mod >= 0) ? mod : scale + mod;
    }

    public int getBall5Pos() {
        return ball5.getSensorCollection().getAnalogInRaw();
    }

    public boolean toggleControl() {
        manual = !manual;
        if (manual) {
            ball4.setNeutralMode(NeutralMode.Coast);
            ball5.setNeutralMode(NeutralMode.Coast);
        }
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
        ball4Zero = getRelBall4Pos();
        ball5Zero = getBall5Pos();
        SmartDashboard.putNumber("Ball 4 Zero", ball4Zero);
        SmartDashboard.putNumber("Ball 5 Zero", ball5Zero);
    }
}
