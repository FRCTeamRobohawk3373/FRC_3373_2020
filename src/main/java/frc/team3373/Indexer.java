package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
    private int scale;
    private WPI_TalonSRX ball3;
    private WPI_TalonSRX ball4;
    // TODO sensor code
    private boolean manual;

    public Indexer(int ball3Index, int ball4Index) {
        ball3 = new WPI_TalonSRX(ball3Index);
        ball4 = new WPI_TalonSRX(ball4Index);
        ball3.setNeutralMode(NeutralMode.Brake);
        ball4.setNeutralMode(NeutralMode.Brake);

        manual = false;
        scale = (int)Config.getNumber("encoderScale", 1992);

        SmartDashboard.putBoolean("Manual", manual);
    }

    public void update() {
        // If sensor detects a ball, stop motors?
        scale = (int)Config.getNumber("encoderScale", 1992);
    }

    public int getAbsBall3Pos() {
        return ball3.getSensorCollection().getQuadraturePosition();
    }

    public int getRelBall3Pos() {
        int relPos = getAbsBall3Pos() % scale;
        if (relPos < 0) {
            return scale + relPos;
        }
        return relPos;
    }

    public int getAbsBall4Pos() {
        return ball4.getSensorCollection().getQuadraturePosition();
    }

    public int getRelBall4Pos() {
        int relPos = getAbsBall4Pos() % scale;
        if (relPos < 0) {
            return scale + relPos;
        }
        return relPos;
    }

    public boolean toggleControl() {
        manual = !manual;
        if (manual) {
            ball3.setNeutralMode(NeutralMode.Coast);
            ball4.setNeutralMode(NeutralMode.Coast);
        }
        SmartDashboard.putBoolean("Manual", manual);
        return manual;
    }

    public boolean rotate3(double speed) {
        if (manual) {
            ball3.set(speed);
            return true;
        }
        return false;
    }

    public boolean rotate4(double speed) {
        if (manual) {
            ball4.set(speed);
            return true;
        }
        return false;
    }


    public void zero() {
        ball3.getSensorCollection().setQuadraturePosition(0, 20);
        ball4.getSensorCollection().setQuadraturePosition(0, 20);
    }
}
