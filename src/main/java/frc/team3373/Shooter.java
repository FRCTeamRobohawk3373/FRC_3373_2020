package frc.team3373;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.MathUtil;
import frc.team3373.Constants;

public class Shooter {

    CANSparkMax motor;
    double[][] powerTable;
    int liveSpeedDisplay = 0;
    double liveSpeedAdjustment = 0;

    // For calibration
    private boolean isCalibrationDone = false;
    private int calibrationTimes;
    private double currentInches, startInches, endInches, inchesInterval, currentPower;
    private double[][] calibratingTable;

    public Shooter() {
        motor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
        motor.disableVoltageCompensation();
        powerTable = Constants.SHOOT_TABLE;

        if (RobotState.isTest()) {
            currentPower = 0;
            calibrationTimes = 0;
            currentInches = startInches = 24;
            endInches = 140;
            inchesInterval = 12;
            int numberOfIntervals = (int)((endInches-startInches)/inchesInterval+1);
            calibratingTable = new double[numberOfIntervals][2];
        }
    }

    public void setRawSpeed(double speed) {
        motor.set(speed);
    }

    public void setSpeed(double speed) {
        System.out.println(speed);
        motor.set(MathUtil.clamp(speed+liveSpeedAdjustment, 0.0, 1.0));
    }

    public void setSpeedFromDistance(double inches) {
        double val = 0;
        for (int i = 0; i < powerTable.length; i ++) {
            double distLow = powerTable[i][0];
            double powLow = powerTable[i][1];
            if (i == powerTable.length-1) {
                val = powLow;
            } else {
                double distHigh = powerTable[i+1][0];
                double powHigh = powerTable[i+1][1];
                val = MathUtil.lerp(powLow, powHigh, inches/(distHigh-distLow));
            }
        }
        setSpeed(val);
    }

    public void bumpUpSpeed() {
        liveSpeedAdjustment += Constants.SHOOTER_BUMP_AMOUNT;
        liveSpeedDisplay ++;
    }

    public void bumpDownSpeed() {
        liveSpeedAdjustment -= Constants.SHOOTER_BUMP_AMOUNT;
        liveSpeedDisplay --;
    }

    public void updateTeleOp() {
   
        String motorInfo;
        String positiveSignPrefix = "";
        if (liveSpeedDisplay >= 0) {
            positiveSignPrefix = "+";
        }
        double motorSpeed = motor.get();
        if (motorSpeed > 0) {
            motorInfo = " ("+MathUtil.round(motorSpeed, 2)+")";
        } else {
            motorInfo = "";
        }
        SmartDashboard.putString("Shooter", positiveSignPrefix+liveSpeedDisplay+motorInfo);

    }

    public void updateTestMode() {
        if (isCalibrationDone) {
            SmartDashboard.putString("Shooter Calibration", "Done");
        } else {
            SmartDashboard.putString("Shooter Calibration", "Go to: "+currentInches+" inches");
        }
        SmartDashboard.putNumber("Shooter (set)", currentPower);
        SmartDashboard.putNumber("Shooter (get)", motor.get());

        motor.set(currentPower);
    }

    public void calibrationMotorSpeed(boolean isBLPushed, boolean isBRPushed, double lTrigger, double rTrigger) {
        if (isBLPushed) currentPower -= 0.001;
        if (isBRPushed) currentPower += 0.001;
        currentPower -= Math.pow(lTrigger*0.12, 3);
        currentPower += Math.pow(rTrigger*0.12, 3);
        currentPower = MathUtil.clamp(currentPower, 0, 1);
    }

    public void nextCalibrationInterval() {
        if (isCalibrationDone) {
            currentPower = 0;
        } else {
            calibratingTable[calibrationTimes] = new double[]{currentInches, currentPower};
            calibrationTimes ++;
            currentInches += inchesInterval;
            if (currentInches > endInches) {
                currentPower = 0;
                printTable(calibratingTable);
                isCalibrationDone = true;
            }
        }
    }

    private void printTable(double[][] table) {
        String printThis = "Table output:\n\n\nSHOOT_TABLE = new double[][]{\n";
        for(int i = 0; i < table.length; i++) {
            printThis += String.format("\t{%.2f, %.5f},\n", table[i][0], table[i][1]);
        }
        System.out.println(printThis+"};");
    }
}
        