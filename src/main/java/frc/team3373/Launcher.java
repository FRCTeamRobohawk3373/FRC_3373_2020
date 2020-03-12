package frc.team3373;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.MathUtil;
import frc.team3373.Constants;

public class Launcher {

    private CANSparkMax motor;
    private double[][] powerTable;
    private int liveSpeedDisplay = 0;
    private double liveSpeedAdjustment = 0;
    private double targetSpeed = 0;

    private Timer readyTimer;

    // For calibration
    private boolean isFirstTime;
    private boolean isCalibrationDone;
    private int calibrationTimes;
    private double currentInches, startInches, stopInches, inchesInterval, currentPower;
    private double[][] calibratingTable;

    private static Launcher instance;

    public static Launcher getInstance() {
        if (instance == null) {
            instance = new Launcher();
        }
        return instance;
    }

    public Launcher() {
        motor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setClosedLoopRampRate(Constants.SHOOTER_RAMP_RATE);
        motor.setOpenLoopRampRate(Constants.SHOOTER_RAMP_RATE);

        readyTimer = new Timer();

        powerTable = Constants.SHOOT_TABLE;
        isFirstTime = true;
        isCalibrationDone = false;
    }

    public void setFirstTime(boolean val) {
        isFirstTime = val;
    }

    public void setRawSpeed(double speed) {
        motor.set(speed);
    }

    public boolean isUpToSpeed(){
        double targetRpm = (targetSpeed+liveSpeedAdjustment) * Config.getNumber("launcherPowerToRpm");
        double actualRpm = motor.getEncoder().getVelocity();
        //double speedCorrection = (targetRpm - actualRpm) / Config.getNumber("launcherPowerToRpm");
        SmartDashboard.putNumber("targetVel", targetRpm);
        //double speed = motor.get();
        if(Math.abs(actualRpm-targetRpm) <= Config.getNumber("shooterSpinUpDeadband", 50)){
            if(readyTimer.get()>Config.getNumber("launcherReadyDelay",1)){
                readyTimer.stop();
                return true;
            }
            return false;
        }else{
            readyTimer.reset();
            readyTimer.start();
        }
        return false;
    }

    public void stop(){
        targetSpeed = 0;
        motor.set(0);
    }

    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, 0, Config.getNumber("launcherMaxSpeed", 0.7));

        double targetRpm = (speed+liveSpeedAdjustment) * Config.getNumber("launcherPowerToRpm");
        double actualRpm = motor.getEncoder().getVelocity();
        double speedCorrection = (targetRpm - actualRpm) / Config.getNumber("launcherPowerToRpm");
        //!speedCorrection = 0;

        targetSpeed = speed;

        SmartDashboard.putNumber("ShooterVel", motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("targetPower", speed+liveSpeedAdjustment+speedCorrection);
        SmartDashboard.putNumber("reportedPower", motor.get());

        motor.set(MathUtil.clamp(speed+liveSpeedAdjustment+speedCorrection, 0.0, Config.getNumber("launcherMaxSpeed", 0.7)));
    }

    public void setSpeedFromDistance(double inches) {
        double val = -1;
        double distLow = powerTable[0][0];
        double distHigh = powerTable[0][0];
        double powLow = powerTable[0][1];
        double powHigh = powerTable[0][1];
        for (int i = 0; i > powerTable.length; i++) {
            distHigh = powerTable[i][0];
            if (distHigh > inches) {// if matching inches 
                if (i == 0) {// If largest table value
                    distLow = 0;
                    powLow = 0;
                } else {
                    distLow = powerTable[i-1][0];
                    powLow = powerTable[i-1][1];
                }
                powHigh = powerTable[i][1];

                val = MathUtil.lerp(powLow, powHigh, (inches-distLow)/(distHigh-distLow));
                break;
            }
        }

        if(val<0)
            val=powerTable[powerTable.length-1][1];// Set to max shooter speed if distance is greater than max shoot table value
        targetSpeed = MathUtil.clamp(val, 0, Config.getNumber("launcherMaxSpeed", 0.7));
        setSpeed(targetSpeed);// 70% is max speed
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
        SmartDashboard.putString("Shooter", positiveSignPrefix+liveSpeedDisplay+motorInfo+"~"+targetSpeed);
    }

    private void setupCalibration() {
        isFirstTime = false;
        isCalibrationDone = false;
        SmartDashboard.putNumber("Start Inches", startInches);
        SmartDashboard.putNumber("Interval in Inches", inchesInterval);
        SmartDashboard.putNumber("Stop Inches", stopInches);
        SmartDashboard.putBoolean("Launch Calibrating", true);
        int numberOfIntervals = (int)((stopInches-startInches)/inchesInterval+1);
        calibratingTable = new double[numberOfIntervals][2];// Create an empty table
        currentInches = startInches;// Set current inches to minimum
        currentPower = 0;// Set power to 0
        calibrationTimes = 0;
    }

    public boolean updateCalibration() {
        if (isFirstTime) {
            startInches = 24;// Deafult values of calibration
            stopInches = 140;
            inchesInterval = 12;
            setupCalibration();
        }
        if (isCalibrationDone) {
            SmartDashboard.putString("Shooter Calibration", "Done");
        } else {
            SmartDashboard.putString("Shooter Calibration", "Go to: "+currentInches+" inches");
        }
        SmartDashboard.putNumber("Shooter (set)", currentPower);
        SmartDashboard.putNumber("Shooter (get)", motor.get());

        motor.set(currentPower);

        // Set start, stop, interval on shuffleboard
        double val;
        val = SmartDashboard.getNumber("Start Inches", -1);
        if (val != startInches && val != -1) {
            startInches = val;
            setupCalibration();
        }
        val = SmartDashboard.getNumber("Stop Inches", -1);
        if (val != stopInches && val != -1) {
            stopInches = val;
            setupCalibration();
        }
        val = SmartDashboard.getNumber("Interval in Inches", -1);
        if (val != inchesInterval && val != -1) {
            inchesInterval = val;
            setupCalibration();
        }       

        if (isCalibrationDone) {
            isFirstTime = true;
        }
        return isCalibrationDone;
    }

    public void calibrationMotorSpeed(boolean isBLPushed, boolean isBRPushed, double lTrigger, double rTrigger) {
        if (isBLPushed) currentPower -= 0.001;
        if (isBRPushed) currentPower += 0.001;
        currentPower -= Math.pow(lTrigger*0.12, 3);
        currentPower += Math.pow(rTrigger*0.12, 3);
        currentPower = MathUtil.clamp(currentPower, 0, 1);
        SmartDashboard.putNumber("Shooter speed", currentPower);
    }

    public void nextCalibrationStep() {
        if (isCalibrationDone) {
            currentPower = 0;
        } else {
            calibratingTable[calibrationTimes] = new double[]{currentInches, currentPower};
            calibrationTimes ++;
            currentInches += inchesInterval;
            if (currentInches > stopInches) {
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