package frc.team3373;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {

    final double poleMotorSpeed = 0.1;
    final double winchMotorSpeed = 0.1;

    CANSparkMax poleMotor, winchMotor;
    CANEncoder poleEncoder, winchEncoder;

    CANPIDController pid;

    private double zeroInchesPos = 0;
    private double extendedInchesPos = 24;
    private double lastPoleGoto = 0;

    private double ticksPerInch = 1;

    private double manualPolemanualPoleSpeed = 0.2;

    private static enum state {
        GOTO, MANUAL, CALIBRATE, STOP
    }
    private static enum calibration_state {
        FAIL_SAFE, INITIALIZE, WAIT_FOR_ZERO_INCHES, WAIT_FOR_25_INCHES
    }

    private state mode = state.GOTO;
    private calibration_state calibrationMode = calibration_state.INITIALIZE;

    public double kp = 0.9;
    public double ki = 0.000011;
    public double kd = 0.0;
    public double kPoleGoto = 0;
    public double kinch = 0;

    public Climber(int poleMotorID, int winchMotorID) {
        if (RobotState.isTest()) {
            SmartDashboard.putNumber("pole motor P", kp);
            SmartDashboard.putNumber("pole motor I", ki);
            SmartDashboard.putNumber("pole motor D", kd);
            SmartDashboard.putNumber("Go to (inches)", kinch);
        }
        
        poleMotor = new CANSparkMax(poleMotorID, MotorType.kBrushless);
        // winchMotor = new CANSparkMax(winchMotorID, MotorType.kBrushed);
        poleEncoder = new CANEncoder(poleMotor);

        poleMotor.setInverted(true);
        poleEncoder.setPositionConversionFactor(1);
        poleEncoder.setPosition(0);
        poleMotor.setIdleMode(IdleMode.kBrake);
        // winchEncoder = new CANEncoder(winchMotor);

        pid = poleMotor.getPIDController();
        pid.setP(kp);
        pid.setI(ki);
        pid.setD(kd);
        pid.setOutputRange(-poleMotorSpeed, poleMotorSpeed);
        pid.setReference(0, ControlType.kPosition);

    }

    public void setGotoInches(double inches) {
        kPoleGoto = (inches-zeroInchesPos)*ticksPerInch;
        pid.setReference(kPoleGoto, ControlType.kPosition);
    }

    public void setZeroInches(double pos) {
        zeroInchesPos = pos;
    }

    public void setExtenedInches(double pos, double heightInInches) {
        ticksPerInch = (pos - zeroInchesPos)/heightInInches;
        extendedInchesPos = zeroInchesPos + ticksPerInch*25;
    }

    public void onDPadUp() {
        mode = state.GOTO;
        pid.setReference(extendedInchesPos, ControlType.kPosition);

    }

    public void onDPadDown() {
        mode = state.GOTO;
        pid.setReference(zeroInchesPos, ControlType.kPosition);
    }

    public void onYStick(double value) {
        // Switch from GOTO to MANUAL mode
        if (Math.abs(value) > 0.05) {
            if (mode == state.GOTO) {
                mode = state.MANUAL;
            }
        }

        // Manually control motor
        if (mode == state.MANUAL || mode == state.CALIBRATE) {
            if (Math.abs(value) > 0.05) {
                kPoleGoto += value*manualPolemanualPoleSpeed;
            }
        }
    }

    public void onCalibrateButton() {
        if (mode != state.CALIBRATE) {//? Always 
            mode = state.CALIBRATE;
            calibrationMode = calibration_state.INITIALIZE;
        }
        if (mode == state.CALIBRATE) {
            switch (calibrationMode) {
                case FAIL_SAFE:
                    SmartDashboard.putString("Calibration", "Error");
                    break;
                case INITIALIZE:
                    SmartDashboard.putString("Calibration", "Waiting for 0 inches");
                    calibrationMode = calibration_state.WAIT_FOR_ZERO_INCHES;
                    break;
                case WAIT_FOR_ZERO_INCHES:
                    setZeroInches(kPoleGoto);
                    SmartDashboard.putString("Calibration", "Waiting for 25 inches");
                    calibrationMode = calibration_state.WAIT_FOR_25_INCHES;
                    break;
                case WAIT_FOR_25_INCHES:
                    setExtenedInches(kPoleGoto, 25.0);
                    SmartDashboard.putString("Calibration", "Done");
                    calibrationMode = calibration_state.FAIL_SAFE;
                    mode = state.GOTO;
                    onDPadUp();
                    break;
            }
        }
 
    }

    public void update() {
        if (mode == state.MANUAL || mode == state.CALIBRATE) {
       
            if (lastPoleGoto != kPoleGoto) {
                pid.setReference(kPoleGoto, ControlType.kPosition);
            }
            lastPoleGoto = kPoleGoto;
        } else if (mode == state.GOTO) {
            kPoleGoto = poleEncoder.getPosition();
        }

        if (RobotState.isTest())
            displayOnShuffleboard();
 
    }

    public void displayOnShuffleboard() {
        SmartDashboard.putNumber("Pole pos", poleEncoder.getPosition());
        SmartDashboard.putNumber("Goto pos", kPoleGoto);
        SmartDashboard.putString("Climber mode", mode.toString());

        double p = SmartDashboard.getNumber("pole motor P", 0);
        double i = SmartDashboard.getNumber("pole motor I", 0);
        double d = SmartDashboard.getNumber("pole motor D", 0);

        double inch = SmartDashboard.getNumber("Go to (inches)", 0);

        if (kp != p) {
            kp = p;
            pid.setP(p);
        }
        if (ki != i) {
            ki = i;
            pid.setI(i);
        }
        if (kd != d) {
            kd = d;
            pid.setD(d);
        }
        if (kinch != inch) {
            kinch = inch;
            setGotoInches(inch);
        }
    }
}