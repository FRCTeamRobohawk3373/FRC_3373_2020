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

    final double START_POLE_TICKS_PER_INCH = 0.28;
    final double START_WINCH_TICKS_PER_INCH = 0.155033;

    final double ALL_MODES_MAX_SPEED = 0.08;
    final double MAX_MANUAL_CONTROL_SPEED = 0.07;
    final double MAX_CALIBRATE_CONTROL_SPEED = 0.06;
    final double POLE_MOTOR_EXTENDED_INCHES = 25;//! For calibration
    final double WINCH_MOTOR_EXTENDED_INCHES = 16;//!

    final double LOW_POSITION_INCHES = 15;
    final double MIDDLE_POSITION_INCHES = 21;
    final double HIGH_POSITION_INCHES = 27;
    final double CLIMB_POSITION_INCHES = -2;

    private double p_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
    private double p_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
    private double w_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
    private double w_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;

    CANSparkMax poleMotor, winchMotor;
    CANEncoder poleEncoder, winchEncoder;

    CANPIDController p_pid, w_pid;

    private double p_zeroInchesPos;
    private double w_zeroInchesPos;
    private double p_lastGoto = 0;
    private double w_lastGoto = 0;

    private double p_extendedInchesPos = 0;
    private double w_extendedInchesPos = 0;

    private double p_ticksPerInch = 1;
    private double w_ticksPerInch = 1;

    private static enum motor_state {
        GOTO, MANUAL, CALIBRATE, STOP
    }
    private static enum calibration_state {
        FAIL_SAFE, WAITING_FOR_A_OR_B, 
        a_P_WAIT_FOR_ZERO_INCHES, a_W_WAIT_FOR_ZERO_INCHES,
        b_P_WAIT_FOR_ZERO_INCHES, b_P_WAIT_FOR_EXTENDED_INCHES, b_W_WAIT_FOR_ZERO_INCHES, b_W_WAIT_FOR_EXTENDED_INCHES 
    }

    private motor_state mode = motor_state.GOTO;
    private calibration_state calibrationMode = calibration_state.FAIL_SAFE;

    private double p_p = 0.9;
    private double p_i = 0;
    private double p_d = 0.0;
    private double p_goto = 0;
    private double p_inch = 0;

    private double w_p = 0.9;
    private double w_i = 0;//0.000011;
    private double w_d = 0.0;
    private double w_goto = 0;
    private double w_inch = 0;

    public Climber(int poleMotorID, int winchMotorID) {
        
        poleMotor = new CANSparkMax(poleMotorID, MotorType.kBrushless);
        
        poleEncoder = new CANEncoder(poleMotor);
        poleMotor.setInverted(true);
        poleEncoder.setPosition(0);
        poleMotor.setIdleMode(IdleMode.kBrake);

        winchMotor = new CANSparkMax(winchMotorID, MotorType.kBrushless);
        winchEncoder = new CANEncoder(winchMotor);
        winchMotor.setInverted(true);
        winchEncoder.setPosition(0);
        winchMotor.setIdleMode(IdleMode.kBrake);


        p_pid = poleMotor.getPIDController();
        p_pid.setP(p_p);
        p_pid.setI(p_i);
        p_pid.setD(p_d);
        p_pid.setOutputRange(-ALL_MODES_MAX_SPEED, ALL_MODES_MAX_SPEED);
        p_pid.setReference(0, ControlType.kPosition);

        w_pid = winchMotor.getPIDController();
        w_pid.setP(w_p);
        w_pid.setI(w_i);
        w_pid.setD(w_d);
        w_pid.setOutputRange(-ALL_MODES_MAX_SPEED, ALL_MODES_MAX_SPEED);
        w_pid.setReference(0, ControlType.kPosition);

        
        setPoleZeroInches(0);
        setPoleExtenedInches(START_POLE_TICKS_PER_INCH, 1);
        setPoleGotoInches(0);

        setWinchZeroInches(0);
        setWinchExtenedInches(START_WINCH_TICKS_PER_INCH, 1);
        setWinchGotoInches(0);

        smartAdjustManualAndCalibrateSpeeds();

        if (RobotState.isTest()) {
            SmartDashboard.putNumber("Go to (inches)", 0);

            SmartDashboard.putNumber("pole now", polePosToInches(poleEncoder.getPosition()));
            SmartDashboard.putNumber("pole goto", polePosToInches(p_goto));
            SmartDashboard.putNumber("pole motor P", p_p);
            SmartDashboard.putNumber("pole motor I", p_i);
            SmartDashboard.putNumber("pole motor D", p_d);
            
            SmartDashboard.putNumber("winch now", winchPosToInches(winchEncoder.getPosition()));
            SmartDashboard.putNumber("winch goto", winchPosToInches(w_goto));
            SmartDashboard.putNumber("winch motor P", w_p);
            SmartDashboard.putNumber("winch motor I", w_i);
            SmartDashboard.putNumber("winch motor D", w_d);
        }

    }

    public boolean isCalibrating() {
        return mode == motor_state.CALIBRATE;
    }

    public void setPoleGotoInches(double inches) {
        p_goto = inches*p_ticksPerInch+p_zeroInchesPos;
        p_pid.setReference(p_goto, ControlType.kPosition);
    }

    public void setWinchGotoInches(double inches) {
        w_goto = inches*w_ticksPerInch+w_zeroInchesPos;
        w_pid.setReference(w_goto, ControlType.kPosition);
    }

    public void gotoLowPosition() {
        setPoleGotoInches(LOW_POSITION_INCHES);
        setWinchGotoInches(LOW_POSITION_INCHES);
    }
    public void gotoMiddlePosition() {
        setPoleGotoInches(MIDDLE_POSITION_INCHES);
        setWinchGotoInches(MIDDLE_POSITION_INCHES);
    }
    public void gotoHighPosition() {
        setPoleGotoInches(HIGH_POSITION_INCHES);
        setWinchGotoInches(HIGH_POSITION_INCHES);
    }

    public void climb() {
        setPoleGotoInches(CLIMB_POSITION_INCHES);
        setWinchGotoInches(CLIMB_POSITION_INCHES);
    }

    public void setPoleZeroInches(double pos) {
        p_zeroInchesPos = pos;
    }

    public void setWinchZeroInches(double pos) {
        w_zeroInchesPos = pos;
    }

    public double polePosToInches(double pos) {
        return (pos-p_zeroInchesPos)/p_ticksPerInch;
    }

    public double winchPosToInches(double pos) {
        return (pos-w_zeroInchesPos)/w_ticksPerInch;
    }

    public double poleInchesToPos(double inches) {
        return inches*p_ticksPerInch+p_zeroInchesPos;
    }

    public double winchInchesToPos(double inches) {
        return inches*w_ticksPerInch+w_zeroInchesPos;
    }

    public void setPoleExtenedInches(double pos, double heightInInches) {
        p_ticksPerInch = (pos - p_zeroInchesPos)/heightInInches;
        p_extendedInchesPos = p_zeroInchesPos + p_ticksPerInch*POLE_MOTOR_EXTENDED_INCHES;
    }

    public void setWinchExtenedInches(double pos, double heightInInches) {
        w_ticksPerInch = (pos - w_zeroInchesPos)/heightInInches;
        w_extendedInchesPos = w_zeroInchesPos + w_ticksPerInch*WINCH_MOTOR_EXTENDED_INCHES;
    }

    public void onYStick(double value) {
        switch(mode) {
            case GOTO:
                mode = motor_state.MANUAL;
                break;

            case MANUAL:
                p_goto += value*p_manualSpeed;
                if (polePosToInches(p_goto) < 0) {
                    p_goto = poleInchesToPos(0);
                } else if (polePosToInches(p_goto) > POLE_MOTOR_EXTENDED_INCHES) {
                    p_goto = poleInchesToPos(POLE_MOTOR_EXTENDED_INCHES);
                }
                w_goto += value*w_manualSpeed;
                if (winchPosToInches(w_goto) < 0) {
                    w_goto = winchInchesToPos(0);
                } else if (winchPosToInches(w_goto) > WINCH_MOTOR_EXTENDED_INCHES) {
                    w_goto = winchInchesToPos(WINCH_MOTOR_EXTENDED_INCHES);
                }
                break;
            
            case CALIBRATE:

                if (
calibrationMode == calibration_state.a_P_WAIT_FOR_ZERO_INCHES ||
calibrationMode == calibration_state.b_P_WAIT_FOR_ZERO_INCHES ||
calibrationMode == calibration_state.b_P_WAIT_FOR_EXTENDED_INCHES) p_goto += value*p_calibrateSpeed;

                if (
calibrationMode == calibration_state.a_W_WAIT_FOR_ZERO_INCHES ||
calibrationMode == calibration_state.b_W_WAIT_FOR_ZERO_INCHES ||
calibrationMode == calibration_state.b_W_WAIT_FOR_EXTENDED_INCHES) w_goto += value*w_calibrateSpeed;

                break;
        }
    }
    

    public void smartAdjustManualAndCalibrateSpeeds() {
        double ratio = p_ticksPerInch/w_ticksPerInch;
        if (ratio > 1) {
            p_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
            p_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
            w_manualSpeed = MAX_MANUAL_CONTROL_SPEED / ratio;
            w_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED / ratio;
        } else {
            p_manualSpeed = MAX_MANUAL_CONTROL_SPEED * ratio;
            p_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED * ratio;
            w_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
            w_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
        }
    }   

    public void onCalibrateButton() {
        if (mode == motor_state.CALIBRATE) {
            SmartDashboard.putString("Calibration", "Calibration aborted");
        } else { 
            mode = motor_state.CALIBRATE;
            calibrationMode = calibration_state.WAITING_FOR_A_OR_B;
            SmartDashboard.putString("Calibration", "Waiting for A or B config.")
        } 

    }

    public void onAButton() {
        switch(calibrationMode) {

            case WAITING_FOR_A_OR_B:
                SmartDashboard.putString("Calibration", "a-[P] Waiting for 0 inches");
                calibrationMode = calibration_state.a_P_WAIT_FOR_ZERO_INCHES;
                break;
            case a_P_WAIT_FOR_ZERO_INCHES:
                setPoleZeroInches(p_goto);
                SmartDashboard.putString("Calibration", "a-[P] Waiting for 0 inches");
                calibrationMode = calibration_state.a_W_WAIT_FOR_ZERO_INCHES;
                break;
            case a_W_WAIT_FOR_ZERO_INCHES:
                setWinchZeroInches(w_goto);
                SmartDashboard.putString("Calibration", "a-Calibration done");
                calibrationMode = calibration_state.FAIL_SAFE;
                mode = motor_state.GOTO;
                setPoleGotoInches(0);
                setWinchGotoInches(0);
                break;
        }
    }

    public void onBButton() {
        switch(calibrationMode) {

            case WAITING_FOR_A_OR_B:
                SmartDashboard.putString("Calibration", "[P] Waiting for 0 inches");
                calibrationMode = calibration_state.b_P_WAIT_FOR_ZERO_INCHES;
                break;
            case b_P_WAIT_FOR_ZERO_INCHES:
                setPoleZeroInches(p_goto);
                SmartDashboard.putString("Calibration", "b-[P] Waiting for "+POLE_MOTOR_EXTENDED_INCHES+" inches");
                calibrationMode = calibration_state.b_P_WAIT_FOR_EXTENDED_INCHES;
                break;
            case b_P_WAIT_FOR_EXTENDED_INCHES:
                setPoleExtenedInches(p_goto, POLE_MOTOR_EXTENDED_INCHES);
                SmartDashboard.putString("Calibration", "b-[W] Waiting for 0 inches");
                calibrationMode = calibration_state.b_W_WAIT_FOR_ZERO_INCHES;
                break;
            case b_W_WAIT_FOR_ZERO_INCHES:
                setWinchZeroInches(w_goto);
                SmartDashboard.putString("Calibration", "b-[W] Waiting for "+WINCH_MOTOR_EXTENDED_INCHES+" inches");
                calibrationMode = calibration_state.b_W_WAIT_FOR_EXTENDED_INCHES;
                break;
            case b_W_WAIT_FOR_EXTENDED_INCHES:
                setWinchExtenedInches(w_goto, WINCH_MOTOR_EXTENDED_INCHES);
                SmartDashboard.putString("Calibration", "b-Calibration done");
                calibrationMode = calibration_state.FAIL_SAFE;
                smartAdjustManualAndCalibrateSpeeds();
                mode = motor_state.GOTO;
                setPoleGotoInches(0);
                setWinchGotoInches(0);
                break;
        }
    }

    public void interruptCalibration() {
    }

    public void updateTeleOp() {

    }

    public void updateTestMode() {
        if (mode == motor_state.MANUAL || mode == motor_state.CALIBRATE) {
            /* Manually control pole motor */
           if (calibrationMode != calibration_state.a_W_WAIT_FOR_ZERO_INCHES && calibrationMode != calibration_state.b_W_WAIT_FOR_ZERO_INCHES && calibrationMode != calibration_state.b_W_WAIT_FOR_EXTENDED_INCHES) {
                if (p_lastGoto != p_goto) {
                    p_pid.setReference(p_goto, ControlType.kPosition);
                }
            }

            /* Manually control winch motor */
            if (calibrationMode != calibration_state.a_P_WAIT_FOR_ZERO_INCHES && calibrationMode != calibration_state.b_P_WAIT_FOR_ZERO_INCHES && calibrationMode != calibration_state.b_P_WAIT_FOR_EXTENDED_INCHES) {
                if (w_lastGoto != w_goto) {
                    w_pid.setReference(w_goto, ControlType.kPosition);
                }  
            }

            p_lastGoto = p_goto;
            w_lastGoto = w_goto;
        } else if (mode == motor_state.GOTO) {
            p_goto = poleEncoder.getPosition();/// really? why not just leave this out
            w_goto = winchEncoder.getPosition();
        }

        displayOnShuffleboard();

    }

    public void displayOnShuffleboard() {
        SmartDashboard.putString("Climber mode", mode.toString());

        /* Pole motor */
        SmartDashboard.putNumber("pole now", polePosToInches(poleEncoder.getPosition()));
        SmartDashboard.putNumber("pole goto", winchPosToInches(p_goto));
        double p = SmartDashboard.getNumber("pole motor P", 0);
        double i = SmartDashboard.getNumber("pole motor I", 0);
        double d = SmartDashboard.getNumber("pole motor D", 0);
        double inch = SmartDashboard.getNumber("Go to (inches)", 0);
        if (p_p != p) {
            p_p = p;
            p_pid.setP(p);
        }
        if (p_i != i) {
            p_i = i;
            p_pid.setI(i);
        }
        if (p_d != d) {
            p_d = d;
            p_pid.setD(d);
        }
        if (p_inch != inch) {
            if (inch < 0) {
                inch = 0;
                SmartDashboard.putNumber("Go to (inches)", 0);
            } else if (inch > POLE_MOTOR_EXTENDED_INCHES) {
                inch = POLE_MOTOR_EXTENDED_INCHES;
                SmartDashboard.putNumber("Go to (inches)", POLE_MOTOR_EXTENDED_INCHES);
            }
            p_inch = inch;
            setPoleGotoInches(inch);
        }
    

        //! delete this code when done??? maybe not
        SmartDashboard.putNumber("pole ticks/inch", p_ticksPerInch);
        SmartDashboard.putNumber("winch ticks/inch", w_ticksPerInch);


        /* Winch motor */
        SmartDashboard.putNumber("winch now", winchPosToInches(winchEncoder.getPosition()));
        SmartDashboard.putNumber("winch goto", winchPosToInches(w_goto));
        p = SmartDashboard.getNumber("winch motor P", 0);
        i = SmartDashboard.getNumber("winch motor I", 0);
        d = SmartDashboard.getNumber("winch motor D", 0);
        inch = SmartDashboard.getNumber("Go to (inches)", 0);
        if (w_p != p) {
            w_p = p;
            w_pid.setP(p);
        }
        if (w_i != i) {
            w_i = i;
            w_pid.setI(i);
        }
        if (w_d != d) {
            w_d = d;
            w_pid.setD(d);
        }
        if (w_inch != inch) {
            if (inch < 0) {
                inch = 0;
                SmartDashboard.putNumber("Go to (inches)", 0);
            } else if (inch > WINCH_MOTOR_EXTENDED_INCHES) {
                inch = WINCH_MOTOR_EXTENDED_INCHES;
                SmartDashboard.putNumber("Go to (inches)", WINCH_MOTOR_EXTENDED_INCHES);
            }
            w_inch = inch;
            setWinchGotoInches(inch);
        }
    }
}