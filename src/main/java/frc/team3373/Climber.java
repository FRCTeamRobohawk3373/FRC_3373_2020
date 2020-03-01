package frc.team3373;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    final double START_POLE_ZERO_POSITION = 0;
    final double START_WINCH_ZERO_POSITION = 0;
    final double START_POLE_TICKS_PER_INCH = 0.278;
    final double START_WINCH_TICKS_PER_INCH = 0.153;

    final double ALL_MODES_MAX_SPEED = 0.9;
    final double MAX_MANUAL_CONTROL_SPEED = 0.9;
    final double MAX_CALIBRATE_CONTROL_SPEED = 0.9;
    final double POLE_MOTOR_EXTENDED_INCHES = 24;//! For calibration
    final double WINCH_MOTOR_EXTENDED_INCHES = 24;//!

    final double LOW_POSITION_INCHES = 16;
    final double MIDDLE_POSITION_INCHES = 20;
    final double HIGH_POSITION_INCHES = 24;
    final double CLIMB_POSITION_INCHES = 0;
    final double SOLENOID_POSITION_INCHES = -1;
    final double WINCH_CLIMB_INCHES = -2;
    final double INCLINE_MOTOR_SPEED = 0.9;

    private double p_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
    private double p_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
    private double w_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
    private double w_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;

    CANSparkMax poleMotor, winchMotor, inclineMotor;
    CANEncoder poleEncoder, winchEncoder;

    Solenoid poleSolenoid, winchSolenoid;

    CANPIDController p_pid, w_pid;

    private double p_ticksPerInch = 1;
    private double w_ticksPerInch = 1;

    private boolean isCalibrating = false;

    private static enum climber_state {
        INIT, GOTO_SOLENOID, POGO, CLIMB
    }
    private climber_state climberMode = climber_state.INIT;

    private static enum calibration_state {
        FAIL_SAFE, WAITING_FOR_A_OR_B,
        A_WAIT_FOR_ZERO_INCHES,
        B_WAIT_FOR_ZERO_INCHES, B_WAIT_FOR_EXTENDED_INCHES
    }
    private calibration_state calibrationMode = calibration_state.FAIL_SAFE;

    private double p_p = Constants.C_PP;
    private double p_i = Constants.C_PI;
    private double p_d = Constants.C_PD;
    private double p_goto = 0;
    private double p_inch = 0;

    private double w_p = Constants.C_WP;
    private double w_i = Constants.C_WI;//0.000011;
    private double w_d = Constants.C_WD;
    private double w_goto = 0;

    private double p_zeroInchesPos;
    private double w_zeroInchesPos;

    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {    
        
        //public static Climber 
        poleMotor = new CANSparkMax(Constants.POLE_MOTOR_ID, MotorType.kBrushless);
        poleMotor.setInverted(false);
        poleMotor.setIdleMode(IdleMode.kBrake);

        poleEncoder = new CANEncoder(poleMotor);
        
        poleSolenoid = new Solenoid(Constants.PCM_ID, Constants.POLE_SOLENOID_ID);
        poleSolenoid.set(true);

        winchMotor = new CANSparkMax(Constants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchMotor.setInverted(false);        
        winchMotor.setIdleMode(IdleMode.kBrake);

        winchEncoder = new CANEncoder(winchMotor);

        winchSolenoid = new Solenoid(Constants.PCM_ID, Constants.WINCH_SOLENOID_ID);
        winchSolenoid.set(true);

        //inclineMotor = new CANSparkMax(Constants.INCLINE_MOTOR_ID, MotorType.kBrushed);
    
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

        setPoleZeroInches(START_POLE_ZERO_POSITION);
        setPoleExtenedInches(START_POLE_ZERO_POSITION+START_POLE_TICKS_PER_INCH, 1);
        setPoleGotoInches(0);

        setWinchZeroInches(START_WINCH_ZERO_POSITION);
        setWinchExtenedInches(START_WINCH_ZERO_POSITION+START_WINCH_TICKS_PER_INCH, 1);
        setWinchGotoInches(0);

        adjustManualAndCalibrateSpeeds();

        SmartDashboard.putNumber("Go to (inches)", 0);
    }

    public void setMotorsToZero() {
        poleEncoder.setPosition(0);
        winchEncoder.setPosition(0);

    }

    public boolean getCalibrating() {
        return isCalibrating;
    }

    public void setPoleGotoInches(double inches) {
        p_goto = poleInchesToPos(inches);
        p_pid.setReference(p_goto, ControlType.kPosition);
    }

    public void setWinchGotoInches(double inches) {
        w_goto = winchInchesToPos(inches);
        w_pid.setReference(w_goto, ControlType.kPosition);
    }

    public void gotoLowPosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(LOW_POSITION_INCHES);
            setWinchGotoInches(LOW_POSITION_INCHES);
        }
    }
    public void gotoMiddlePosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(MIDDLE_POSITION_INCHES);
            setWinchGotoInches(MIDDLE_POSITION_INCHES);
        }
    }
    public void gotoHighPosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(HIGH_POSITION_INCHES);
            setWinchGotoInches(HIGH_POSITION_INCHES);
        }
    }

    private void setPoleZeroInches(double pos) {
        p_zeroInchesPos = pos;
    }
    private void setWinchZeroInches(double pos) {
        w_zeroInchesPos = pos;
    }
    private double polePosToInches(double pos) {
        return (pos-p_zeroInchesPos)/p_ticksPerInch;
    }
    private double winchPosToInches(double pos) {
        return (pos-w_zeroInchesPos)/w_ticksPerInch;
    }
    private double poleInchesToPos(double inches) {
        return inches*p_ticksPerInch+p_zeroInchesPos;
    }
    private double winchInchesToPos(double inches) {
        return inches*w_ticksPerInch+w_zeroInchesPos;
    }
    private void setPoleExtenedInches(double pos, double heightInInches) {
        p_ticksPerInch = (pos - p_zeroInchesPos)/heightInInches;
    }
    private void setWinchExtenedInches(double pos, double heightInInches) {
        w_ticksPerInch = (pos - w_zeroInchesPos)/heightInInches;
    }

    public void xStickManual(double value) {
        if (climberMode == climber_state.CLIMB) {
            ////inclineMotor.set(INCLINE_MOTOR_SPEED);
        }
    }

    public void yStickManual(double value) {
        if (climberMode == climber_state.POGO) {
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
            p_pid.setReference(p_goto, ControlType.kPosition);
            w_pid.setReference(w_goto, ControlType.kPosition);
        } else if (climberMode == climber_state.CLIMB) {
            w_goto += Math.min(0, value*w_manualSpeed);
            if (winchPosToInches(w_goto) < WINCH_CLIMB_INCHES) {
                w_goto = winchInchesToPos(WINCH_CLIMB_INCHES);
            }
            w_pid.setReference(w_goto, ControlType.kPosition);
        }
    }

    public void yStick1Calibrate(double value) {
        p_goto += value*p_calibrateSpeed;
        p_pid.setReference(p_goto, ControlType.kPosition);
    }

    
    public void yStick2Calibrate(double value) {
        w_goto += value*w_calibrateSpeed;
        w_pid.setReference(w_goto, ControlType.kPosition);
    }

    private void adjustManualAndCalibrateSpeeds() {
        double ratio = p_ticksPerInch/w_ticksPerInch;
        if (ratio > 1) {
            p_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
            p_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
            w_manualSpeed = MAX_MANUAL_CONTROL_SPEED / ratio;
            w_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
        } else {
            p_manualSpeed = MAX_MANUAL_CONTROL_SPEED * ratio;
            p_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
            w_manualSpeed = MAX_MANUAL_CONTROL_SPEED;
            w_calibrateSpeed = MAX_CALIBRATE_CONTROL_SPEED;
        }
    }   

    public void startCalibrateOptions() {
        if (isCalibrating) {
            isCalibrating = false;
            SmartDashboard.putString("Calibration", "Calibration aborted");
        } else{

            if (climberMode == climber_state.POGO) { 
                isCalibrating = true;
                calibrationMode = calibration_state.WAITING_FOR_A_OR_B;
                SmartDashboard.putString("Calibration", "Waiting for A or B option.");
            } else {
                SmartDashboard.putString("Calibration", "Robot not in POGO mode!");
            }
        }
    }

    public void calibrateHeights() {
        switch(calibrationMode) {
            case WAITING_FOR_A_OR_B:
                SmartDashboard.putString("Calibration", "[A] Waiting for 0 inches");
                calibrationMode = calibration_state.A_WAIT_FOR_ZERO_INCHES;
                break;
            case A_WAIT_FOR_ZERO_INCHES:
                SmartDashboard.putString("Calibration", "[A] Done");
                isCalibrating = false;
                setPoleGotoInches(0);
                setWinchGotoInches(0);
                break;
        }
    }

    public void calibrateInches() {
        switch(calibrationMode) {
            case WAITING_FOR_A_OR_B:
                SmartDashboard.putString("Calibration", "[B] Waiting for 0 inches");
                calibrationMode = calibration_state.B_WAIT_FOR_ZERO_INCHES;
                break;
            case B_WAIT_FOR_ZERO_INCHES:
                SmartDashboard.putString("Calibration", "[B] Waiting for "+POLE_MOTOR_EXTENDED_INCHES+" inches");
                setPoleZeroInches(p_goto);
                setWinchZeroInches(w_goto);
                calibrationMode = calibration_state.B_WAIT_FOR_EXTENDED_INCHES;
                break;
            case B_WAIT_FOR_EXTENDED_INCHES:
                setPoleExtenedInches(p_goto, POLE_MOTOR_EXTENDED_INCHES);
                setWinchExtenedInches(w_goto, WINCH_MOTOR_EXTENDED_INCHES);
                SmartDashboard.putString("Calibration", "[B] Done");
                calibrationMode = calibration_state.FAIL_SAFE;
                adjustManualAndCalibrateSpeeds();
                isCalibrating = false;
                break;
        }
    }

    public void initiateClimbMode() {
        if (climberMode == climber_state.INIT) {
            poleSolenoid.set(true);
            winchSolenoid.set(true);
            if (RobotState.isTest()) {// If in test mode, have option to go to position in inches from ShuffleBoard
                SmartDashboard.putNumber("Go to (inches)", 0.0001);
                climberMode = climber_state.POGO;
            } else {
            climberMode = climber_state.GOTO_SOLENOID;
            }
        } else if (climberMode == climber_state.POGO) {
            poleSolenoid.set(false);
            winchSolenoid.set(false);
            setPoleGotoInches(CLIMB_POSITION_INCHES);
            climberMode = climber_state.CLIMB;
        }
    }

    public void update() {
        switch (climberMode) {
            case INIT:
                setPoleGotoInches(START_POLE_ZERO_POSITION);
                setWinchGotoInches(START_WINCH_ZERO_POSITION);
                break;
            
            case GOTO_SOLENOID:
                setPoleGotoInches(START_POLE_ZERO_POSITION+SOLENOID_POSITION_INCHES);
                setWinchGotoInches(START_WINCH_ZERO_POSITION+SOLENOID_POSITION_INCHES);
                if (Math.abs(polePosToInches(poleEncoder.getPosition())   - (START_POLE_ZERO_POSITION+SOLENOID_POSITION_INCHES)) < 0.1 &&
                    Math.abs(winchPosToInches(winchEncoder.getPosition()) - (START_WINCH_ZERO_POSITION+SOLENOID_POSITION_INCHES)) < 0.1) {
                    climberMode = climber_state.POGO;
                }
                break;

            case POGO:
                break;

            case CLIMB:
                break;
        }
        SmartDashboard.putString("Climb Mode", climberMode.toString());
    }


    public void displayOnShuffleboard() {

        /* Pole motor */
        //!SmartDashboard.putNumber("pole now", polePosToInches(poleEncoder.getPosition()));
        //!SmartDashboard.putNumber("pole goto", polePosToInches(p_goto));
        //!double p = SmartDashboard.getNumber("pole motor P", p_p);
        //!double i = SmartDashboard.getNumber("pole motor I", p_i);
        //!double d = SmartDashboard.getNumber("pole motor D", p_d);
        double inch = SmartDashboard.getNumber("Go to (inches)", p_inch);
        /*if (p_p != p) {
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
        }*/
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
            setWinchGotoInches(inch);
        }

        SmartDashboard.putNumber("pole zero position", p_zeroInchesPos);
        SmartDashboard.putNumber("winch zero position", w_zeroInchesPos);
        SmartDashboard.putNumber("pole ticks/inch", p_ticksPerInch);
        SmartDashboard.putNumber("winch ticks/inch", w_ticksPerInch);

        /* Winch motor */
        //!SmartDashboard.putNumber("winch now", winchPosToInches(winchEncoder.getPosition()));
        //!SmartDashboard.putNumber("winch goto", winchPosToInches(w_goto));
        //!p = SmartDashboard.getNumber("winch motor P", w_p);
        //!i = SmartDashboard.getNumber("winch motor I", w_i);
        //!d = SmartDashboard.getNumber("winch motor D", w_d);
        /*if (w_p != p) {
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
        }*/
    }

    public void disable() {
        climberMode = climber_state.INIT;
    }
}