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

    private double p_manualSpeed = Config.getNumber("maxManualControlSpeed", 0.9);
    private double p_calibrateSpeed = Config.getNumber("maxCalibrateControlSpeed", 0.9);
    private double w_manualSpeed = Config.getNumber("maxManualControlSpeed", 0.9);
    private double w_calibrateSpeed = Config.getNumber("maxCalibrateControlSpeed", 0.9);

    CANSparkMax poleMotor, winchMotor, inclineMotor;
    CANEncoder poleEncoder, winchEncoder;

    Solenoid poleSolenoid, winchSolenoid;

    CANPIDController p_pid, w_pid;

    /* For calibration */
    private double p_zeroPosition = Config.getNumber("startPoleZeroPosition", 0);
    private double w_zeroPosition = Config.getNumber("startWinchZeroPosition", 0);
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
        
        //solenoids
        poleSolenoid = new Solenoid(Constants.PCM_ID, Constants.POLE_SOLENOID_ID);
        poleSolenoid.set(true);

        winchSolenoid = new Solenoid(Constants.PCM_ID, Constants.WINCH_SOLENOID_ID);
        winchSolenoid.set(true);
        
        //motor setup
        poleMotor = new CANSparkMax(Constants.POLE_MOTOR_ID, MotorType.kBrushless);
        poleMotor.setInverted(false);
        poleMotor.setIdleMode(IdleMode.kBrake);

        poleEncoder = new CANEncoder(poleMotor);
        poleEncoder.setPosition(0);
        
        winchMotor = new CANSparkMax(Constants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchMotor.setInverted(false);        
        winchMotor.setIdleMode(IdleMode.kBrake);

        winchEncoder = new CANEncoder(winchMotor);
        winchEncoder.setPosition(0);
        
        inclineMotor = new CANSparkMax(Constants.INCLINE_MOTOR_ID, MotorType.kBrushless);
    
        p_pid = poleMotor.getPIDController();
        p_pid.setP(p_p);
        p_pid.setI(p_i);
        p_pid.setD(p_d);
        p_pid.setOutputRange(-Config.getNumber("allModesMaxSpeed", 0.9), Config.getNumber("allModesMaxSpeed", 0.9));
        p_pid.setReference(0, ControlType.kPosition);

        w_pid = winchMotor.getPIDController();
        w_pid.setP(w_p);
        w_pid.setI(w_i);
        w_pid.setD(w_d);
        w_pid.setOutputRange(-Config.getNumber("allModesMaxSpeed", 0.9), Config.getNumber("allModesMaxSpeed", 0.9));
        w_pid.setReference(0, ControlType.kPosition);

        setPoleZeroInches(p_zeroPosition);
        setPoleExtenedInches(p_zeroPosition+Config.getNumber("poleTicksPerInch", 1), 1);
        setPoleGotoInches(0);

        setWinchZeroInches(w_zeroPosition);
        setWinchExtenedInches(w_zeroPosition+Config.getNumber("winchTicksPerInch", 1), 1);
        setWinchGotoInches(0);

        adjustManualAndCalibrateSpeeds();

        SmartDashboard.putNumber("Go to (inches)", 0);
    }

    /* public void setMotorsToZero() {
        poleEncoder.setPosition(0);
        winchEncoder.setPosition(0);

    } */

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
            setPoleGotoInches(Config.getNumber("climbLowInches", 16));
            setWinchGotoInches(Config.getNumber("climbLowInches", 16));
        }
    }
    public void gotoMiddlePosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(Config.getNumber("climbMiddleInches", 20));
            setWinchGotoInches(Config.getNumber("climbMiddleInches", 20));
        }
    }
    public void gotoHighPosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(Config.getNumber("climbHighInches", 24));
            setWinchGotoInches(Config.getNumber("climbHighInches", 24));
        }
    }

    //###################################
    //##         Robot Methods         ##
    //###################################

    /**
     * Controller inputs
     * @param ly Left joystick Y input
     * @param lx Left joystick X input
     */
    public void teleOpControl(double ly, double lx) {
        if (Math.abs(lx) > 0.1) {
            xStickManual(lx);
        } else {
            xStickManual(0);
        }
        if (Math.abs(ly) > 0.05) {
            yStickManual(ly);
        }
    }


    public void xStickManual(double value) {
        if (climberMode == climber_state.CLIMB) {
            inclineMotor.set(value*Config.getNumber("inclineMotorSpeed", 0.9));
        }
    }

    public void yStickManual(double value) {
        if (climberMode == climber_state.POGO) {
            p_goto += value*p_manualSpeed;
            if (polePosToInches(p_goto) < 0) {
                p_goto = poleInchesToPos(0);
            } else if (polePosToInches(p_goto) > Config.getNumber("calibratePoleMotorExtendedInches", 24)) {
                p_goto = poleInchesToPos(Config.getNumber("calibratePoleMotorExtendedInches", 24));
            }
            w_goto += value*w_manualSpeed;
            if (winchPosToInches(w_goto) < 0) {
                w_goto = winchInchesToPos(0);
            } else if (winchPosToInches(w_goto) > Config.getNumber("calibrateWinchMotorExtendedInches", 24)) {
                w_goto = winchInchesToPos(Config.getNumber("calibrateWinchMotorExtendedInches", 24));
            }
            p_pid.setReference(p_goto, ControlType.kPosition);
            w_pid.setReference(w_goto, ControlType.kPosition);
        } else if (climberMode == climber_state.CLIMB) {
            w_goto += Math.min(0, value*w_manualSpeed);
            if (winchPosToInches(w_goto) < Config.getNumber("winchClimbUpInches", -2)) {
                w_goto = winchInchesToPos(Config.getNumber("winchClimbUpInches", -2));
            }
            w_pid.setReference(w_goto, ControlType.kPosition);
        }
    }

    public void update() {
        switch (climberMode) {
            case INIT:
                setPoleGotoInches(p_zeroPosition);
                setWinchGotoInches(w_zeroPosition);
                break;
            
            case GOTO_SOLENOID:
                setPoleGotoInches(p_zeroPosition+Config.getNumber("solenoidPositionInches", -0.25));
                setWinchGotoInches(w_zeroPosition+Config.getNumber("solenoidPositionInches", -0.25));
                if (Math.abs(polePosToInches(poleEncoder.getPosition())   - (p_zeroPosition+Config.getNumber("solenoidPositionInches", -0.25))) < 0.1 &&
                    Math.abs(winchPosToInches(winchEncoder.getPosition()) - (w_zeroPosition+Config.getNumber("solenoidPositionInches", -0.25))) < 0.1) {
                    climberMode = climber_state.POGO;
                }
                break;

            case POGO: //TODO why?
                break;

            case CLIMB:
                break;
        }
        SmartDashboard.putString("Climb Mode", climberMode.toString());
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


    //###################################
    //##      Calibration Methods      ##
    //###################################

    /**
     * Controller inputs
     * @param ly Left joystick Y input
     * @param ry Right joystick Y input
     */
    public void calibrateControl(double ly, double ry) {
        if (Math.abs(ly) > 0.05) {
            yStick1Calibrate(ly);// Keep both negative!
        }
        if (Math.abs(ry) > 0.05) {
            yStick2Calibrate(ry);
        }
    }

    public boolean getCalibrating() {
        return isCalibrating;
    }

    private void setPoleZeroInches(double pos) {// Calibrate 0 inches
        p_zeroInchesPos = pos;
    }
    private void setWinchZeroInches(double pos) {
        w_zeroInchesPos = pos;
    }
    private void setPoleExtenedInches(double pos, double heightInInches) {// Calibrate extended inches
        p_ticksPerInch = (pos - p_zeroInchesPos)/heightInInches;
    }
    private void setWinchExtenedInches(double pos, double heightInInches) {
        w_ticksPerInch = (pos - w_zeroInchesPos)/heightInInches;
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
            p_manualSpeed = Config.getNumber("maxManualControlSpeed", 0.9);
            p_calibrateSpeed = Config.getNumber("maxCalibrateControlSpeed", 0.9);
            w_manualSpeed = Config.getNumber("maxManualControlSpeed", 0.9) / ratio;
            w_calibrateSpeed = Config.getNumber("maxCalibrateControlSpeed", 0.9);
        } else {
            p_manualSpeed = Config.getNumber("maxManualControlSpeed", 0.9) * ratio;
            p_calibrateSpeed = Config.getNumber("maxCalibrateControlSpeed", 0.9);
            w_manualSpeed = Config.getNumber("maxManualControlSpeed", 0.9);
            w_calibrateSpeed = Config.getNumber("maxCalibrateControlSpeed", 0.9);
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
                SmartDashboard.putString("Calibration", "[B] Waiting for "+Config.getNumber("calibratePoleMotorExtendedInches", 24)+" inches");
                setPoleZeroInches(p_goto);
                setWinchZeroInches(w_goto);
                calibrationMode = calibration_state.B_WAIT_FOR_EXTENDED_INCHES;
                break;
            case B_WAIT_FOR_EXTENDED_INCHES:
                setPoleExtenedInches(p_goto, Config.getNumber("calibratePoleMotorExtendedInches", 24));
                setWinchExtenedInches(w_goto, Config.getNumber("calibrateWinchMotorExtendedInches", 24));
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
            setPoleGotoInches(Config.getNumber("poleClimbUpInches", 0));
            climberMode = climber_state.CLIMB;
        }
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
            } else if (inch > Config.getNumber("calibratePoleMotorExtendedInches", 24)) {
                inch = Config.getNumber("calibratePoleMotorExtendedInches", 24);
                SmartDashboard.putNumber("Go to (inches)", Config.getNumber("calibratePoleMotorExtendedInches", 24));
            }
            p_inch = inch;
            setPoleGotoInches(inch);
            setWinchGotoInches(inch);
        }

        SmartDashboard.putNumber("pole zero position", p_zeroInchesPos);
        SmartDashboard.putNumber("winch zero position", w_zeroInchesPos);
        SmartDashboard.putNumber("pole ticks/inch", p_ticksPerInch);
        SmartDashboard.putNumber("winch ticks/inch", w_ticksPerInch);
        SmartDashboard.putNumber("Pole Encoder", poleEncoder.getPosition());
        SmartDashboard.putNumber("Winch Encoder", winchEncoder.getPosition());

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