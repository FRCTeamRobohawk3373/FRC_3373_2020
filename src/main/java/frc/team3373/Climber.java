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
import frc.team3373.util.MathUtil;

public class Climber {
    private CANSparkMax poleMotor, winchMotor, inclineMotor;
    private CANEncoder poleEncoder, winchEncoder;

    private Solenoid poleSolenoid, winchSolenoid;

    private CANPIDController p_pid, w_pid;

    /* For calibration */
    //// private double p_zeroInchesOffset =
    //// Config.getNumber("pogoZeroInchesOffset", 0);
    //// private double w_zeroInchesOffset =
    //// Config.getNumber("winchZeroInchesOffset", 0);

    private boolean isCalibrating = false;

    private static enum climber_state {
        INIT, GOTO_SOLENOID, POGO, CLIMB
    }

    private climber_state climberMode = climber_state.INIT;

    private static enum calibration_state {
        INITIAL, WAITING_FOR_FIRST_B, A_WAIT_FOR_ZERO_INCHES, WAIT_FOR_ZERO_INCHES, WAIT_FOR_EXTENDED_INCHES
    }

    private calibration_state calibrationMode = calibration_state.INITIAL;

    private double p_p = Constants.CLIMBER_PP;
    private double p_i = Constants.CLIMBER_PI;
    private double p_d = Constants.CLIMBER_PD;
    private double p_goto = 0;
    private double p_inch = 0;

    private double w_p = Constants.CLIMBER_WP;
    private double w_i = Constants.CLIMBER_WI;// 0.000011;
    private double w_d = Constants.CLIMBER_WD;
    private double w_goto = 0;
    private double w_inch = 0;

    private double zeroInchesOffset = 0;
    private double targetInches = 0;

    private double p_manualIncrement = 2;
    private double w_manualIncrement = 2;
    //// private double p_zeroInchesOffset;
    //// private double w_zeroInchesOffset;
    private double p_rotPerInch;
    private double w_rotPerInch;

    double w_cal_low, p_cal_low, inches_cal_high, inches_cal_low;

    private static Climber instance;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {
        // Get zero inches
        zeroInchesOffset = Config.getNumber("climberZeroInchesOffset", 0);
        // w_zeroInchesOffset = Config.getNumber("winchZeroInchesOffset", 0);
        p_manualIncrement = Config.getNumber("climberManualIncrement", 2);
        w_manualIncrement = Config.getNumber("climberManualIncrement", 2);
        p_rotPerInch = Config.getNumber("pogoRotationPerInch", 7.606507339328597);
        w_rotPerInch = Config.getNumber("winchRotationPerInch", 16.14835958033776);

        targetInches = Config.getNumber("pogoZeroInchesOffset", 0);

        // Solenoids
        poleSolenoid = new Solenoid(Constants.PCM_ID, Constants.POLE_SOLENOID_ID);
        poleSolenoid.set(false);

        winchSolenoid = new Solenoid(Constants.PCM_ID, Constants.WINCH_SOLENOID_ID);
        winchSolenoid.set(false);

        // Motor setup
        poleMotor = new CANSparkMax(Constants.POLE_MOTOR_ID, MotorType.kBrushless);
        poleMotor.setOpenLoopRampRate(0);
        poleMotor.setClosedLoopRampRate(0);
        poleMotor.setInverted(false);
        poleMotor.setIdleMode(IdleMode.kBrake);

        poleEncoder = new CANEncoder(poleMotor);
        poleEncoder.setPosition(0);

        winchMotor = new CANSparkMax(Constants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchMotor.setOpenLoopRampRate(0);
        winchMotor.setClosedLoopRampRate(0);
        
        winchMotor.setInverted(false);
        winchMotor.setIdleMode(IdleMode.kBrake);

        winchEncoder = new CANEncoder(winchMotor);
        winchEncoder.setPosition(0);

        inclineMotor = new CANSparkMax(Constants.INCLINE_MOTOR_ID, MotorType.kBrushless);

        p_pid = poleMotor.getPIDController();
        p_pid.setP(p_p);
        p_pid.setI(p_i);
        p_pid.setD(p_d);
        p_pid.setOutputRange(-Config.getNumber("climberMaxSpeed", 0.9), Config.getNumber("climberMaxSpeed", 0.9));
        p_pid.setReference(0, ControlType.kPosition);

        w_pid = winchMotor.getPIDController();
        w_pid.setP(w_p);
        w_pid.setI(w_i);
        w_pid.setD(w_d);
        w_pid.setOutputRange(-Config.getNumber("climberMaxSpeed", 0.9), Config.getNumber("climberMaxSpeed", 0.9));
        w_pid.setReference(0, ControlType.kPosition);

        adjustMotorSpeeds();

        //adjustManualAndCalibrateSpeeds();

        SmartDashboard.putNumber("Go to (inches)", 0);
    }

    /*
     * public void setMotorsToZero() { poleEncoder.setPosition(0);
     * winchEncoder.setPosition(0);
     * 
     * }
     */
    private void adjustMotorSpeeds() {
        double ratio = w_rotPerInch/p_rotPerInch;
        if (ratio > 1) {
            double speed = Config.getNumber("climberMaxSpeed", 0.9) / (ratio*Config.getNumber("climberAdditionalSyncRatio",1));
            p_pid.setOutputRange(-speed, speed);
        } else {
            double speed = Config.getNumber("climberMaxSpeed", 0.9) * (ratio*Config.getNumber("climberAdditionalSyncRatio",1));
            w_pid.setOutputRange(-speed, speed);
        }
    }

    public void gotoLowPosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(Config.getNumber("climberLowInches", 16));
            setWinchGotoInches(Config.getNumber("climberLowInches", 16));
        }
    }

    public void gotoMiddlePosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(Config.getNumber("climberMiddleInches", 20));
            setWinchGotoInches(Config.getNumber("climberMiddleInches", 20));
        }
    }

    public void gotoHighPosition() {
        if (climberMode == climber_state.POGO) {
            setPoleGotoInches(Config.getNumber("climberHighInches", 24));
            setWinchGotoInches(Config.getNumber("climberHighInches", 24));
        }
    }

    private void setPoleGotoInches(double inches) {
        inches = Math.max(inches, zeroInchesOffset + Config.getNumber("climberReleaseOffsetInches", -0.25));
        p_inch = Math.min(inches, Config.getNumber("climberMaxInches", 67));
        SmartDashboard.putNumber("pogoTargetPosition", p_inch);//TODO remove
        p_goto = (p_inch - zeroInchesOffset) * p_rotPerInch;

        ////p_goto = Math.min(p_goto, Config.getNumber("pogoMaxRotations"));

        SmartDashboard.putNumber("pogoTargetRotations", p_goto);//TODO remove
        p_pid.setReference(p_goto, ControlType.kPosition);
    }

    private void setWinchGotoInches(double inches) {
        inches = Math.max(inches, zeroInchesOffset + Config.getNumber("climberReleaseOffsetInches", -0.25));
        w_inch = Math.min(inches, Config.getNumber("climberMaxInches", 67));
        SmartDashboard.putNumber("winchTargetPosition", w_inch);//TODO remove
        w_goto = (w_inch - zeroInchesOffset) * w_rotPerInch;

        ////w_goto = Math.min(w_goto, Config.getNumber("winchMaxRotations"));

        SmartDashboard.putNumber("winchTargetRotations", w_goto);//TODO remove
        w_pid.setReference(w_goto, ControlType.kPosition);
    }

    // ###################
    // ## Robot Methods ##
    // ###################

    /**
     * Controller inputs
     * 
     * @param ly Left joystick Y input
     * @param lx Left joystick X input
     */
    public void teleOpControl(double ly, double lx) {

        // Deadband
        if (Math.abs(lx) < 0.05)
            lx = 0;

        if (Math.abs(ly) < 0.05)
            ly = 0;

        // Divide joystick movement into 8 sections
        double mag = Math.sqrt(lx * lx + ly * ly);
        double regionFactor = Math.PI / (Constants.numberOfControlSegments);
        double regionNumber = Math.round(Math.atan2(ly, lx) / regionFactor);
        if (regionNumber == 0 || Math.abs(regionNumber) == Constants.numberOfControlSegments
                || Math.abs(regionNumber) == Constants.numberOfControlSegments / 2) {
            double angle = regionNumber * regionFactor;
            lx = mag * Math.cos(angle);
            ly = mag * Math.sin(angle);
        }
        
        SmartDashboard.putNumber("LX control", lx);
        // Incline
        if (climberMode == climber_state.CLIMB) {
            inclineMotor.set(lx * Config.getNumber("climberInclineMotorSpeed", 0.9));
        }

        // Climbing up/down
        if (climberMode == climber_state.POGO) {
            p_inch += ly * p_manualIncrement;
            w_inch += ly * w_manualIncrement;

            setPoleGotoInches(p_inch);
            setWinchGotoInches(w_inch);

        } else if (climberMode == climber_state.CLIMB) {
            w_inch += ly * w_manualIncrement;
            setWinchGotoInches(w_inch);

            /*
            if (ly > 0 && w_inch > Config.getNumber("climberMaxInches")) {
                winchMotor.set(0);
            } else if (ly < 0 && w_inch < zeroInchesOffset + Config.getNumber("climberReleaseOffsetInches", -0.25)) {
                winchMotor.set(0);
            } else {
                winchMotor.set(ly*0.9);
            }*/
        }
    }

    public void update() {
        if (climberMode == climber_state.GOTO_SOLENOID) {
            setPoleGotoInches(zeroInchesOffset + Config.getNumber("climberReleaseOffsetInches", -0.25));
            setWinchGotoInches(zeroInchesOffset + Config.getNumber("climberReleaseOffsetInches", -0.25));

            double p_delta = polePosToInches(poleEncoder.getPosition()) - (zeroInchesOffset
                    + Config.getNumber("climberReleaseOffsetInches", -0.25));
            double w_delta = winchPosToInches(winchEncoder.getPosition()) - (zeroInchesOffset
                    + Config.getNumber("climberReleaseOffsetInches", -0.25));

            SmartDashboard.putNumber("pogoDelta", p_delta);
            SmartDashboard.putNumber("winchDelta", w_delta);
            if (Math.abs(p_delta) < 0.1 && Math.abs(w_delta) < 0.1) {
                climberMode = climber_state.POGO;
            }
        }
        SmartDashboard.putString("Climb Mode", climberMode.toString());

        displayOnShuffleboard();//TODO remove
    }

    private double polePosToInches(double pos) {
        return (pos / p_rotPerInch) + zeroInchesOffset;
    }

    private double winchPosToInches(double pos) {
        return (pos / w_rotPerInch) + zeroInchesOffset;
    }

    public void changeClimbMode() {
        if (climberMode == climber_state.INIT) {// INIT -> GOTO_SOLENOID
            poleSolenoid.set(true);
            winchSolenoid.set(true);
            if (RobotState.isTest()) {
                SmartDashboard.putNumber("Go to (inches)", 0.0001);
                climberMode = climber_state.POGO;
            } else {
                climberMode = climber_state.GOTO_SOLENOID;
            }
        } else if (climberMode == climber_state.POGO) {// POGO -> CLIMB
            poleSolenoid.set(false);
            //winchSolenoid.set(false);
            setPoleGotoInches(Config.getNumber("climberZeroInchesOffset", 0));// Retract pogo stick
            climberMode = climber_state.CLIMB;
        }
    }

    // #########################
    // ## Calibration Methods ##
    // #########################

    /**
     * Controller inputs
     * 
     * @param ly Left joystick Y input
     * @param ry Right joystick Y input
     */
    public void calibrateControl(double ly, double ry) {
        if (Math.abs(ly) > 0.05) {// Calibrate-control pogo motor
            p_goto += ly * Config.getNumber("climberManualIncrement", 2)
                    * Config.getNumber("climberCalibrateIncrementRatio", 0.555);
            p_pid.setReference(p_goto, ControlType.kPosition);
        }
        if (Math.abs(ry) > 0.05) {// Calibrate-control winch motor
            w_goto += ry * Config.getNumber("climberManualIncrement", 2)
                    * Config.getNumber("climberCalibrateIncrementRatio", 0.555);
            w_pid.setReference(w_goto, ControlType.kPosition);
        }
        SmartDashboard.putNumber("pogoTargetRotations", p_goto);
        SmartDashboard.putNumber("winchTargetRotations", w_goto);        
    }

    public boolean getCalibrating() {
        return isCalibrating;
    }
/*
    public void startCalibrateOptions() {
        if (isCalibrating) {
            isCalibrating = false;
            SmartDashboard.putString("Calibration", "Calibration aborted");
        } else {
            if (climberMode == climber_state.POGO) {
                isCalibrating = true;
                calibrationMode = calibration_state.WAITING_FOR_A_OR_B;
                SmartDashboard.putString("Calibration", "Waiting for A or B option.");
            } else {
                SmartDashboard.putString("Calibration", "Robot not in POGO mode!");
            }
        }
    }
*/
    /* public void calibrateHeights() {
        switch (calibrationMode) {
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
        default:
            break;
        }
    } */

    public void unlockSolenoids() {
        poleSolenoid.set(true);
        winchSolenoid.set(true);
        climberMode = climber_state.GOTO_SOLENOID;
    }

    public void calibrateInches() {
        switch (calibrationMode) {
        case INITIAL:// <-- previous state
            poleMotor.set(0);
            winchMotor.set(0);

            poleSolenoid.set(true);
            winchSolenoid.set(true);

            poleEncoder.setPosition(0);
            winchEncoder.setPosition(0);
            
            p_goto=0;
            w_goto=0;

            p_pid.setReference(p_goto, ControlType.kPosition);
            w_pid.setReference(p_goto, ControlType.kPosition);

            SmartDashboard.putString("Calibration", "[B] Waiting for low inches");
            SmartDashboard.putNumber("[!] INPUT INCHES [!]", 43);
               
            isCalibrating = true;
            calibrationMode = calibration_state.WAIT_FOR_ZERO_INCHES;
            break;
        case WAIT_FOR_ZERO_INCHES:
            SmartDashboard.putString("Calibration", "[B] Waiting for high position");
            p_cal_low = poleEncoder.getPosition();
            w_cal_low = winchEncoder.getPosition();

            inches_cal_low = SmartDashboard.getNumber("[!] INPUT INCHES [!]", 43);
            zeroInchesOffset = inches_cal_low;
            Config.writeNumber("climberZeroInchesOffset", zeroInchesOffset);

            SmartDashboard.putNumber("[!] INPUT INCHES [!]", 99);
            calibrationMode = calibration_state.WAIT_FOR_EXTENDED_INCHES;
            break;
        case WAIT_FOR_EXTENDED_INCHES:
            inches_cal_high = SmartDashboard.getNumber("[!] INPUT INCHES [!]", 65);

            SmartDashboard.putNumber("poleDelta", p_goto - p_cal_low);
            SmartDashboard.putNumber("winchDelta", w_goto - w_cal_low);
            SmartDashboard.putNumber("inchesDelta", inches_cal_high - inches_cal_low);

            setPoleExtenedInches(poleEncoder.getPosition() - p_cal_low, inches_cal_high - inches_cal_low);
            setWinchExtenedInches(winchEncoder.getPosition() - w_cal_low, inches_cal_high - inches_cal_low);
            SmartDashboard.putString("Calibration", "[B] Done");
            calibrationMode = calibration_state.INITIAL;
            isCalibrating = false;

            // Write values to defaults.json
            Config.writeNumber("pogoRotationPerInch", p_rotPerInch);
            Config.writeNumber("winchRotationPerInch", w_rotPerInch);

            System.out.println("p: " + p_rotPerInch);
            System.out.println("w: " + w_rotPerInch);
            break;
        default:
            break;
        }
    }

    private void setPoleExtenedInches(double pos, double heightInInches) {// Calibrate extended inches
        p_rotPerInch = pos / heightInInches;
        
    }

    private void setWinchExtenedInches(double pos, double heightInInches) {
        w_rotPerInch = pos / heightInInches;
    }

    public void displayOnShuffleboard() {

        /* Pole motor */
        // !SmartDashboard.putNumber("pole now",
        // polePosToInches(poleEncoder.getPosition()));
        // !SmartDashboard.putNumber("pole goto", polePosToInches(p_goto));
        // !double p = SmartDashboard.getNumber("pole motor P", p_p);
        // !double i = SmartDashboard.getNumber("pole motor I", p_i);
        // !double d = SmartDashboard.getNumber("pole motor D", p_d);
        //double inch = SmartDashboard.getNumber("Go to (inches)", p_inch);
        /*
         * if (p_p != p) { p_p = p; p_pid.setP(p); } if (p_i != i) { p_i = i;
         * p_pid.setI(i); } if (p_d != d) { p_d = d; p_pid.setD(d); }
         */

         /*
        if (p_inch != inch) {

            inch = MathUtil.clamp(inch, zeroInchesOffset, Config.getNumber("pogoMaxRotations"));
            inch = MathUtil.clamp(inch, zeroInchesOffset, Config.getNumber("winchMaxRotation"));
            setPoleGotoInches(MathUtil.clamp(inch, 0, Config.getNumber("pogoCalibrateExtendedInches", 24)));
            setWinchGotoInches(MathUtil.clamp(inch, 0, Config.getNumber("winchCalibrateExtendedInches", 24)));

            p_inch = inch;
            setPoleGotoInches(inch);
            setWinchGotoInches(inch);
        }*/

        SmartDashboard.putNumber("zero position", zeroInchesOffset);
        SmartDashboard.putNumber("pogo rotations/inch", p_rotPerInch);
        SmartDashboard.putNumber("winch rotations/inch", w_rotPerInch);
        SmartDashboard.putNumber("[!]Pogo Encoder", poleEncoder.getPosition());
        SmartDashboard.putNumber("[!]Winch Encoder", winchEncoder.getPosition());

        SmartDashboard.putNumber("Pogo Inches", polePosToInches(poleEncoder.getPosition()));
        SmartDashboard.putNumber("Winch Inches", winchPosToInches(winchEncoder.getPosition()));//!!

        /* Winch motor */
        // !SmartDashboard.putNumber("winch now",
        // winchPosToInches(winchEncoder.getPosition()));
        // !SmartDashboard.putNumber("winch goto", winchPosToInches(w_goto));
        // !p = SmartDashboard.getNumber("winch motor P", w_p);
        // !i = SmartDashboard.getNumber("winch motor I", w_i);
        // !d = SmartDashboard.getNumber("winch motor D", w_d);
        /*
         * if (w_p != p) { w_p = p; w_pid.setP(p); } if (w_i != i) { w_i = i;
         * w_pid.setI(i); } if (w_d != d) { w_d = d; w_pid.setD(d); }
         */
    }

    public void disable() {
        climberMode = climber_state.INIT;
    }
}