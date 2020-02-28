package frc.team3373;
//TODO maybe delete ballCount from code?
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.DelayTrueBoolean;

public class Indexer4 {
    private static Indexer4 instance;
    private WPI_TalonSRX intake, conveyor, preload, load;
    private DigitalInput intakeSensor, conveyorSensor, preloadSensor;

    private DelayTrueBoolean timedBool1, timedBool2, timedBool3, timedBool4;
    private boolean pos1, pos2, realPos2, pos3, timedLock4;// Timed booleans controled by respective DelayTrueBoolean values
    private boolean occupy4; // Is ball moving from 4 -> 5
    private boolean is3Locked, is4Locked;
    private int ballCount;
    private int preloadPos, loadPos;
    private boolean isShooting = false;
    private boolean isRunningIntake = false;

    private boolean isPanicMode = false;
    private Timer panicTimer = new Timer();
    private Timer delayPos2 = new Timer();

    private int numTimes = 0; //TODO Remove

    // Calibration variables
    private Timer calTimer = new Timer();
    private boolean startedCal = false;
    private boolean intakeCal = false;
    private boolean conveyorCal = false;
    private boolean preloadCal = false;
    private boolean loadCal = false;
    private int conveyorCenterCal = 0;

    public enum State {
        AVAILABLE, OCCUPIED, PREMOVING, MOVING, POSTMOVING
    }

    private State[] ballStates = new State[] { State.AVAILABLE, State.AVAILABLE, State.AVAILABLE,
            State.AVAILABLE };

    public static Indexer4 getInstance() {
        if (instance == null) {
            instance = new Indexer4();
        }
        return instance;
    }

    public Indexer4() {
        intakeSensor = new DigitalInput(Constants.INTAKE_BALL_SENSOR_INDEX);
        conveyorSensor = new DigitalInput(Constants.CONVEYOR_BALL_SENSOR_INDEX);
        preloadSensor = new DigitalInput(Constants.PRELOAD_BALL_SENSOR_INDEX);

        // Timed booleanstimedBool3
        timedBool1 = new DelayTrueBoolean();
        timedBool2 = new DelayTrueBoolean();
        timedBool3 = new DelayTrueBoolean();
        timedBool4 = new DelayTrueBoolean();
        // tboolConveyor = new DelayTrueBoolean();
        pos1 = false;
        pos2 = false;
        realPos2 = false;
        pos3 = false;
        timedLock4 = false;
        is3Locked = false;
        is4Locked = false;

        ballCount = 0;
        isPanicMode = false;
        panicTimer = new Timer();
        panicTimer.stop();

        intake = new WPI_TalonSRX(Constants.INTAKE_INDEX);
        conveyor = new WPI_TalonSRX(Constants.CONVEYOR_INDEX);
        preload = new WPI_TalonSRX(Constants.PRELOAD_INDEX);
        load = new WPI_TalonSRX(Constants.LOAD_INDEX);

        // !
        intake.configFactoryDefault();
        conveyor.configFactoryDefault();
        preload.configFactoryDefault();
        load.configFactoryDefault();
        // !

        intake.setNeutralMode(NeutralMode.Brake);
        conveyor.setNeutralMode(NeutralMode.Brake);
        preload.setNeutralMode(NeutralMode.Brake);
        load.setNeutralMode(NeutralMode.Brake);

        preload.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        preload.setSensorPhase(false); // (was set to true) TODO Change?
        preload.setInverted(false);
        preload.config_kP(0, Config.getNumber("preload_P", 5));
        preload.config_kI(0, 0);
        preload.config_kD(0, 0);
        preload.configClosedLoopPeakOutput(0, Config.getNumber("maxPreloadSpeed", 0.7));
        preload.configAllowableClosedloopError(0,
                (int) (Config.getNumber("pidError", 0.1) * Config.getNumber("preloadEncoderScale", 1992)));
        preload.getSensorCollection().setQuadraturePosition(0, 100);
        preload.set(ControlMode.Position, 0);

        load.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        load.setSensorPhase(false); // TODO Change?
        load.setInverted(false);
        load.config_kP(0, Config.getNumber("preload_P", 5));
        load.config_kI(0, 0);
        load.config_kD(0, 0);
        load.configClosedLoopPeakOutput(0, Config.getNumber("maxLoadSpeed", 0.7));
        load.configAllowableClosedloopError(0,
                (int) (Config.getNumber("pidError", 0.05) * Config.getNumber("loadEncoderScale", 1992)));
        load.getSensorCollection().setQuadraturePosition(0, 100);
        load.set(ControlMode.Position, 0);
    }

    private boolean isState(int index, State val) {
        return ballStates[index - 1] == val;
    }

    private State getState(int index) {
        return ballStates[index - 1];
    }

    private void setState(int index, State val) {
        ballStates[index - 1] = val;
    }

    /**
     * In case the robot is preloaded.
     */
    public boolean setInitialBallStates(State[] newStates) {
        if (newStates.length != 4) {
            return false;
        }
        ballStates = newStates;
        ballCount = 0;
        for (State b : ballStates) {
            if (b == State.OCCUPIED) {
                addBall();
            }
        }
        isShooting = false;

        return true;
    }

    public void stop(){
        for (State b : ballStates) {
            if (b == State.MOVING) {
                b = State.OCCUPIED;
            }
        }

        intake.set(0);
        conveyor.set(0);
        preload.set(0);
        load.set(0);
    }

    public void startShooting() {
        //intake.set(0);
        if (ballCount > 0)
            isShooting = true;
    }

    public void stopShooting() {
        isShooting = false;
    }

    /**
     * Called by Shooter.java to remove the 5th ball position.
     */
    public void unloadBall5() {
        if (isState(4, State.OCCUPIED) && isShooting) {
            loadPos += Config.getNumber("loadRotations", -2);
            load.set(ControlMode.Position, loadPos * Config.getNumber("loadEncoderScale", 3413));
            setState(4, State.MOVING);
            ballCount--;// Decrement ball counter
        }
    }

    public void unloadBall4() {
        unloadBall5();
    }

    public void startIntake() {
        if (!isState(1, State.OCCUPIED)) {
            isRunningIntake = true;
            //setState(1, State.MOVING);
            intake.set(Config.getNumber("intakeMotorSpeed", -0.6));
        }
    }

    public void stopIntake() {
        isRunningIntake = false;
        intake.set(0);
    }

    private void updatePos1() {
        switch (getState(1)) {
        case AVAILABLE: // Ready to receive a ball
            if (pos1) { // If color sensor detects a new cal
                System.out.println("1 is occupied with "+ballCount+" balls");
                setState(1, State.OCCUPIED);// Set position 1 state to OCCUPIED
                addBall(); // Increment ball counter
            }
            break;

        case OCCUPIED:
            if (ballCount >= 4) {
                intake.set(0);
            }
            if (isState(2, State.AVAILABLE)) {
                intake.set(Config.getNumber("intakeMotorSpeed", -0.6));
                setState(1, State.MOVING);
            } else {
                intake.set(0);
            }
            break;

        case MOVING: // If ball is being pushed to ball position 2
            if (!pos1) { // If position 1 is empty...
                setState(1, State.AVAILABLE); // Reset position 1 back to the original state of being ready to
                                              // recieve a ball
                System.out.println("Set 1 to available");
                if (!isRunningIntake) {
                    intake.set(0);
                }
            }
            break;

        default:
            break;
        }
    }

    private void updateConveyor() {
  
        switch (getState(2)) {
            case AVAILABLE:
                if (isState(1, State.MOVING)) {
                    setState(2, State.PREMOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.6));
                }
                break;

            case PREMOVING:
                if (pos2) {
                    delayPos2.reset();
                    delayPos2.start();
                    setState(2, State.MOVING);
                }
                break;   
                
            case MOVING:
                if (isState(3, State.AVAILABLE)) {
                    setState(2, State.POSTMOVING);
                    delayPos2.stop();
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.6));

                }else if (delayPos2.get() > Config.getNumber("conveyorMoveDelay", 0.777)) {//TODO put in config
                    delayPos2.stop();
                    setState(2, State.OCCUPIED);
                    conveyor.set(0);
                }            
                break;

            case OCCUPIED:
                if (isState(3, State.AVAILABLE)) {
                    setState(2, State.POSTMOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.6));
                }
                break;

            case POSTMOVING:
                if (!isState(3, State.AVAILABLE)) {
                    setState(2, State.AVAILABLE);
                    conveyor.set(0);
                }
                break;

            default:
                break;
        }
    }

    private void updatePos3() {
        switch (getState(3)) {
        case AVAILABLE:
            is3Locked = false;
            if (pos3) {
                setState(3, State.OCCUPIED);
            }
            break;

        case OCCUPIED:
            if (isState(4, State.AVAILABLE)) {
                setState(3, State.MOVING);
                preloadPos += Config.getNumber("preloadRotations", -3);
                preload.set(ControlMode.Position, preloadPos * Config.getNumber("preloadEncoderScale", 4096));
                occupy4 = true;
            } else if (!is3Locked) {
                preload.set(ControlMode.Position, (preloadPos - Config.getNumber("preloadLockOffset", -0.2))
                        * Config.getNumber("preloadEncoderScale", 4096));
                is3Locked = true;
            }
            break;

        case MOVING:
            double posError = Math.abs(preloadPos) * Config.getNumber("preloadEncoderScale", 1992)
                    - Math.abs(preload.getSensorCollection().getQuadraturePosition());

            if (Math.abs(posError) < Config.getNumber("pidError", 0.05)
                    * Config.getNumber("preloadEncoderScale", 1992)) {
                preload.set(0);
                setState(3, State.AVAILABLE);
                // setState(5, State.OCCUPIED) moved to timed 
            }
            break;

        default:
            break;
        }
    }

    private void updatePos4() {
        switch (getState(4)) {
        case OCCUPIED:
            if (!is4Locked) {
                load.set(ControlMode.Position, (loadPos - Config.getNumber("loadLockOffset", -0.1))
                        * Config.getNumber("loadEncoderScale", 3413)); // Moves to lock position
                is4Locked = true;
            }
            break;

        case MOVING:
            double posError = Math.abs(loadPos) * Config.getNumber("loadEncoderScale", 3413)
                    - Math.abs(load.getSensorCollection().getQuadraturePosition()); // Calculate PID error

            if (Math.abs(posError) < Config.getNumber("pidError", 0.05) * Config.getNumber("loadEncoderScale", 3413)) {

                is4Locked = false;
                load.set(0); // Stop motor after PID exits
                setState(4, State.AVAILABLE); // Set to available
            }
            break;

        default:
            break;
        }
    }

    private void addBall() {
        if (ballCount < 4) {
            ballCount++;
        } else {
            ballCount = 4;
        }
    }

    public void init() {
        startIntake();
    }

    public void enterPanicMode() {
        if (!isPanicMode) {
            isPanicMode = true;
            panicTimer.start();
        }
    }

    public void exitPanicMode() {
        isPanicMode = false;
        intake.set(0);
        conveyor.set(0);
        setState(1, State.AVAILABLE);
        setState(2, State.AVAILABLE);
        panicTimer.reset();
        panicTimer.stop();
    }

    /**
     * Updates status of all 5 positions and real sensor booleans
     */
    public void update() {
        
        if (isPanicMode) {
            intake.set(-Config.getNumber("intakeMotorSpeed", 0.6));// Inverse intake motor
            conveyor.set(-Config.getNumber("conveyorMotorSpeed", 0.6));// Inverse conveyor motor
            if (panicTimer.get() > 1) {
                exitPanicMode(); 
            }

        } else {
            if (timedLock4) { // When 4 -> 5 and timer on boolean has ended, set 5 to occupied
                occupy4 = false;
                setState(4, State.OCCUPIED);
            }
            pos1 = timedBool1.update(intakeSensor.get(), Config.getNumber("intakeSensorDelay", 0.3));
            pos2 = timedBool2.update(conveyorSensor.get(), Config.getNumber("conveyorSensorDelay", 0.2));        
            pos3 = timedBool3.update(preloadSensor.get(), Config.getNumber("preloadSensorDelay", 0.5));
            timedLock4 = timedBool4.update(occupy4, Config.getNumber("lockLoadDelay", 0.84));

            updatePos1();
            updateConveyor();
            updatePos3();
            updatePos4();
        }

        // ! testing code
        displayData();
    }

    public void zeroMotors() {
        preload.set(ControlMode.Position, preloadPos);
        load.set(ControlMode.Position, loadPos);
    }

    public void displayData() {
        SmartDashboard.putBoolean("Intake Sensor", pos1);// ? Do we need this code after debugging?
        SmartDashboard.putBoolean("Conveyor Sensor", pos2);
        SmartDashboard.putBoolean("Preload Sensor", pos3);
        SmartDashboard.putNumber("Intake Speed", intake.get());
        SmartDashboard.putNumber("Conveyor Speed", conveyor.get());

        String states = "[";
        for (int i = 0; i < ballStates.length - 1; i++) {
            states += ballStates[i] + ", ";
        }
        states += ballStates[ballStates.length - 1] + "]";
        SmartDashboard.putString("States", states);
        SmartDashboard.putNumber("Ball count", ballCount);
        SmartDashboard.putNumber("Preload Pos", preloadPos);
        SmartDashboard.putNumber("Preload Pos Encoder Units",
                preloadPos * Config.getNumber("preloadEncoderScale", 1992));
        SmartDashboard.putBoolean("4 Locked", is3Locked);
        SmartDashboard.putBoolean("5 Locked", is4Locked);

        SmartDashboard.putBoolean("isShooting", isShooting);

        SmartDashboard.putNumber("Preload Speed", preload.get());
        SmartDashboard.putNumber("Load Speed", load.get());

        SmartDashboard.putNumber("Preload Quad", preload.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber("Load Quad", load.getSensorCollection().getQuadraturePosition());
        // SmartDashboard.putNumber("Preload Pulse",
        // preload.getSensorCollection().getPulseWidthPosition());
        // SmartDashboard.putNumber("Load Pulse",
        // load.getSensorCollection().getPulseWidthPosition());

    }

    public void moveMotor(String name, double speed) {
        switch (name) {
        case "preload":
            preload.set(speed);
            break;
        case "load":
            load.set(speed);
            break;
        default:
            break;
        }
    }

    public void configTiming(String sensor) {
        switch (sensor) {
        case "indexer":
            intake.set(Config.getNumber("intakeMotorSpeed", -0.6));
            intakeCal = !intakeCal;
            if (!intakeCal) {
                calTimer.stop();
                System.out.println("Indexer timing: " + calTimer.get());
                intake.set(0);
                startedCal = false;
            }
            break;
        case "conveyor":
            intake.set(Config.getNumber("intakeMotorSpeed", -0.6));
            conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
            conveyorCal = !conveyorCal;
            if (!conveyorCal) {
                calTimer.stop();
                System.out.println("Conveyor center timing: " + calTimer.get());
                intake.set(0);
                conveyor.set(0);
                startedCal = false;
            }
            break;
        case "conveyor_center":
            intake.set(Config.getNumber("intakeMotorSpeed", -0.6));
            conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
            if (conveyorCenterCal == 0) {
                calTimer.reset();
                calTimer.start();
                SmartDashboard.putString("Calibrate", "Premove");
            }
            conveyorCenterCal++;
            if (conveyorCenterCal == 2) {
                calTimer.stop();
                System.out.println("Conveyor center timing: " + calTimer.get());
                intake.set(0);
                conveyor.set(0);
                startedCal = false;
                conveyorCenterCal = 0;
            }
            break;
        case "preload":
            intake.set(Config.getNumber("intakeMotorSpeed", -0.6));
            conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
            preloadCal = !preloadCal;
            if (!preloadCal) {
                calTimer.stop();
                System.out.println("Preload timing: " + calTimer.get());
                intake.set(0);
                conveyor.set(0);
                startedCal = false;
            }
            break;
        case "load":
            preload.set(-Config.getNumber("maxPreloadSpeed", 0.7));
            loadCal = !loadCal;
            if (!loadCal) {
                calTimer.stop();
                System.out.println("Load timing: " + calTimer.get());
                preload.set(0);
            } else {
                calTimer.reset();
                calTimer.start();
            }
            break;
        default:
            return;
        }
    }

    public void updateTest() {
        if (intakeCal && !startedCal) {
            if (intakeSensor.get()) {
                System.out.println("Starting indexer timing");
                calTimer.reset();
                calTimer.start();
                startedCal = true;
            }/////////////// hello from the other side
        } else if (conveyorCal && !startedCal) {
            if (conveyorSensor.get()) {
                System.out.println("Starting conveyor timing");
                calTimer.reset();
                calTimer.start();
                startedCal = true;
            }
        } else if (preloadCal && !startedCal) {
            if (preloadSensor.get()) {
                System.out.println("Starting preload timing");
                calTimer.reset();
                calTimer.start();
                startedCal = true;
            }
        }
        displayData();
    }

}
