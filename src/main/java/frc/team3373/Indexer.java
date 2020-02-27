package frc.team3373;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.DelayTrueBoolean;

public class Indexer {
    private static Indexer instance;
    private WPI_TalonSRX intake, conveyor, preload, load;
    private DigitalInput intakeSensor, conveyorSensor, preloadSensor;

    private DelayTrueBoolean timedBool1, timedBool2, timedBool4, timedBool5;
    private boolean pos1, pos2, pos4, timedLock5;// Timed booleans controled by respective DelayTrueBoolean values
    private boolean occupy5; // Is ball moving from 4 -> 5
    private boolean is4Locked, is5Locked;
    private int ballCount;
    private int preloadPos, loadPos;
    private boolean isShooting = false;

    private int numTimes = 0; // TODO Remove

    // Calibration variables
    private Timer calTimer = new Timer();
    private boolean startedCal = false;
    private boolean intakeCal = false;
    private boolean conveyorCal = false;
    private boolean preloadCal = false;
    private boolean loadCal = false;

    public enum State {
        AVAILABLE, OCCUPIED, MOVING, REVERSE
    }

    State[] ballStates = new State[] { State.AVAILABLE, State.AVAILABLE, State.AVAILABLE, State.AVAILABLE,
            State.AVAILABLE };

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    public Indexer() {
        intake = new WPI_TalonSRX(Constants.INTAKE_INDEX);
        conveyor = new WPI_TalonSRX(Constants.CONVEYOR_INDEX);
        preload = new WPI_TalonSRX(Constants.PRELOAD_INDEX);
        load = new WPI_TalonSRX(Constants.LOAD_INDEX);

        intake.setNeutralMode(NeutralMode.Brake);
        conveyor.setNeutralMode(NeutralMode.Brake);
        preload.setNeutralMode(NeutralMode.Brake);
        load.setNeutralMode(NeutralMode.Brake);

        intakeSensor = new DigitalInput(Constants.INTAKE_BALL_SENSOR_INDEX);
        conveyorSensor = new DigitalInput(Constants.CONVEYOR_BALL_SENSOR_INDEX);
        preloadSensor = new DigitalInput(Constants.PRELOAD_BALL_SENSOR_INDEX);

        // Timed booleans
        timedBool1 = new DelayTrueBoolean();
        timedBool2 = new DelayTrueBoolean();
        timedBool4 = new DelayTrueBoolean();
        timedBool5 = new DelayTrueBoolean();
        // tboolConveyor = new DelayTrueBoolean();
        pos1 = false;
        pos2 = false;
        pos4 = false;
        timedLock5 = false;
        is4Locked = false;
        is5Locked = false;

        ballCount = 0;

        // !
        preload.configFactoryDefault();
        load.configFactoryDefault();
        // !

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
        if (newStates.length != 5) {
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
        intake.set(0);
        if (ballCount > 0)
            isShooting = true;
    }

    public void stopShooting() {
        isShooting = false;
    }

    public void reverseConveyor() {
        setState(3, State.REVERSE);
        conveyor.set(-Config.getNumber("conveyorMotorSpeed", 0.7));
    }

    /**
     * Called by Shooter.java to remove the 5th ball position.
     */
    public void unloadBall5() {
        if (isState(5, State.OCCUPIED) && isShooting) {
            loadPos += Config.getNumber("loadRotations", -2);
            load.set(ControlMode.Position, loadPos * Config.getNumber("loadEncoderScale", 3413));
            setState(5, State.MOVING);
            ballCount--;// Decrement ball counter
        }
    }

    public void startIntake() {
        if (!isState(1, State.OCCUPIED)) {
            setState(1, State.MOVING);
            intake.set(Config.getNumber("intakeMotorSpeed", -0.6));
        }
    }

    public void stopIntake() {
        intake.set(0);
    }

    private void updatePos1() {
        switch (getState(1)) {
        case AVAILABLE: // Ready to receive a ball
            if (pos1) { // If color sensor detects a new cal
                setState(1, State.OCCUPIED);// Set position 1 state to OCCUPIED
                addBall(); // Increment ball counter
            }
            break;

        case OCCUPIED:
            if (ballCount >= 5) {
                //System.out.println("Stopping intake: balls >= 5 ballCount = " + ballCount);
                SmartDashboard.putNumber("Stopping intake", numTimes++);
                stopIntake();
            }
            if (isState(2, State.REVERSE)) {
                stopIntake();
            }

        case MOVING: // If ball is being pushed to ball position 2
            startIntake();
            //System.out.println("Starting intake");
            if (!pos1) { // If position 1 is empty...
                setState(1, State.AVAILABLE); // Reset position 1 back to the original state of being ready to
                                              // recieve a ball
                System.out.println("Set 1 to available");
            }
            break;

        default:
            break;
        }
    }

    private void updateConveyor() {
        if (pos2 && isState(2, State.AVAILABLE)) {
            setState(2, State.OCCUPIED);
            conveyor.set(0);
        }
        if (isShooting) { // ? Can assume at least one ball?
            if (isState(2, State.OCCUPIED) && isState(3, State.OCCUPIED)) {
                if (isState(4, State.AVAILABLE)) { // Move 2 -> 3 & 3 -> 4
                    if (isState(1, State.OCCUPIED))
                        setState(1, State.MOVING);
                    System.out.println("O, O: 2 -> 3 && 3 -> 4");
                    setState(2, State.MOVING);
                    setState(3, State.MOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
                }
            } else if (isState(2, State.MOVING) && isState(3, State.MOVING)) {
                if (!isState(1, State.MOVING) && !isState(4, State.AVAILABLE) && !pos2) {
                    // ! Check if timing is off: 3 -> 4 before pos2 turns off || 2 -> 3 before pos4
                    // ! turns on
                    System.out.println("M, M: 1 ! moving, 4 ! available");
                    setState(2, State.AVAILABLE);
                    setState(3, State.OCCUPIED);
                    conveyor.set(0);
                } else if (!isState(4, State.AVAILABLE)) {
                    System.out.println("M, M: 1 moving, 4 ! available");
                    setState(2, State.AVAILABLE);
                    setState(3, State.OCCUPIED);
                }
            } else if (isState(2, State.AVAILABLE) && isState(3, State.OCCUPIED)) {
                if (isState(4, State.AVAILABLE)) {
                    System.out.println("A, O: 4 available");
                    setState(3, State.MOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
                }
            } else if (isState(2, State.AVAILABLE) && isState(3, State.MOVING)) {
                if (!isState(4, State.AVAILABLE)) {
                    System.out.println("A, M: 4 ! available");
                    setState(3, State.AVAILABLE);
                    conveyor.set(0);
                }
            }
        } else {
            if ((isState(2, State.AVAILABLE) || isState(2, State.OCCUPIED)) && isState(3, State.AVAILABLE)) {
                // Can start moving conveyor along if 3 is available and there is a ball in 1
                if (isState(1, State.OCCUPIED)) {
                    setState(1, State.MOVING);
                    if (isState(2, State.OCCUPIED)) // Ball 2 -> 3 if ball in 2
                        setState(2, State.MOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
                } // ? Finishes moving when first 'if' triggers?
            } else if (isState(2, State.MOVING) && isState(3, State.AVAILABLE)) {
                if (!pos2) { // Wait for 2 to get to 3 and change states
                    setState(2, State.AVAILABLE);
                    setState(3, State.OCCUPIED);
                }
            } else if (isState(2, State.OCCUPIED) && isState(3, State.OCCUPIED) && isState(1, State.OCCUPIED)
                    && isState(4, State.AVAILABLE)) {
                setState(2, State.MOVING);
                setState(3, State.MOVING);
                conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
            } else if (isState(2, State.MOVING) && isState(3, State.MOVING)) { // Has to come from previous state
                if (!pos2) { // ? Can assume ball from 1
                    setState(2, State.AVAILABLE);
                    setState(3, State.OCCUPIED);
                }
            } /*else if (isState(2, State.AVAILABLE) && isState(3, State.OCCUPIED) && isState(1, State.AVAILABLE)) { // When conveyor is messed up by
                                                                                    // shooting
                // ? 1 can't be occupied after shooting
                setState(3, State.REVERSE);
                conveyor.set(-Config.getNumber("conveyorMotorSpeed", 0.7));
            } else if (isState(3, State.REVERSE)) {
                if (timedBool2.update(conveyorSensor.get(), Config.getNumber("reverseConveyorDelay"))) {
                    setState(2, State.OCCUPIED);
                    setState(3, State.AVAILABLE);
                    conveyor.set(0);
                }
            }*/
        }
    }

    @SuppressWarnings("unused")
    private void updatePos2() {
        switch (getState(2)) {
        case AVAILABLE:
            if (pos2) {
                setState(2, State.OCCUPIED);
                conveyor.set(0);
                if (isState(3, State.MOVING)) {
                    setState(3, State.OCCUPIED);
                }
            }
            break;

        case OCCUPIED:
            if (isShooting) {
                if (isState(4, State.AVAILABLE)) {
                    setState(2, State.MOVING);
                    setState(3, State.MOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.6));
                }
            } else {
                if (isState(1, State.OCCUPIED)) {
                    switch (getState(3)) {
                    case AVAILABLE:
                        setState(2, State.MOVING);
                        setState(3, State.MOVING);
                        conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.6));
                        break;

                    case OCCUPIED:
                        if (isState(4, State.AVAILABLE)) {
                            setState(2, State.MOVING);
                            setState(3, State.MOVING);
                            conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.6));
                        }
                        break;

                    default:
                        break;
                    }
                }
            }
            break;

        case MOVING:
            if (isShooting && isState(1, State.AVAILABLE) && !pos2) {// If position 1 is available
                setState(2, State.AVAILABLE);
                setState(3, State.OCCUPIED);
                conveyor.set(0);
            } else {
                if (!pos2) {
                    setState(2, State.AVAILABLE);// ! look at later
                }
            }
            break;

        case REVERSE:
            if (timedBool2.update(!conveyorSensor.get(), Config.getNumber("reverseConveyorDelay", 0.05))) {// TODO REMOVE
                setState(2, State.OCCUPIED);// ! look at later
                setState(3, State.AVAILABLE);
                conveyor.set(0);
            }
            break;

        default:
            break;
        }
    }

    @SuppressWarnings("unused")
    private void updatePos3() {
        switch (getState(3)) {
        case OCCUPIED:
            if (isShooting) {
                if (isState(4, State.AVAILABLE)) {
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.6));
                    setState(3, State.MOVING);
                }
            } else {
                if (isState(2, State.AVAILABLE)) {
                    setState(2, State.REVERSE);
                    setState(3, State.REVERSE);
                    conveyor.set(-Config.getNumber("conveyorMotorSpeed", 0.6));
                }
            }
            break;

        case MOVING:
            if (isShooting) {
                if (isState(4, State.OCCUPIED)) {
                    conveyor.set(0);
                    setState(3, State.AVAILABLE);
                }
            }
            break;

        default:
            break;
        }
    }

    private void updatePos4() {
        switch (getState(4)) {
        case AVAILABLE:
            is4Locked = false;
            if (pos4) {
                setState(4, State.OCCUPIED);
            }
            break;

        case OCCUPIED:
            if (isState(5, State.AVAILABLE)) {
                setState(4, State.MOVING);
                preloadPos += Config.getNumber("preloadRotations", -3);
                preload.set(ControlMode.Position, preloadPos * Config.getNumber("preloadEncoderScale", 4096));
                occupy5 = true;
            } else if (!is4Locked) {
                preload.set(ControlMode.Position, (preloadPos - Config.getNumber("preloadLockOffset", -0.2))
                        * Config.getNumber("preloadEncoderScale", 4096));
                is4Locked = true;
            }
            break;

        case MOVING:
            double posError = Math.abs(preloadPos) * Config.getNumber("preloadEncoderScale", 1992)
                    - Math.abs(preload.getSensorCollection().getQuadraturePosition());

            if (Math.abs(posError) < Config.getNumber("pidError", 0.05)
                    * Config.getNumber("preloadEncoderScale", 1992)) {
                preload.set(0);
                setState(4, State.AVAILABLE);
                // setState(5, State.OCCUPIED) moved to timed 
            }
            break;

        default:
            break;
        }
    }

    private void updatePos5() {
        switch (getState(5)) {
        case OCCUPIED:
            if (!is5Locked) {
                load.set(ControlMode.Position, (loadPos - Config.getNumber("loadLockOffset", -0.1))
                        * Config.getNumber("loadEncoderScale", 3413)); // Moves to lock position
                is5Locked = true;
            }
            break;

        case MOVING:
            double posError = Math.abs(loadPos) * Config.getNumber("loadEncoderScale", 3413)
                    - Math.abs(load.getSensorCollection().getQuadraturePosition()); // Calculate PID error

            if (Math.abs(posError) < Config.getNumber("pidError", 0.05) * Config.getNumber("loadEncoderScale", 3413)) {

                is5Locked = false;
                load.set(0); // Stop motor after PID exits
                setState(5, State.AVAILABLE); // Set to available
            }
            break;

        default:
            break;
        }
    }

    private void addBall() {
        if (ballCount < 5) {
            ballCount++;
        } else {
            ballCount = 5;
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
                System.out.println("Conveyor timing: " + calTimer.get());
                intake.set(0);
                conveyor.set(0);
                startedCal = false;
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
            }
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

    /**
     * Updates status of all 5 positions and real sensor booleans
     */
    public void update() {
        if (timedLock5) { // When 4 -> 5 and timer on boolean has ended, set 5 to occupied
            occupy5 = false;
            setState(5, State.OCCUPIED);
        }
        pos1 = timedBool1.update(intakeSensor.get(), Config.getNumber("intakeSensorDelay", 0.3));
        pos2 = timedBool2.update(conveyorSensor.get(), Config.getNumber("conveyorSensorDelay", 0.2));
        pos4 = timedBool4.update(preloadSensor.get(), Config.getNumber("preloadSensorDelay", 0.5));
        timedLock5 = timedBool5.update(occupy5, Config.getNumber("lockLoadDelay", 0.84));

        updatePos1();
        updateConveyor();
        updatePos4();
        updatePos5();

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
        SmartDashboard.putBoolean("Preload Sensor", pos4);
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
        SmartDashboard.putBoolean("4 Locked", is4Locked);
        SmartDashboard.putBoolean("5 Locked", is5Locked);

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
}
