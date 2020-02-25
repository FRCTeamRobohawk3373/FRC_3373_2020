package frc.team3373;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.DelayTrueBoolean;

public class Indexer {
    private static Indexer instance;
    private WPI_TalonSRX intake, conveyor, preload, load;
    private DigitalInput intakeSensor, conveyorSensor, preloadSensor;

    private DelayTrueBoolean timedBool1, timedBool2, timedBool4;
    private boolean pos1, pos2, pos4;
    private boolean is4Locked, is5Locked;
    private int ballCount;
    private int preloadPos, loadPos;
    private boolean isShooting = false;

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
        // tboolConveyor = new DelayTrueBoolean();
        pos1 = false;
        pos2 = false;
        pos4 = false;
        is4Locked = false;
        is5Locked = false;

        ballCount = 0;

        //!
        preload.configFactoryDefault();
        load.configFactoryDefault();
        //!

        preload.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        preload.setSensorPhase(false); // (was set to true) TODO Change?
        preload.setInverted(false);
        preload.config_kP(0, Config.getNumber("preload_P", 5));
        preload.config_kI(0, 0);
        preload.config_kD(0, 0);
        preload.configClosedLoopPeakOutput(0, Config.getNumber("maxPreloadSpeed", 0.7));
        preload.configAllowableClosedloopError(0,
                (int) (Config.getNumber("pidError", 0.1) * Config.getNumber("encoderScale", 1992)));
        preload.setNeutralMode(NeutralMode.Brake);
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
                (int) (Config.getNumber("pidError", 0.05) * Config.getNumber("encoderScale", 1992)));
        load.setNeutralMode(NeutralMode.Brake);
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
        return true;
    }

    public void startShooting() {
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
        if (isState(5, State.OCCUPIED) && isShooting) {
            loadPos += Config.getNumber("loadRotations", 2);
            load.set(ControlMode.Position, loadPos * Config.getNumber("loadEncoderScale", 1992));
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

            case OCCUPIED: // Ball is just above the sensor
                // switch (getState(2)) { // Get the state of the motor above
                // case AVAILABLE: // Move ball to position 2 if it's empty
                // setState(1, State.MOVING);
                // setState(2, State.MOVING);
                // conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
                // startIntake();
                // break;

                // case MOVING: // If position 2 is moving,
                // setState(1, State.MOVING);
                // break;

                // case OCCUPIED:
                // if (ballCount >= 5) { // If robot is completely full of balls
                // stopIntake();
                // }
                // break;

                // case REVERSE: // To prevent 2 ball collisions (extrusions) with intake and
                // conveyor belts
                // // moving towards each together
                // stopIntake();
                // }
                if (ballCount >= 5) {
                    stopIntake();
                }
                if (isState(2, State.REVERSE)) {
                    stopIntake();
                }

            case MOVING: // If ball is being pushed to ball position 2
                startIntake();
                if (!pos1) { // If position 1 is empty...
                    setState(1, State.AVAILABLE); // Reset position 1 back to the original state of being ready to
                                                  // recieve a ball
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
                    setState(2, State.MOVING);
                    setState(3, State.MOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
                }
            } else if (isState(2, State.MOVING) && isState(3, State.MOVING)) {
                if (!isState(1, State.MOVING) && !isState(4, State.AVAILABLE) && !pos2) {
                    // ! Check if timing is off: 3 -> 4 before pos2 turns off || 2 -> 3 before pos4
                    // ! turns on
                    setState(2, State.AVAILABLE);
                    setState(3, State.OCCUPIED);
                    conveyor.set(0);
                } else if (!isState(4, State.AVAILABLE) && !pos2) {
                    setState(2, State.AVAILABLE);
                    setState(3, State.OCCUPIED);
                }
            } else if (isState(2, State.AVAILABLE) && isState(3, State.OCCUPIED)) {
                if (isState(4, State.AVAILABLE)) {
                    setState(3, State.MOVING);
                    conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
                }
            } else if (isState(2, State.AVAILABLE) && isState(3, State.MOVING)) {
                if (!isState(4, State.AVAILABLE)) {
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
            } else if (isState(2, State.AVAILABLE) && isState(3, State.OCCUPIED)) { // When conveyor is messed up by shooting
                // ? 1 can't be occupied after shooting
                setState(3, State.REVERSE);
                conveyor.set(-Config.getNumber("conveyorMotorSpeed", 0.7));
            } else if (isState(3, State.REVERSE)) {
                if (pos2) {
                    setState(2, State.OCCUPIED);
                    setState(3, State.AVAILABLE);
                    conveyor.set(0);
                }
            }
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
                        conveyor.set(Config.getNumber("conveyorMotorSpeed"));
                    }
                } else {
                    if (isState(1, State.OCCUPIED)) {
                        switch (getState(3)) {
                            case AVAILABLE:
                                setState(2, State.MOVING);
                                setState(3, State.MOVING);
                                conveyor.set(Config.getNumber("conveyorMotorSpeed"));
                                break;

                            case OCCUPIED:
                                if (isState(4, State.AVAILABLE)) {
                                    setState(2, State.MOVING);
                                    setState(3, State.MOVING);
                                    conveyor.set(Config.getNumber("conveyorMotorSpeed"));
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
                if (timedBool2.update(!conveyorSensor.get(), Config.getNumber("reverseConveyorDelay"))) {// TODO REMOVE
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
                        conveyor.set(Config.getNumber("conveyorMotorSpeed"));
                        setState(3, State.MOVING);
                    }
                } else {
                    if (isState(2, State.AVAILABLE)) {
                        setState(2, State.REVERSE);
                        setState(3, State.REVERSE);
                        conveyor.set(-Config.getNumber("conveyorMotorSpeed"));
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
                    preloadPos += Config.getNumber("preloadRotations", 3);

                    preload.set(ControlMode.Position, preloadPos * Config.getNumber("preloadEncoderScale", 1992));
                } else if (!is4Locked) {
                    preload.set(ControlMode.Position, (preloadPos - Config.getNumber("lockOffsetRotations", 0.1))
                            * Config.getNumber("preloadEncoderScale", 1992));
                    is4Locked = true;
                }
                break;

            case MOVING:
                double posError = preloadPos * Config.getNumber("preloadEncoderScale", 1992)
                        - Math.abs(preload.getSensorCollection().getQuadraturePosition());

                if (Math.abs(posError) < Config.getNumber("pidError", 0.05) * Config.getNumber("preloadEncoderScale", 1992)) {
                    preload.set(0);
                    setState(4, State.AVAILABLE);
                    setState(5, State.OCCUPIED);
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
                    load.set(ControlMode.Position, (loadPos - Config.getNumber("lockOffsetRotations", 0.1))
                            * Config.getNumber("loadEncoderScale", 1992));
                    is5Locked = true;
                }
                break;

            case MOVING:
                double posError = loadPos * Config.getNumber("loadEncoderScale", 1992)
                        - Math.abs(load.getSensorCollection().getQuadraturePosition());

                if (Math.abs(posError) < Config.getNumber("pidError", 0.05) * Config.getNumber("loadEncoderScale", 1992)) {

                    is5Locked = false;
                    load.set(0);
                    setState(5, State.AVAILABLE);
                }
                break;

            default:
                break;
        }
    }

    public void zeroMotor() {

    }

    private void addBall() {
        if (ballCount <= 5) {
            ballCount++;
        } else {
            ballCount = 5;
        }
    }

    public void updateTest() {
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
        pos1 = timedBool1.update(intakeSensor.get(), Config.getNumber("intakeSensorDelay", 0.5));
        pos2 = timedBool2.update(conveyorSensor.get(), Config.getNumber("conveyorSensorDelay", 0.5));
        pos4 = timedBool4.update(preloadSensor.get(), Config.getNumber("preloadSensorDelay", 0.5));

        updatePos1();

        // if (isShooting) {
        updateConveyor();
        // } else {
        //updatePos2();
        //updatePos3();
        // }

        updatePos4();
        updatePos5();

        // ! testing code
        displayData();
    }

    public void displayData() {
        SmartDashboard.putBoolean("Intake Sensor", pos1);//? Do we need this code after debugging?
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
        SmartDashboard.putNumber("Preload Pos Encoder Units", preloadPos * Config.getNumber("encoderScale", 1992));
        SmartDashboard.putBoolean("4 Locked", is4Locked);
        SmartDashboard.putBoolean("5 Locked", is5Locked);

        SmartDashboard.putBoolean("isShooting", isShooting);


        SmartDashboard.putNumber("Preload Speed", preload.get());
        SmartDashboard.putNumber("Load Speed", load.get());

        SmartDashboard.putNumber("Preload Quad", preload.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber("Load Quad", load.getSensorCollection().getQuadraturePosition());
        // SmartDashboard.putNumber("Preload Pulse", preload.getSensorCollection().getPulseWidthPosition());
        // SmartDashboard.putNumber("Load Pulse", load.getSensorCollection().getPulseWidthPosition());

    }
}
