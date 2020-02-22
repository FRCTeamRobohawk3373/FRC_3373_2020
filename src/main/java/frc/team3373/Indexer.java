package frc.team3373;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
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

        preload.config_kP(0, Config.getNumber("preload_P", 5));
        preload.config_kI(0, 0);
        preload.config_kD(0, 0);
        preload.configClosedLoopPeakOutput(0, Config.getNumber("maxPreloadSpeed", 0.7));
        preload.configAllowableClosedloopError(0,
                (int) (Config.getNumber("pidError", 0.1) * (int) Config.getNumber("encoderScale", 1992)));

        load.config_kP(0, Config.getNumber("preload_P", 5));
        load.config_kI(0, 0);
        load.config_kD(0, 0);
        load.configClosedLoopPeakOutput(0, Config.getNumber("maxLoadSpeed", 0.7));
        load.configAllowableClosedloopError(0,
                (int) (Config.getNumber("pidError", 0.1) * (int) Config.getNumber("encoderScale", 1992)));
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
        return true;
    }

    /**
     * Called by Shooter.java to remove the 5th ball position.
     */
    public void unloadBall5() {
        loadPos++;
        preload.set(ControlMode.Position, loadPos * Config.getNumber("encoderScale", 1992));
        setState(5, State.MOVING);
        ballCount--;// Decrement ball counter
        // ! move motor
    }

    public void startIntake() {
        if (!isState(1, State.OCCUPIED)) {
            setState(1, State.MOVING);
            intake.set(Config.getNumber("intakeMotorSpeed", 0.6));
        }
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void moveConveyor() {
        // setState(2, State.MOVING);
        // setState(3, State.MOVING);
        conveyor.set(Config.getNumber("conveyorMotorSpeed", 0.7));
    }

    public void moveConveyorCounterAdvance() {// ! This might cause problems because this is moving backwards
        // setState(2, State.MOVING);
        // setState(3, State.MOVING);
        conveyor.set(-Config.getNumber("conveyorMotorSpeed", 0.7));
    }

    public void reloadConveyor() {
        // setState(2, State.AVAILABLE);
        // setState(3, State.AVAILABLE);
        conveyor.set(0);
    }

    public void updatePos1() {
        switch (getState(1)) {
        case AVAILABLE:// Ready to receive a ball
            if (pos1) {// If color sensor detects a new cal
                setState(1, State.OCCUPIED);// Set position 1 state to OCCUPIED
                ballCount++;// Increment ball counter
            }
            break;

        case OCCUPIED:// Ball is just above the sensor
            switch (getState(2)) {// Get the state of the motor above
            case AVAILABLE:// Move ball to position 2 if it's empty
                setState(1, State.MOVING);
                setState(2, State.MOVING);
                moveConveyor();
                startIntake();
                break;

            case MOVING:// If position 2 is moving,
                setState(1, State.MOVING);
                break;

            case OCCUPIED:
                if (ballCount >= 5) {// If robot is completely full of balls
                    stopIntake();
                }
                break;

            case REVERSE:// To prevent 2 ball collisions (extrusions) with intake and conveyor belts
                         // moving towards each together
                stopIntake();
            }

        case MOVING:// If ball is being pushed to ball position 2
            if (!pos1) {// If position 1 is empty...
                setState(1, State.AVAILABLE);// Reset position 1 back to the original state of being ready to recieve a
                                             // ball
            }
            break;

        default:
            break;
        }
    }

    public void updatePos2() {
        switch (getState(2)) {
            case AVAILABLE:
                if (pos2) {
                    setState(2, State.OCCUPIED);
                    conveyor.stopMotor();
                }
                break;
     
            case OCCUPIED:
                if (isState(1, State.OCCUPIED)) {//
                    switch(getState(3)){
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
                break;

            case MOVING:
                if (!pos2) {
                    setState(2, State.AVAILABLE);//! look at later
                }
                break;
            
            case REVERSE:
                if (timedBool2.update(conveyorSensor.get(), Config.getNumber("reverseConveyorDelay"))) {
                    setState(2, State.OCCUPIED);//! look at later
                    conveyor.stopMotor();
                }
                break;

            default:
                break;
        }
    }

    public void updatePos3() {
        switch (getState(3)) {
        case OCCUPIED:
            if (isState(2, State.AVAILABLE)) {
                setState(2, State.REVERSE);
                setState(3, State.REVERSE);
                conveyor.set(-Config.getNumber("conveyorMotorSpeed"));
            }
            break;

        case REVERSE:
            if (isState(2, State.OCCUPIED)) {
                setState(3, State.AVAILABLE);
            }
            break;

        default:
            break;
        }
    }

    public void updatePos4() {
        switch (getState(4)) {
        case AVAILABLE:
            if (pos4) {
                setState(4, State.OCCUPIED);
            }
            break;

        case OCCUPIED:
            if (isState(5,  State.AVAILABLE)) {
                setState(4, State.MOVING);
                is4Locked=false;
                preloadPos += Config.getNumber("preloadRotations", 3);
                preload.set(ControlMode.Position, preloadPos * Config.getNumber("encoderScale", 1992));
            } else if (!is4Locked) {
                preload.set(ControlMode.Position,
                        preload.getSensorCollection().getQuadraturePosition() - Config.getNumber("lockOffsetRotations", 0.1)*Config.getNumber("encoderScale", 1992));
                is4Locked=true;
            }
            break;

        case MOVING:
            if (Math.abs(preload.getClosedLoopError(0)) < Config.getNumber("pidError", 0.1)
                    * Config.getNumber("encoderScale", 1992)) {
                preload.stopMotor();
                setState(4, State.AVAILABLE);
                setState(5, State.OCCUPIED);
            }
            break;

        default:
            break;
        }
    }

    public void updatePos5() {

        switch (getState(5)) {
        case OCCUPIED:
            if (!is5Locked) {
                load.set(ControlMode.Position,
                        load.getClosedLoopTarget(0) - Config.getNumber("lockOffsetRotations", 0.1));
                is5Locked = true;
            }

        case MOVING:
            is5Locked = false;
            if (Math.abs(load.getClosedLoopError(0)) < Config.getNumber("pidError", 0.1)
                    * Config.getNumber("encoderScale", 1992)) {
                load.stopMotor();
                setState(5, State.AVAILABLE);
            }
            break;

        default:
            break;
        }
    }

    public void zeroMotor() {

    }

    /**
     * Updates status of all 5 positions and real sensor booleans
     */
    public void update() {
        pos1 = timedBool1.update(intakeSensor.get(), Config.getNumber("intakeSensorDelay", 0.5));
        pos2 = timedBool2.update(conveyorSensor.get(), Config.getNumber("conveyorDelay"));
        pos4 = timedBool4.update(preloadSensor.get(), Config.getNumber("preloadDelay"));

        updatePos1();
        updatePos2();
        if (!false) {// TODO If not shooting
            updatePos3();
        }
        updatePos4();
        updatePos5();
    }
}
