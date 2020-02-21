package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.team3373.util.TimedBoolean;

public class Indexer {
    private static Indexer instance;
    private WPI_TalonSRX intake, conveyor, preload, load;
    private DigitalInput intakeSensor, conveyorSensor, preloadSensor;

    private TimedBoolean tbool1, tbool2, tbool4;
    private boolean bool1, bool2, bool4;

    public enum State {
        AVAILABLE, OCCUPIED, ADVANCING
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
        tbool1 = new TimedBoolean();
        tbool2 = new TimedBoolean();
        tbool4 = new TimedBoolean();
    }

    private boolean isState(int index, State val) {
        return ballStates[index - 1] == val;
    }
    private boolean isState(int index, State val1, State val2) {
        return isState(index, val1) || isState(index, val2);
    }

    private State getState(int index) {
        return ballStates[index - 1];
    }

    private void setState(int index, State val) {
        ballStates[index - 1] = val;
    }

    /**
     * Begins the intake.
     */
    public void beginIntake() {

    }

    /**
     * Stops intake motor.
     */
    public void stopIntake() {

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
        setState(5, State.AVAILABLE);
    }

    public void updatePos1() {
        switch (getState(1)) {
            case AVAILABLE:
                break;

            case OCCUPIED:
                break;

            case ADVANCING:
                break;
        }
    }

    public void updatePos2() {
        switch (getState(2)) {
            case AVAILABLE:
                break;

            case OCCUPIED:
                break;

            case ADVANCING:
                break;
        }
    }

    public void updatePos3() {
        switch (getState(3)) {
            case AVAILABLE:
                break;

            case OCCUPIED:
                break;

            case ADVANCING:
                break;
        }        
    }

    public void updatePos4() {

    }

    public void updatePos5() {

    }

    /**
     * Updates status of all 5 positions and real sensor booleans
     */
    public void update() {
        bool1 = tbool1.update(intakeSensor.get(), 0.5);
        bool2 = tbool2.update(conveyorSensor.get(), 0.5);
        bool4 = tbool4.update(preloadSensor.get(), 0.5);
        updatePos1();
        updatePos2();
        updatePos3();
        updatePos4();
        updatePos5();
    }
}
