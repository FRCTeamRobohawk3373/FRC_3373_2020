package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.team3373.util.TimedBoolean;

public class Indexer {
    private static Indexer instance;
    private WPI_TalonSRX intake, conveyor, preload, load;
    private DigitalInput intakeSensor, conveyorSensor, preloadSensor;

    private TimedBoolean tbool;

    public enum State {
        OCCUPIED, ADVANCING, AVAILABLE
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
        tbool = new 
    }

    private State getState(int index) {
        return ballStates[index - 1];
    }

    private void setState(int index, State value) {
        ballStates[index - 1] = value;
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

    }

    public void updatePos2() {

    }

    public void updatePos3() {

    }

    public void updatePos4() {

    }

    public void updatePos5() {

    }

    /**
     * Updates status of all 5 positions and real sensor booleans
     */
    public void update() {
        updatePos1();
        updatePos2();
        updatePos3();
        updatePos4();
        updatePos5();
    }
}
