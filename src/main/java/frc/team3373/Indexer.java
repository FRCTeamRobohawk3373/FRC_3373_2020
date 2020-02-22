package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.team3373.util.DelayTrueBoolean;

public class Indexer {
    private static Indexer instance;
    private WPI_TalonSRX intake, conveyor, preload, load;
    private DigitalInput intakeSensor, conveyorSensor, preloadSensor;

    private DelayTrueBoolean timedBool1, timedBool2, timedBool4, tboolConveyor;
    private boolean pos1, pos2, pos4;
    private int ballCount;

    public enum State {
        AVAILABLE, OCCUPIED, MOVING, UNMOVE
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
        tboolConveyor = new DelayTrueBoolean();
        pos1 = false;
        pos2 = false;
        pos4 = false;

        ballCount = 0;

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
        setState(5, State.MOVING);
        ballCounter --;// Decrement ball counter
        // ! move motor
    }

    public void startIntake() {
        if (!isState(1, State.OCCUPIED)) {
            setState(1, State.MOVING);
            intake.set(Config.getNumber("intakeMotorSpeed"));
        }
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void moveConveyor() {
        // setState(2, State.MOVING);
        // setState(3, State.MOVING);
        conveyor.set(Config.getNumber("conveyorMotorSpeed"));
    }

    public void moveConveyorCounterAdvance() {// ! This might cause problems because this is moving backwards
        // setState(2, State.MOVING);
        // setState(3, State.MOVING);
        conveyor.set(-Config.getNumber("conveyorMotorSpeed"));
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
                    ballCount ++;// Increment ball counter
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

                    case UNMOVE:// To prevent 2 ball collisions (extrusions) with intake and conveyor belts moving towards each together
                        stopIntake();
                }

            case MOVING:// If ball is being pushed to ball position 2
                if (!pos1) {// If position 1 is empty...
                    setState(1, State.AVAILABLE);// Reset position 1 back to the original state of being ready to recieve a ball
                }
                break;

            default:
                break;
        }
            

        // if (isState(1, State.OCCUPIED) && )
        

        // if(isState(1, State.OCCUPIED) && isState(2, State.AVAILABLE)){
        //     //turn on conveyor
        //     setState(1, State.MOVING);
        //     setState(2, State.MOVING);
        // }
            
        /**
        switch (getState(1)) {
            case AVAILABLE:
                if (timedBool1.update(intakeSensor.get(), Config.getNumber("...", 0.5))) {
                    startIntake();
                }
                // Intentionally no break statement

            case OCCUPIED:
                if (isState(2, State.OCCUPIED) && isState(2, State.OCCUPIED)) {
                    stopIntake();
                }
                break;

            case MOVING:
                if (tboolConveyor.update(true, Config.getNumber("advanceConveyorDuration"))) {
                    reloadIntake();
                }
                break;
        }*///! Delete? 
    }

    public void updatePos2() {
        switch (getState(2)) {
            case AVAILABLE:
                if (isState(2, State.OCCUPIED)) {
                    setState(2, State.OCCUPIED);
                }
                // Intentionally no break statement

            case OCCUPIED:
                if (isState(1, State.OCCUPIED) && isState(2, State.OCCUPIED)) {
                    if (isState(3, State.OCCUPIED) && isState(4, State.OCCUPIED)) {
                        moveConveyor();
                    } else if (isState(3, State.AVAILABLE) || isState(3, State.MOVING)) {
                        moveConveyor();
                    }
                }
                break;

            case MOVING:
                if (tboolConveyor.update(true, Config.getNumber("advanceConveyorDuration"))) {
                    reloadConveyor();
                }
                break;
            }

            default:
                break;
    }

    public void updatePos3() {
        switch (getState(3)) {
            case AVAILABLE:
                break;

            case OCCUPIED:
                if (isState(2, State.AVAILABLE) && isState(4, State.OCCUPIED)) {
                    moveConveyorCounterAdvance();// Counter-advance
                }
                break;

            case MOVING:
                // Already handled in updatePos2's MOVING case
                break;
            
            case UNMOVE:
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
        pos1 = timedBool1.update(intakeSensor.get(), 0.5);
        pos2 = timedBool2.update(conveyorSensor.get(), 0.5);
        pos4 = timedBool4.update(preloadSensor.get(), 0.5);
        updatePos1();
        updatePos2();
        updatePos3();
        updatePos4();
        updatePos5();
    }
}
