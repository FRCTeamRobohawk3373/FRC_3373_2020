package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;


public class Intake {
    private static Intake instance;
    private WPI_TalonSRX intake, conveyor;
    private DigitalInput intakeBallSensor, conveyorBallSensor;

    private int numBalls;

    private enum State {
        OCCUPIED, ADVANCING, AVAILABLE
    }

    private State state1 = State.AVAILABLE;
    private State state2 = State.AVAILABLE;
    private State state3 = State.AVAILABLE;

    long whenTimerExpires = 0;
    boolean p3, p4;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake(Constants.INTAKE_INDEX, Constants.CONVEYOR_INDEX,
                Constants.INTAKE_BALL_SENSOR_INDEX, Constants.CONVEYOR_BALL_SENSOR_INDEX);
        }
        return instance;
    }

    public Intake(int intakeIndex, int conveyorIndex, int intakeBallSensorIndex, int conveyorBallSensorIndex) {
        intake = new WPI_TalonSRX(intakeIndex);
        conveyor = new WPI_TalonSRX(conveyorIndex);
        intake.setNeutralMode(NeutralMode.Brake);
        conveyor.setNeutralMode(NeutralMode.Brake);

        intakeBallSensor = new DigitalInput(intakeBallSensorIndex);
        conveyorBallSensor = new DigitalInput(conveyorBallSensorIndex);
    }

    public void updateLoading() {
        // Sensor logic
        if (intakeBallSensor.get()) {
            state1 = State.OCCUPIED;
        }
        // if (conveyorBallSensor.get()) {
        //     state2 = State.OCCUPIED;
        // }

        if (state1 == State.ADVANCING || state2 == State.ADVANCING || state3 == State.ADVANCING) {
            updateAdvancers();
            return;
        }


        //!

        if (state1 == State.OCCUPIED && state2 == State.AVAILABLE && state3 == State.AVAILABLE) {
            //* 1 -> 2 = 2
            // Intake is moving
            state1 = State.ADVANCING;
        }

        if (state1 == State.AVAILABLE && state2 == State.OCCUPIED && state3 == State.AVAILABLE) {
            //* Do nothing
            // Intake is moving
        }

        if (state1 == State.OCCUPIED && state2 == State.OCCUPIED && state3 == State.AVAILABLE) {
            //* 2 -> 3 && 1 -> 2 = 2,3
            // Intake is moving
            state1 = State.ADVANCING;
            state2 = State.ADVANCING;
        }

        //! Can be counteradvanced
        if (state1 == State.AVAILABLE && state2 == State.AVAILABLE && state3 == State.OCCUPIED) {
            if (Indexer.getInstance().isAvailable()) {
                state3 = State.ADVANCING;
            }
            // Intake is moving
        }

        //! Can be counteradvanced
        if (state1 == State.OCCUPIED && state2 == State.AVAILABLE && state3 == State.OCCUPIED) {
            //* Wait for 4 to be empty
            // Intake is stopped
        }

        if (state1 == State.AVAILABLE && state2 == State.OCCUPIED && state3 == State.OCCUPIED) {
            //* Wait for 4 and 5 to be empty
            // Intake is moving
        }

        if (state1 == State.OCCUPIED && state2 == State.OCCUPIED && state3 == State.OCCUPIED) {
            //* Wait for 4 to be empty
            // Intake is stopped
        }

        //!
    }

    public void updateShooting() {
        
    }

    private void updateAdvancers() {

    }

    public void stopIntake() {

    }

    public void pushBall12() {

    }

    public void pushBall23() {

    }

    public void kyleSolve() {
        long timeNow = System.currentTimeMillis();
        if (timeNow > whenTimerExpires) {
            whenTimerExpires = timeNow + 500l;//* 0.5 seconds

            int[] key = new int[]{2,2,3,2,3,2,2,3,3,3,0,3,3,3,0};
            /*
                ( )[  ]
                ( )[  ]O
                ( )( O)
                ( )[ O]O *counter-advance
                ( )(O )
                ( )[O ]O
                ( )(OO)
                ( )[OO]O
                (O)(  )
                (O)(  )O
                (O)( O)
                [O][ O]O *counter-advance
                (O)(O )
                (O)(O )O
                (O)(OO)
                [O][OO]O

                () = moving
                [] = stop

            */

            boolean p1 = intakeBallSensor.get();
            boolean p2 = conveyorBallSensor.get();
            boolean p4 = Indexer.getInstance(). //TODO get indexer's preload state
            int index = (p1?8:0)+(p2?4:0)+(p3?2:0)+(p4?1:0);




            switch (key[index]) {
                case 0:
                    intake.set(0);
                    conveyor.set(0);
                    break;

                case 1:
                    intake.set(1);
                    conveyor.set(0);
                    break;

                case 2:
                    intake.set(0);
                    conveyor.set(1);
                    p3 = p2;
                    break;

                case 3:
                    intake.set(1);
                    conveyor.set(1);
                    p3 = p2;
                    break;
            }
        }
    }
}