package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.TimedBoolean;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;

public class Indexer {
    private WPI_TalonSRX intake; //, conveyor;
    private LoadingMotor preload, load;
    private DigitalInput ballSensor;
    private boolean manual;
    private boolean isAHeld = false;

    private static Indexer instance;

    public enum Motors {
        INTAKE, CONVEYOR, PRELOAD, LOAD
    }
    
    public enum State {
        HOME, ADVANCE, LOCK
    }

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer(Constants.INTAKE_INDEX, Constants.CONVEYOR_INDEX, 
                Constants.PRELOAD_INDEX, Constants.LOAD_INDEX, Constants.BALL_SENSOR_INDEX);
        }
        return instance;
    }


    /**
     * Initializes the indexer for the intake and storage of balls.
     * 
     * @param intakeIndex   Motor index for the intake
     * @param conveyorIndex Motor index for the conveyor
     * @param preloadIndex  Motor index for the preloader
     * @param loadIndex     Motor index for the loader
     */
    public Indexer(int intakeIndex, int conveyorIndex, int preloadIndex, int loadIndex, int ballSensorIndex) {
        intake = new WPI_TalonSRX(intakeIndex);
        // conveyor = new WPI_TalonSRX(conveyorIndex);
        preload = new LoadingMotor(preloadIndex);
        load = new LoadingMotor(loadIndex);
        preload.setNeutralMode(NeutralMode.Brake);
        load.setNeutralMode(NeutralMode.Brake);

        ballSensor = new DigitalInput(ballSensorIndex);

        // manual = false;
        // SmartDashboard.putBoolean("Manual", manual);
    }

    /**
     * Runs at every refresh. Updates motor loops and SmartDashboard values for the
     * motors.
     */
    public void update() {
        SmartDashboard.putBoolean("Sensor", ballSensor.get());
        SmartDashboard.putBoolean("Preload Stopping", preload.stopping);
        SmartDashboard.putBoolean("Preload Running", preload.running);

        intake.set(-0.15);// for testing

        preload.update();
        load.update();
    }

    public boolean toggleRunning(Motors motor) {
        LoadingMotor m = (LoadingMotor) getMotor(motor);
        if (!m.running) {
            m.run();
            return true;
        }
        m.stop();
        return false;
    }

    public void onAHeld(boolean val) {
        isAHeld = val;
    }

    public boolean getManual() {
        return manual;
    }

    private WPI_TalonSRX getMotor(Motors motor) {
        switch (motor) {
        case INTAKE:
            return intake;
        case CONVEYOR:
            // return conveyor;
        case PRELOAD:
            return preload;
        case LOAD:
            return load;
        default:
            return null;
        }
    }

    public int getAbsPos(Motors motor) {
        return ((LoadingMotor) getMotor(motor)).getAbsPos();
    }

    public int getRelPos(Motors motor) {
        return ((LoadingMotor) getMotor(motor)).getRelPos();
    }

    public double getRelDeg(Motors motor) {
        return ((LoadingMotor) getMotor(motor)).getRelDeg();
    }

    // public boolean toggleControl() {
    // manual = !manual;
    // if (manual) {
    // preload.setNeutralMode(NeutralMode.Coast);
    // load.setNeutralMode(NeutralMode.Coast);
    // }
    // SmartDashboard.putBoolean("Manual", manual);
    // return manual;
    // }

    public boolean rotate(Motors motor, double speed) {
        if (RobotState.isTest()) {
            getMotor(motor).set(speed);
            return true;
        }
        return false;
    }

    public void zero() {
        preload.getSensorCollection().setQuadraturePosition(0, 20);
        load.getSensorCollection().setQuadraturePosition(0, 20);
    }

    class LoadingMotor extends WPI_TalonSRX {
        private boolean running;
        private boolean stopping;
        private double speed;
        private int scale;
        private double predeadband;
        private double deadband; // Range of error in degrees that is acceptable

        private State state;
        private boolean isAvailable;
        private TimedBoolean timer;
        private double advanceBallSpeed;
        private double advanceBallDuration;

        /**
         * Class used by the preloading and loading motors for the Indexer class.
         * 
         * @param index Index of the motor
         */
        public LoadingMotor(int index) {
            super(index);
            running = false;
            scale = (int) Config.getNumber("encoderScale", 1992); // Number of encoder units per rotation
            predeadband = Config.getNumber("predeadband", 55);
            deadband = Config.getNumber("deadband", 25);

            state = State.HOME;
            isAvailable = !Config.getBool("ballLoaded", false);

            if (this == preload) {
                advanceBallSpeed = Config.getNumber("advancePreloadSpeed");
                advanceBallDuration = Config.getNumber("advancePreloadDuration");
            } else if (this == load) {
                advanceBallSpeed = Config.getNumber("advanceLoadSpeed");
                advanceBallDuration = Config.getNumber("advanceLoadDuration");
            }
            

            super.configOpenloopRamp(0); // Set ramp rate to 0
        }

        /**
         * Starts the motor running at the set speed.
         * @return The speed that the motor is set to.
         */
        private double run() {
            scale = (int)Config.getNumber("encoderScale", 1992);
            running = true;
            super.set(Config.getNumber("indexerSpeed", -1));
            return speed;
        }

        /**
         * 
         * @return If the motor has an empty ball slot for the previous motor in the intake sequence to reference
         */
        public boolean getAvailable() {
            return isAvailable;
        }

        public void setState(State newState) {
            state = newState;
        }

        /**
         * Starts the stopping routine for the motor.
         */
        private void stop() {
            running = false;
            stopping = true;
            predeadband = Config.getNumber("predeadband", 55);
            deadband = Config.getNumber("deadband", 25);
            scale = (int) Config.getNumber("encoderScale", 1992);
            // super.set(-(scale * 0.1) / getRelPos());
        }

        /**
         * Updates every time that the code runs. Updates the stopping routine.
         */
        private void update() {

            boolean changeState = false;
            switch (state) {
                case HOME:
                    isAvailable = true;

                    // Triggers for a state change
                    if (this == preload) {
                        changeState = timer.update(ballSensor.get(), 0.25);
                    } else if (this == load) {
                        // Important! Only the preload motor can change the state of the load motor
                    }
                    
                    if (changeState) {
                        // Which state to change to?
                        if (this == preload) {
                            if (load.getAvailable()) {// If load motor has empty slot
                                state = State.ADVANCE;
                            } else {
                                state = State.LOCK;
                            }
                        }                  
                        timer.restart();// Reset the all-purpose timer/boolean
                    }

                    break;
                case ADVANCE:
                    isAvailable = true;

                    if (timer.update(true, advanceBallDuration)) {// When push timer expires
         
                        // Which state to change to?
                        if (this == preload) {
                            // When preload motor is done, the ball will have travelled to the load motor position, so set its isAvailable to false
                            ////load.setAvailable(false);
                            if (isAHeld) {
                                load.state = State.ADVANCE; 
                            } else {
                                load.state = State.HOME;
                            }
                            if (load.getAvailable()) {// If load motor has empty slot
                                state = State.HOME;
                                
                            } else {
                                state = State.LOCK;
                            }
                        } else if (this == load) {
                            if (isAHeld) {
                                state = State.ADVANCE; 
                            } else {
                                state = State.HOME;
                            }
                        }           
                        timer.restart();// Reset the all-purpose timer/boolean
                    
                    }
                    break;
                case LOCK:
                    isAvailable = false;

                    // Triggers for a state change
                    if (this == preload) {
                        changeState = load.isAvailable;
                    } else if (this == load) {
                        changeState = isAHeld;
                    }
                    
                    if (changeState) {
                        state = State.ADVANCE;       
                        timer.restart();// Reset the all-purpose timer/boolean
                    }
                    
                    break;
            }

            SmartDashboard.putString("State", state.toString());
            SmartDashboard.putBoolean("isAvailable", isAvailable);
            
//TODO put in case statement
            if (state == State.ADVANCE) {// Set speed
                super.set(advanceBallSpeed);
            } else if (state == State.HOME) {// Go to position
                gotoHomePosition();
            } else if (state == State.LOCK) {
                gotoLockPosition();
            }
        }

        public void gotoHomePosition() {
            double relDeg = getRelDeg();
            if (relDeg > scale - deadband || relDeg < deadband) {
                super.set(0);
                stopping = false;
            } else if (relDeg > 360 - predeadband) {
                super.set(-0.2);
            } else {
                super.set(-1);
            }
        };

        public void gotoLockPosition() {
            double relDeg = getRelDeg();
            if (relDeg > scale - deadband || relDeg < deadband) {
                super.set(0);
                stopping = false;
            } else if (relDeg > 360 - predeadband) {
                super.set(-0.2);
            } else {
                super.set(-1);
            }
        };

        private int getAbsPos() {
            return super.getSensorCollection().getQuadraturePosition();
        }

        private int getRelPos() {
            int relPos = getAbsPos() % scale;
            if (relPos < 0) {
                return scale + relPos;
            }
            return relPos;
        }

        private double getRelDeg() {
            return (getRelPos() * (360.0 / scale));
        }
    }
}
