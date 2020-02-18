package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.util.TimedBoolean;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class Indexer {
    private static Indexer instance;

    private LoadingMotor preload, load;
    private DigitalInput ballSensor;

    private boolean shooting;
    private int stateCounter = 0;

    public enum Motors {
        PRELOAD, LOAD
    }
    
    public enum State {
        HOME, ADVANCE, LOCK
    }

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer(Constants.PRELOAD_INDEX, Constants.LOAD_INDEX,
                Constants.PRELOAD_BALL_SENSOR_INDEX);
        }
        return instance;
    }


    /**
     * Initializes the indexer for the storage and shooting of balls.
     * 
     * @param preloadIndex    Motor index for the preloader
     * @param loadIndex       Motor index for the loader
     * @param ballSensorIndex DIO port for ball sensor
     */
    public Indexer(int preloadIndex, int loadIndex, int ballSensorIndex) {
        preload = new LoadingMotor(preloadIndex, Motors.PRELOAD);
        load = new LoadingMotor(loadIndex, Motors.LOAD);
        preload.setNeutralMode(NeutralMode.Brake);
        load.setNeutralMode(NeutralMode.Brake);
        
        shooting = false;

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
            
        SmartDashboard.putString("State of Preload", preload.getState().toString());
        SmartDashboard.putString("State of Load", load.getState().toString());


        preload.update();
        load.update();
    }

    public boolean isAvailable() {
        return ((LoadingMotor)getMotor(Motors.PRELOAD)).isOccupied;
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

    public void shoot() {
        if (!shooting)
            shooting = true;
    }

    public void getOccupied() {
        SmartDashboard.putBoolean("Preload Occ", preload.isOccupied);
        SmartDashboard.putBoolean("Load Occ", load.isOccupied);
    }

    public WPI_TalonSRX getMotor(Motors motor) {
        switch (motor) {
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
        public boolean isOccupied;
        private TimedBoolean timer = new TimedBoolean();
        private double advanceBallSpeed;
        private double advanceBallDuration;

        private Motors name;

        private int lockTime;
        private Timer lockTimer;

        /**
         * Class used by the preloading and loading motors for the Indexer class.
         * 
         * @param index Index of the motor
         */
        public LoadingMotor(int index, Motors name) {
            super(index);
            running = false;
            scale = (int) Config.getNumber("encoderScale", 1992); // Number of encoder units per rotation
            predeadband = Config.getNumber("predeadband", 55);
            deadband = Config.getNumber("deadband", 25);
            lockTime = (int) Config.getNumber("lockTime", 0.1);
            lockTimer = new Timer();

            state = State.HOME;
            this.name = name;
            isOccupied = false;

            if (name == Motors.PRELOAD) {
                advanceBallSpeed = Config.getNumber("advancePreloadSpeed", 1);
                advanceBallDuration = Config.getNumber("advancePreloadDuration", 2.1);
            } else if (name == Motors.LOAD) {
                advanceBallSpeed = Config.getNumber("advanceLoadSpeed", 0.5);
                advanceBallDuration = Config.getNumber("advanceLoadDuration", 0.7);
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
        public State getState() {
            return state;
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
                    // Triggers for a state change
                    if (name == Motors.PRELOAD) {
                        changeState = timer.update(ballSensor.get(), 0.5);
                    }

                    if (changeState) {
                        // Which state to change to?
                        if (name == Motors.PRELOAD) {
                            if (load.isOccupied || load.state != State.HOME) { // If load motor has empty slot
                                state = State.LOCK;
                            } else {
                                state = State.ADVANCE;
                                timer = new TimedBoolean();
                            }
                        }
                        
                    }
                    gotoHomePosition();
                    break;
                case ADVANCE:
                    super.set(advanceBallSpeed);
    
                    if (timer.update(true, advanceBallDuration)) {// When push timer expires
                        if (name == Motors.PRELOAD) {
                            state = State.HOME;
                            //!
                            load.state = State.LOCK;
                            load.isOccupied = true;
                            //!
                            isOccupied = false;
                        } else if (name == Motors.LOAD) {
                            state = State.HOME;
                            isOccupied = false;
                        }
                    }
                    break;
                case LOCK:
                    gotoLockPosition();

                    if (name == Motors.PRELOAD) {
                        if (!load.isOccupied) {

                            state = State.ADVANCE;
                            timer = new TimedBoolean();
                        }
                    } else if (name == Motors.LOAD) {
                        if (shooting) {
                            shooting = false;
                            state = State.ADVANCE;
                            timer = new TimedBoolean();
                        }
                    }

                    
                    break;
            }
        }

        public void gotoHomePosition() {
            if (!(state == State.HOME))
                return;
            double relDeg = getRelDeg();
            if (relDeg > scale - deadband || relDeg < deadband) {
                super.set(0);
                stopping = false;
            } else if (relDeg > 360 - predeadband) {
                super.set(-0.2);
            } else {
                super.set(-1);
            }
        }

        public void gotoLockPosition() { // TODO Fix
            if (!(state == State.LOCK))
                return;
            double relDeg = getRelDeg();
            double position = scale - Config.getNumber("loadingMotorLockOffset", 15.0);
            if (relDeg < position) {
                super.set(0);
            } else {
                super.set(0.1);
            }
        }

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
