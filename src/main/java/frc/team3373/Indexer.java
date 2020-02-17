package frc.team3373;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;

public class Indexer {
    private WPI_TalonSRX intake, conveyor;
    private LoadingMotor preload, load;
    private DigitalInput distanceSensor;
    private boolean manual;

    public enum Motors {
        INTAKE, CONVEYOR, PRELOAD, LOAD
    }

    /**
     * Initializes the indexer for the intake and storage of balls.
     * @param intakeIndex Motor index for the intake
     * @param conveyorIndex Motor index for the conveyor
     * @param preloadIndex Motor index for the preloader
     * @param loadIndex Motor index for the loader
     */
    public Indexer(int intakeIndex, int conveyorIndex, int preloadIndex, int loadIndex) {
        intake = new WPI_TalonSRX(intakeIndex);
        //conveyor = new WPI_TalonSRX(conveyorIndex);
        preload = new LoadingMotor(preloadIndex);
        load = new LoadingMotor(loadIndex);
        preload.setNeutralMode(NeutralMode.Brake);
        load.setNeutralMode(NeutralMode.Brake);

        distanceSensor = new DigitalInput(7);

        //manual = false;
        //SmartDashboard.putBoolean("Manual", manual);
    }

    // TODO Remove unneeded constructor
    public Indexer(int preloadIndex, int loadIndex) {
        this(0, 0, preloadIndex, loadIndex);
    }

    /**
     * Runs at every refresh. Updates motor loops and SmartDashboard values for the motors.
     */
    public void update() {
        SmartDashboard.putBoolean("Sensor", distanceSensor.get());
        SmartDashboard.putBoolean("Preload Stopping", preload.stopping);
        SmartDashboard.putBoolean("Preload Running", preload.running);

        //!
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

    public boolean getManual() {
        return manual;
    }

    private WPI_TalonSRX getMotor(Motors motor) {
        switch (motor) {
        case INTAKE:
            return intake;
        case CONVEYOR:
            //return conveyor;
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
    //     manual = !manual;
    //     if (manual) {
    //         preload.setNeutralMode(NeutralMode.Coast);
    //         load.setNeutralMode(NeutralMode.Coast);
    //     }
    //     SmartDashboard.putBoolean("Manual", manual);
    //     return manual;
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
        private int predeadband = 50;
        private int deadband = 20; // Range of error in degrees that is acceptable

        /**
         * Class used by the preloading and loading motors for the Indexer class.
         * 
         * @param index Index of the motor
         */
        public LoadingMotor(int index) {
            super(index);
            running = false;
            scale = (int) Config.getNumber("encoderScale", 1992); // Number of encoder units per rotation

            super.configOpenloopRamp(0); // Set ramp rate to 0
        }

        /**
         * Starts the motor running at the set speed.
         * 
         * @return The speed that the motor is set to.
         */
        private double run() {
            running = true;
            speed = Config.getNumber("indexerSpeed", -1);
            super.set(-1);
            return speed;
        }

        /**
         * Starts the stopping routine for the motor.
         */
        private void stop() {
            running = false;
            stopping = true;
            // scale = (int) Config.getNumber("encoderScale", 1992);
            //super.set(-(scale * 0.1) / getRelPos());
        }

        /**
         * Updates every time that the code runs. Updates the stopping routine.
         */
        private void update() {
            if (stopping) {
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
