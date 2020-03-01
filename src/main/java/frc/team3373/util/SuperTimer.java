package frc.team3373.util;

import edu.wpi.first.wpilibj.Timer;


public class SuperTimer extends Timer{
    private double offsetTimer = 0;

    public SuperTimer() {
        super();
    }

    @Override
    public synchronized double get() {
        return super.get()+offsetTimer;
    }

    public synchronized void set(double value) {
        offsetTimer=value;
    }

    @Override
    public synchronized void reset() {
        // TODO Auto-generated method stub
        offsetTimer=0;
        super.reset();
    }

}