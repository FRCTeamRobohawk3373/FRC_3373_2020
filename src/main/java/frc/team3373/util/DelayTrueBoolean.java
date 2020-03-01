// (modified) Credit to Team 254 https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/util/TimeDelayedBoolean.java

package frc.team3373.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * This class contains a boolean value and a timer. It can set its boolean value and return whether the timer is within
 * a set timeout. This returns true if the stored value is true and the timeout has expired.
 */
public class DelayTrueBoolean {
    private Timer t = new Timer();
    private boolean m_old = false;
    private boolean override = false;

    public boolean update(boolean value, double timeout) {
        // if(override){
        //     if(t.get() >= timeout){
        //         override=false;
        //         return value;
        //     }

        //     return true;
        // }

        // value = m_old || override;

        if (!m_old && value) {
            t.reset();
            t.start();
        }
        m_old = value;
        if (!value) {
            return false;
        }
        return value && t.get() >= timeout;
    }

    public void setImediate(boolean value){
        override=value;
    }
}
