package frc.team3373;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualInput {

    private DigitalInput[] digins;
    private Map<Integer, Integer> map;

    public ManualInput() {
        digins = new DigitalInput[7];

        for (int i = 0; i < 7; i ++) {
          digins[i] = new DigitalInput(i);
        }

        map = new HashMap<Integer, Integer>();
        map.put(12, 0);
        map.put(14, 1);
        map.put(15, 2);
        map.put(13, 3);
        map.put(9, 4);
        map.put(11, 5);
        map.put(3, 6);
        map.put(1, 7);
        map.put(5, 8);
        map.put(7, 9);
        map.put(6, 10);
        map.put(4, 11);
        map.put(0, 12);
        map.put(2, 13);
        map.put(10, 14);
        map.put(8, 15); 
    }

    public int getClockNumber() {
        int num = (digins[0].get() ? 1 : 0) + (digins[1].get() ? 2 : 0) + (digins[2].get() ? 4 : 0) + (digins[3].get() ? 8 : 0);
        num = map.get(num);
        return num;
    }

    public boolean leftSwitchOn() {
        return !digins[6].get();
    }

    public boolean rightSwitchOn() {
        return !digins[5].get();
    }

    public boolean rightSwitchToggle() {
        return !digins[4].get();
    }

    public void displayOnShuffleboard() {
        SmartDashboard.putNumber("Clock input", getClockNumber());
        SmartDashboard.putBoolean("Left switch", leftSwitchOn());
        SmartDashboard.putBoolean("Right switch", rightSwitchOn());
        SmartDashboard.putBoolean("Right toggle", rightSwitchToggle());
    }
}