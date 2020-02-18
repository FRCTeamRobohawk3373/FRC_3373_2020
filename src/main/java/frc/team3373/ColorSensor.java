package frc.team3373;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Provides easy output from the revrobotics.ColorSensorV3 color sensor.
 */
public class ColorSensor {

    private final I2C.Port i2cPort;
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    private final Color WHEEL_COLOR_RED, WHEEL_COLOR_YELLOW, WHEEL_COLOR_GREEN, WHEEL_COLOR_BLUE;
    private Color detectedColor;

    public ColorSensor() {
        i2cPort = I2C.Port.kOnboard;
        m_colorSensor = new ColorSensorV3(i2cPort);
        m_colorMatcher = new ColorMatch();
        WHEEL_COLOR_RED =    ColorMatch.makeColor(0.51, 0.35, 0.14);
        WHEEL_COLOR_YELLOW = ColorMatch.makeColor(0.31, 0.59, 0.13);
        WHEEL_COLOR_GREEN =  ColorMatch.makeColor(0.17, 0.58, 0.26);
        WHEEL_COLOR_BLUE =   ColorMatch.makeColor(0.19, 0.43, 0.46);
        m_colorMatcher.addColorMatch(WHEEL_COLOR_RED);
        m_colorMatcher.addColorMatch(WHEEL_COLOR_YELLOW);
        m_colorMatcher.addColorMatch(WHEEL_COLOR_GREEN);
        m_colorMatcher.addColorMatch(WHEEL_COLOR_BLUE);
    }
    
    public enum WheelColor {
        RED,
        YELLOW,
        GREEN,
        BLUE,
        UNKNOWN
    }

    public double getR() {
        return m_colorSensor.getRed();
    }

    public double getG() {
        return m_colorSensor.getGreen();
    }

    public double getB() {
        return m_colorSensor.getBlue();
    }

    public WheelColor getWheelColor() {
        detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == WHEEL_COLOR_RED) {
          return WheelColor.RED;
        } else if (match.color == WHEEL_COLOR_YELLOW) {
            return WheelColor.YELLOW;
        } else if (match.color == WHEEL_COLOR_GREEN) {
            return WheelColor.GREEN;
        } else if (match.color == WHEEL_COLOR_BLUE) {
            return WheelColor.BLUE;
        } else {
            return WheelColor.UNKNOWN;
        }
    } 

    public int getIR() {
        return m_colorSensor.getIR();
    }

    public int getProximity() {
        return m_colorSensor.getProximity();
    }
}