package Library4997.MasqSensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqExternal.MasqColorSensorV2Driver;
import Library4997.MasqExternal.MasqHardware;
import Library4997.MasqExternal.MasqSensor;

/**
 * Created by Archish on 10/7/17.
 */

public class MasqColorSensorV2 implements MasqHardware {
    private static final int
            COLOR_NUMBER_REGISTER = 0x04,
            RED_VALUE_REGISTER = 0x05,
            GREEN_VALUE_REGISTER = 0x06,
            BLUE_VALUE_REGISTER = 0x07,
            WHITE_VALUE_REGISTER = 0x08;

    private static final int
            COMMAND_REGISTER = 0x03,
            ACTIVE_MODE_COMMAND = 0x00,
            PASSIVE_MODE_COMMAND = 0x01;

    private int
            blueMinThreshold = 1, blueMaxThreshold = 4,
            redMinThreshold = 10, redMaxThreshold = 12,
            whiteMinThreshold = 14, whiteMaxThreshold = 16;

    String name;
    private MasqColorSensorV2Driver driver;
    public MasqColorSensorV2 (String name, HardwareMap hardwareMap) {
        this.name = name;
        driver = hardwareMap.get(MasqColorSensorV2Driver.class, name);
    }

    public int colorNumber() {return driver.read(COLOR_NUMBER_REGISTER);}
    public void setActiveMode() {driver.write(COMMAND_REGISTER, ACTIVE_MODE_COMMAND);}
    public void setPassiveMode() {driver.write(COMMAND_REGISTER, PASSIVE_MODE_COMMAND);}

    public int red() {return driver.read(RED_VALUE_REGISTER);}
    public int green() {return driver.read(GREEN_VALUE_REGISTER);}
    public int blue() {return driver.read(BLUE_VALUE_REGISTER);}
    public int alpha() {return driver.read(WHITE_VALUE_REGISTER);}

    public void setBlueThresholds(int min, int max) {
        blueMinThreshold = min;
        blueMaxThreshold = max;
    }

    public void setRedThresholds(int min, int max) {
        redMinThreshold = min;
        redMaxThreshold = max;
    }

    public void setWhiteThresholds(int min, int max) {
        whiteMinThreshold = min;
        whiteMaxThreshold = max;
    }

    public boolean isBlue() {
        int colorNumber = colorNumber();
        return colorNumber >= blueMinThreshold && colorNumber <= blueMaxThreshold;
    }

    public boolean isRed() {
        int colorNumber = colorNumber();
        return colorNumber >= redMinThreshold && colorNumber <= redMaxThreshold;
    }
    public boolean isWhite () {
        int colorNumber = colorNumber();
        return colorNumber >= whiteMinThreshold && colorNumber <= whiteMaxThreshold;
    }

    public String color () {
        int value = colorNumber();
        if (value == 1) return "PURPLE, 1";
        else if (value == 2) return "DARK BLUE, 2";
        else if (value == 3) return "LIGHT BLUE, 3";
        else if (value == 4) return "GREEN BLUE, 4";
        else if (value == 5) return "GREEN, 5";
        else if (value == 6) return "GREEN YELLOW, 6";
        else if (value == 7) return "YELLOW, 7";
        else if (value == 8) return "YELLOW, 8";
        else if (value == 9) return "ORANGE YELLOW, 9";
        else if (value == 10) return "ORANGE, 10";
        else if (value == 11) return "RED, 11";
        else if (value == 12) return "PINK, 12";
        return "UNKNOWN";
    }

    public String getName() {
        return name;
    }
    public String[] getDash() {
        return new String[]{
                "Detected Color: " + color(),
                "Color #:" + colorNumber(),
                "Detect White" + isWhite(),
                "Detect Red" + isRed(),
                "Detect Blue" + isBlue()
        };
    }
}
