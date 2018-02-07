package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqUtilities.MasqSensorDrivers.MasqColorSensorDriver;
import Library4997.MasqUtilities.MasqHardware;

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
    private MasqColorSensorDriver driver;
    public MasqColorSensorV2 (String name, HardwareMap hardwareMap) {
        this.name = name;
        driver = hardwareMap.get(MasqColorSensorDriver.class, name);
        driver.resetDeviceConfigurationForOpMode();
        driver.setEngage();
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
        String colorString;
        if (value == 1) colorString =  "PURPLE";
        else if (value == 2)  colorString = "DARK BLUE";
        else if (value == 3)  colorString = "LIGHT BLUE";
        else if (value == 4)  colorString = "GREEN BLUE";
        else if (value == 5)  colorString = "GREEN";
        else if (value == 6)  colorString = "GREEN YELLOW";
        else if (value == 7)  colorString = "YELLOW";
        else if (value == 8)  colorString = "YELLOW";
        else if (value == 9)  colorString = "ORANGE YELLOW";
        else if (value == 10)  colorString = "ORANGE";
        else if (value == 11)  colorString = "RED";
        else if (value == 12)  colorString = "PINK";
        else colorString = "UNKNOWN";
        return colorString + ", " + value;
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
