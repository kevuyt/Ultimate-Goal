package Library4997.MasqExternal.MasqSensorDrivers;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import Library4997.MasqExternal.MasqHardware;

/**
 * Created by Archish on 10/7/17.
 */

@SuppressWarnings({"WeakerAccess", "unused"})
@I2cSensor(name = "MasqColorSensor", description = "I2C Sensor that supports color number", xmlTag = "MasqColorSensor")
public class MasqColorSensorDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> implements MasqHardware {
    private static final int
            READ_WINDOW_START = 0x04,
            READ_WINDOW_LENGTH = 5;
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

    public void write(int reg, int value) {
        deviceClient.write(reg, TypeConversion.shortToByteArray((short) value));
    }

    public short read(int reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg, 2));
    }

    private MasqColorSensorDriver(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    private MasqColorSensorDriver(I2cDeviceSynch deviceClient, int read) {
        super(deviceClient, true);
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(I2cAddr.create8bit(read));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    private void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(READ_WINDOW_START, READ_WINDOW_LENGTH, I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }
    public void setEngage () {
        engage();
    }

    public int colorNumber() {return read(COLOR_NUMBER_REGISTER);}
    public void setActiveMode() {write(COMMAND_REGISTER, ACTIVE_MODE_COMMAND);}
    public void setPassiveMode() {write(COMMAND_REGISTER, PASSIVE_MODE_COMMAND);}

    public int red() {return read(RED_VALUE_REGISTER);}
    public int green() {return read(GREEN_VALUE_REGISTER);}
    public int blue() {return read(BLUE_VALUE_REGISTER);}
    public int alpha() {return read(WHITE_VALUE_REGISTER);}

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
        return "Sensor";
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
    @Override
    public boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "MASQ COLOR SENSOR";
    }
}
