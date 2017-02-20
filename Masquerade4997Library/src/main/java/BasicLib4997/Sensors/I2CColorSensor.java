package BasicLib4997.Sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.Motors.TankDrive.TankDrive;


/**
 * This is a Modern Robotics Color Sensor.
 * Provides the color number, as well as argb and hsv data.
 * Can detect red, white, and blue, and thresholds for those can be set.
 */

public class I2CColorSensor {

    private final I2cDevice colorSensor;
    private final I2cDeviceSynch colorSensorManager;
    private final String name;

    private float[] hsvValues = new float[3];

    private static final int
            COLOR_NUMBER_REGISTER = 0x04,
            RED_VALUE_REGISTER = 0x05,
            GREEN_VALUE_REGISTER = 0x06,
            BLUE_VALUE_REGISTER = 0x07,
            WHITE_VALUE_REGISTER = 0x08;

    private static final int
            READ_WINDOW_START = COLOR_NUMBER_REGISTER,
            READ_WINDOW_LENGTH = 5;

    private static final int
            COMMAND_REGISTER = 0x03,
            ACTIVE_MODE_COMMAND = 0x00,
            PASSIVE_MODE_COMMAND = 0x01;

    private int
            blueMinThreshold = 1, blueMaxThreshold = 4,
            redMinThreshold = 10, redMaxThreshold = 12,
            whiteMinThreshold = 14, whiteMaxThreshold = 16;


    public I2CColorSensor(String name, int i2cAddress) {
        this.name = name;

        colorSensor = FtcOpModeRegister.opModeManager.getHardwareMap().i2cDevice.get(name);
        colorSensor.resetDeviceConfigurationForOpMode();

        colorSensorManager = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(i2cAddress), false);
        colorSensorManager.resetDeviceConfigurationForOpMode();
        colorSensorManager.engage();
        colorSensorManager.setReadWindow(new I2cDeviceSynch.ReadWindow(READ_WINDOW_START, READ_WINDOW_LENGTH, I2cDeviceSynch.ReadMode.REPEAT));
    }



    public void setActiveMode() {colorSensorManager.write8(COMMAND_REGISTER, ACTIVE_MODE_COMMAND);}
    public void setPassiveMode() {colorSensorManager.write8(COMMAND_REGISTER, PASSIVE_MODE_COMMAND);}

    public int colorNumber() {return colorSensorManager.read8(COLOR_NUMBER_REGISTER);}
    public int red() {return colorSensorManager.read8(RED_VALUE_REGISTER);}
    public int green() {return colorSensorManager.read8(GREEN_VALUE_REGISTER);}
    public int blue() {return colorSensorManager.read8(BLUE_VALUE_REGISTER);}
    public int alpha() {return colorSensorManager.read8(WHITE_VALUE_REGISTER);}

    private void rgb2hsv() {
        byte[] colorSensorCache = colorSensorManager.read(RED_VALUE_REGISTER, 3);
        Color.RGBToHSV(colorSensorCache[0], colorSensorCache[1], colorSensorCache[2], hsvValues);
    }
    public float hue() {rgb2hsv(); return hsvValues[0];}
    public float saturation() {rgb2hsv(); return hsvValues[1];}
    public float value() {rgb2hsv(); return hsvValues[2];}


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

    public boolean detectWhite() {
        int colorNumber = colorNumber();
        return colorNumber >= whiteMinThreshold && colorNumber <= whiteMaxThreshold;
    }

    private double colorAvg () {
        return  (red() + blue() + green() + alpha()) / 4;
    }

    private double colorAvgCN () {
        return  (red() + blue() + green() + alpha() + colorNumber()) / 5;
    }
    private double colorAvgA () {
        return (colorAvg() + colorAvgCN()) / 2;
    }


    public void telemetryRun() {
        TankDrive.getTelemetry().addTelemetry(name, colorNumber());
        TankDrive.getTelemetry().addTelemetry("color Number", colorNumber());
        TankDrive.getTelemetry().addTelemetry("red", red());
        TankDrive.getTelemetry().addTelemetry("alpha", alpha());
        TankDrive.getTelemetry().addTelemetry("blue", blue());
        TankDrive.getTelemetry().addTelemetry("green", green());
        TankDrive.getTelemetry().addTelemetry("hue", hue());
        TankDrive.getTelemetry().addTelemetry("saturation", saturation());
        TankDrive.getTelemetry().addTelemetry("detectBlue", isBlue());
        TankDrive.getTelemetry().addTelemetry("detectWhite", detectWhite());
        TankDrive.getTelemetry().addTelemetry("isRed", isRed());
        TankDrive.getTelemetry().addTelemetry("colorAvg", colorAvg());
        TankDrive.getTelemetry().addTelemetry("colorAvg2", colorAvgCN());
        TankDrive.getTelemetry().addTelemetry("colorAvg3", colorAvgA());
    }


}