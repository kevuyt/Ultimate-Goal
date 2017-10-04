package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import Library4997.MasqExternal.MasqHardware;


/**
 * This is a Modern Robotics Color Sensor.
 * Provides the color number, as well as argb and hsv data.
 * Can detect red, white, and blue, and thresholds for those can be set.
 */

public class MasqIRSeeker implements MasqHardware {

    private final I2cDevice irSeeker;
    private final I2cDeviceSynch irSeekerManager;
    private final String name;

    private float[] hsvValues = new float[3];

    private static final int
            DIRECTION_REGISTER_1200 = 0x04,
            SIGNAL_STRENGTH_REGISTER_1200 = 0x05,
            DIRECTION_REGISTER_600 = 0x06,
            SIGNAL_STRENGTH_REGISTER_600 = 0x07;

    private static final int
            READ_WINDOW_START = DIRECTION_REGISTER_1200,
            READ_WINDOW_LENGTH = 5;

    public MasqIRSeeker(String name, int i2cAddress, HardwareMap hardwareMap) {
        this.name = name;
        irSeeker = hardwareMap.i2cDevice.get(name);
        irSeeker.resetDeviceConfigurationForOpMode();
        irSeekerManager = new I2cDeviceSynchImpl(irSeeker, I2cAddr.create8bit(i2cAddress), false);
        irSeekerManager.resetDeviceConfigurationForOpMode();
        irSeekerManager.engage();
        irSeekerManager.setReadWindow(new I2cDeviceSynch.ReadWindow(READ_WINDOW_START, READ_WINDOW_LENGTH, I2cDeviceSynch.ReadMode.REPEAT));
    }
    public int direction1200() {return irSeekerManager.read8(DIRECTION_REGISTER_1200);}
    public int signal1200() {return irSeekerManager.read8(SIGNAL_STRENGTH_REGISTER_1200);}
    public int direction600() {return irSeekerManager.read8(DIRECTION_REGISTER_600);}
    public int signal600() {return irSeekerManager.read8(SIGNAL_STRENGTH_REGISTER_600);}



    public String getName() {
        return name;
    }
    public String[] getDash() {
        return new String[]{
                "Direction @ 600:" + Integer.toString(direction600()),
                "Signal @ 600:" + Integer.toString(signal600()),
                "Direction @ 1200:" + Integer.toString(direction1200()),
                "Signal @ 1200:" + Integer.toString(signal1200())
        };
    }
}