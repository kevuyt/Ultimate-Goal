package MasqueradeLibrary.MasqSensors;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqIRSeeker {
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
        I2cDevice irSeeker = hardwareMap.i2cDevice.get(name);
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

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nDirection @ 600: %d\nSignal @ 600: %d\nDirection @ 1200: %d\nSignal @ 1200: %d", name, direction600(), signal600(), direction1200(), signal1200());
    }
}