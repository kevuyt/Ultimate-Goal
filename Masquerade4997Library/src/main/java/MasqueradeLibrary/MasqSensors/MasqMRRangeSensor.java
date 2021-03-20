package MasqueradeLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.*;

import static MasqueradeLibrary.MasqResources.MasqUtils.getHardwareMap;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqMRRangeSensor {
    byte[] range1Cache;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14);
    public static final int RANGE1_REG_START = 0x04;
    public static final int RANGE1_READ_LENGTH = 2;
    private int stopThresh = 0;
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    String nameRangeSensor;
    public MasqMRRangeSensor(String name){
        this.nameRangeSensor = name;
        RANGE1 = getHardwareMap().i2cDevice.get(name);
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
    }
    public MasqMRRangeSensor() {new MasqMRRangeSensor("rangeSensor");}
    public void setStopThresh(int thresh) {
        stopThresh = thresh;
    }
    public double ultrasonic() {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        return range1Cache[0] & 0xFF;
    }
    public double ODS () {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        return range1Cache[1] & 0xFF;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nRaw Ultrasonic: %.2f", nameRangeSensor, ultrasonic());
    }
}
