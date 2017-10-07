package Library4997.MasqExternal;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * Created by Archish on 10/7/17.
 */
@I2cSensor(name = "MasqColorSensor", description = "I2C Sensor that supports color number", xmlTag = "ColorSensor")
public class MasqColorSensorV2Driver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final int
            READ_WINDOW_START = 0x04,
            READ_WINDOW_LENGTH = 5;

    public void write(int reg, int value) {
        deviceClient.write(reg, TypeConversion.shortToByteArray((short) value));
    }

    public short read(int reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg, 2));
    }

    protected MasqColorSensorV2Driver(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    public MasqColorSensorV2Driver(I2cDeviceSynch deviceClient, int read) {
        super(deviceClient, true);
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(I2cAddr.create8bit(read));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow() {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(READ_WINDOW_START, READ_WINDOW_LENGTH, I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    @Override
    protected boolean doInitialize() {
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
