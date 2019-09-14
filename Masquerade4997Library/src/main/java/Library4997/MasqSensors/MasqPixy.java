package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Archishmaan Peyyety on 9/22/18.
 * Project: MasqLib
 */

public class MasqPixy {
    private I2cDeviceSynch pixy;
    private byte[] data;
    private byte[] out;
    public MasqPixy (HardwareMap hardwareMap) {
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");
        pixy.setI2cAddress(I2cAddr.create7bit(0x54));
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26,
                I2cDeviceSynch.ReadMode.REPEAT);
        pixy.setReadWindow(readWindow);
        pixy.engage();
        data = pixy.read(0, 16);
        out = data;
    }
    public byte[] getOut () {
        return out;
    }
    public void update() {
        data = pixy.read(0, 16);
        if (data[2] == 0x55) out = data;
    }

}
