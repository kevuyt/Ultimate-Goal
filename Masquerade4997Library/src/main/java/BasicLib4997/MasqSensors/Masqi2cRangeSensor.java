package BasicLib4997.MasqSensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import BasicLib4997.MasqMotors.TankDrive.TankDrive;


public class Masqi2cRangeSensor {

    byte[] range1Cache;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14);
    public static final int RANGE1_REG_START = 0x04;
    public static final int RANGE1_READ_LENGTH = 2;

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    String nameRangeSensor;
    public Masqi2cRangeSensor(String name){
        this.nameRangeSensor = name;
        RANGE1 = FtcOpModeRegister.opModeManager.getHardwareMap().i2cDevice.get("rangeSensor");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
    }

    public double Ultrasonic() {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        return range1Cache[0] & 0xFF;
    }
    public double ODS () {
        return range1Cache[1] & 0xFF;
    }
    public void telemetryRun () {
        TankDrive.getTelemetry().addTelemetry(nameRangeSensor + " :telemetry");
        TankDrive.getTelemetry().addTelemetry("raw ultrasonic", Ultrasonic());
        TankDrive.getTelemetry().addTelemetry("cm optical", ODS());
    }

}
