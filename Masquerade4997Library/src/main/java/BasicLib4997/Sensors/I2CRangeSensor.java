package BasicLib4997.Sensors;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import BasicLib4997.Motors.TankDrive.TankDrive;


public class I2CRangeSensor{

    byte[] range1Cache;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14);
    public static final int RANGE1_REG_START = 0x04;
    public static final int RANGE1_READ_LENGTH = 2;

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    String nameRangeSensor;
    public I2CRangeSensor(String name){
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
