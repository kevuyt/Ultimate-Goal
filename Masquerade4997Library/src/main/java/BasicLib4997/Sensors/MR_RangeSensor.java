package BasicLib4997.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Created by Archish on 10/28/16.
 */

public class MR_RangeSensor{
    ModernRoboticsI2cRangeSensor rangeSensor;
    String nameRangeSensor;
    public MR_RangeSensor(String name){
        this.nameRangeSensor = name;
        rangeSensor = FtcOpModeRegister.opModeManager.getHardwareMap().get(ModernRoboticsI2cRangeSensor.class, name);
    }

    public double rawUltrasonic() {
        return rangeSensor.rawUltrasonic();
    }
    public double rawOpitcal() {
        return rangeSensor.rawOptical();
    }
    public double getDistance(DistanceUnit unit) {
        double cmOptical = rangeSensor.cmOptical();
        double cm = cmOptical > 0 ? cmOptical : rawUltrasonic();
        return unit.fromUnit(DistanceUnit.CM, cm);
    }
    public double cmOptical(){
        return  rangeSensor.cmOptical();
    }

    public void telemetryRun () {
        TankDrive.getTelemetry().addTelemetry(nameRangeSensor + "telemetry");
        TankDrive.getTelemetry().addTelemetry("raw ultrasonic", rangeSensor.rawUltrasonic());
        TankDrive.getTelemetry().addTelemetry("raw optical", rangeSensor.rawOptical());
        TankDrive.getTelemetry().addTelemetry("cm optical", rangeSensor.cmOptical());
        TankDrive.getTelemetry().addTelemetry("cm", rangeSensor.getDistance(DistanceUnit.CM));
    }

}
