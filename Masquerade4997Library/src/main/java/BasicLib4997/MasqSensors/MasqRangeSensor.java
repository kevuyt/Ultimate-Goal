package BasicLib4997.MasqSensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import BasicLib4997.MasqHardware;
import BasicLib4997.MasqMotors.TankDrive.MasqRobot;

/**
 * Created by Archish on 10/28/16.
 */

public class MasqRangeSensor implements MasqHardware{
    ModernRoboticsI2cRangeSensor rangeSensor;
    String nameRangeSensor;
    public MasqRangeSensor(String name){
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
        MasqRobot.getTelemetry().addTelemetry(nameRangeSensor + "telemetry");
        MasqRobot.getTelemetry().addTelemetry("raw ultrasonic", rangeSensor.rawUltrasonic());
        MasqRobot.getTelemetry().addTelemetry("raw optical", rangeSensor.rawOptical());
        MasqRobot.getTelemetry().addTelemetry("cm optical", rangeSensor.cmOptical());
        MasqRobot.getTelemetry().addTelemetry("cm", rangeSensor.getDistance(DistanceUnit.CM));
    }

    public String getName() {
        return nameRangeSensor;
    }

    public String getDash() {
        return "Raw UltraSonic" + Double.toString(rawUltrasonic());
    }
}
