package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * Created by Archishmaan Peyyety on 12/13/18.
 * Project: MasqLib
 */
public class MasqDistanceSensor implements MasqHardware {
    private String name;
    private DistanceSensor distanceSensor;
    public MasqDistanceSensor(String name, HardwareMap hardwareMap) {
        this.name = name;
        distanceSensor = hardwareMap.get(DistanceSensor.class, name);
    }

    public double distance (DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }

}
