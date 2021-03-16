package MasqueradeLibrary.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqVoltageSensor {
    VoltageSensor sensor;
    double sum, count;

    public MasqVoltageSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void update() {sum += getVoltageInstantaneous(); count++; }

    public double getVoltage() {return sum / count;}

    public double getVoltageInstantaneous() {return sensor.getVoltage();}
}