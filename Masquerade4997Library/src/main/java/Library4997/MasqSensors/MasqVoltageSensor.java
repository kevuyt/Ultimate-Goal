package Library4997.MasqSensors;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public class MasqVoltageSensor {
    VoltageSensor sensor;
    double sum, count;

    public MasqVoltageSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        sum = count = 0;
    }

    public void update() {sum+=getVoltageInstantaneous(); count++; }

    public double getVoltage() {return sum/count;}

    public double getVoltageInstantaneous() {return sensor.getVoltage();}
}