package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Library4997.MasqUtilities.MasqHardware;

/**
 * Created by Archish on 10/29/17.
 */

public class MasqREVColorSensor implements MasqHardware {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private int margin = 0;
    private String name;
    float hsvValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;

    public MasqREVColorSensor(String name, HardwareMap hardwareMap){
        this.name = name;
        colorSensor = hardwareMap.colorSensor.get(name);
        distanceSensor = hardwareMap.get(DistanceSensor.class, name);
    }
    public double getDistance(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }
    public int getBlue () {
        return colorSensor.blue();
    }
    public int getRed () {
        return colorSensor.red();
    }
    public int getGreen () {
        return colorSensor.green();
    }
    public void setActive () {colorSensor.enableLed(true);}
    public void setPassive() {colorSensor.enableLed(false);}
    public boolean isBlue () {
        return getBlue() > getRed() + margin;
    }
    public boolean isRed () {return getRed() > getBlue() + margin;}

    public void setMargin(int margin) {this.margin = margin;}

    @Override
    public String getName() {
        return name;
    }
    @Override
    public String[] getDash() {
        return new String[] {
                name,
                "Detect Blue: ", Boolean.toString(isBlue()),
                "Detect Red: ", Boolean.toString(isRed()),
                "Green" + getGreen(),
                "Blue" + getBlue(),
                "Red" + getRed(),
                "Distance in CM" + getDistance(DistanceUnit.CM)
        };
    }
}
