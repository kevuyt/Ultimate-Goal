package MasqLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;

import static MasqLibrary.MasqResources.MasqUtils.getHardwareMap;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqColorSensor {
    private ColorSensor colorSensor;
    private String name;
    private boolean active = true;

    public MasqColorSensor(String name){
        this.name = name;
        colorSensor = getHardwareMap().colorSensor.get(name);
        setActiveMode();
    }
    public void setActiveMode () {
        colorSensor.enableLed(true);
        active = true;
    }
    public void setPassiveMode () {
        colorSensor.enableLed(false);
        active = false;
    }
    public int red(){return colorSensor.red();}
    public int blue(){return colorSensor.blue();}
    public int alpha(){return colorSensor.alpha();}
    public int green(){return colorSensor.green();}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nActive Mode: %b\nRed: %d\nBlue: %d\nGreen: %d\nAlpha: %d\n",
                name, active, red(), blue(), green(), alpha());
    }
}