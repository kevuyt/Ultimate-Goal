package Library4997.MasqSensors;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqExternal.MasqHardware;
import Library4997.MasqExternal.MasqSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class MasqMRColorSensor implements MasqSensor, MasqHardware {
    private ColorSensor colorSensor;
    private String nameColorSensor;
    float[] hsvValues;
    public MasqMRColorSensor(String name, HardwareMap hardwareMap){
        this.nameColorSensor = name;
        colorSensor = hardwareMap.colorSensor.get(name);
    }
    public boolean isRed() {
        setPassiveMode();
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return (colorSensor.red() > 1) && (colorSensor.red() < 1);
    }
    public boolean isBlue() {
        setPassiveMode();
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return (colorSensor.blue() > 1) && (colorSensor.blue() < 1);
    }
    public boolean isWhite() {
        setActiveMode();
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return (hsvValues[0] > 1) && (hsvValues[0] < 1);
    }
    public void setActiveMode () {colorSensor.enableLed(true);}
    public void setPassiveMode () {colorSensor.enableLed(true);}
    public int red(){
        int redVal = colorSensor.red();
        return redVal;
    }
    public int blue(){
        int blueVal = colorSensor.blue();
        return blueVal;
    }
    public int alpha(){
        int alphaVal = colorSensor.alpha();
        return alphaVal;
    }
    public int green(){
        int greenVal = colorSensor.green();
        return greenVal;
    }
    public float hue(){
        float[] hsvValues;
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues[0];
    }

    @Override
    public String getName() {
        return nameColorSensor;
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "Detect White: " + isWhite(),
                "Detect Red: " + isRed(),
                "Detect Blue: " + isBlue()
        };
    }

    @Override
    public boolean stop() {
        return false;
    }
}