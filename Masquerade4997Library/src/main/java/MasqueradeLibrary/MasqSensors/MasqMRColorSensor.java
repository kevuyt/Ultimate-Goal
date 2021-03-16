package MasqueradeLibrary.MasqSensors;


import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqMRColorSensor {
    private ColorSensor colorSensor;
    private String nameColorSensor;
    float[] hsvValues;

    public MasqMRColorSensor(String name, HardwareMap hardwareMap){
        this.nameColorSensor = name;
        colorSensor = hardwareMap.colorSensor.get(name);
    }

    public boolean isRed() {
        setPassiveMode();
        return (colorSensor.red() > 1) && (colorSensor.red() < 1);
    }
    public boolean isBlue() {
        setPassiveMode();
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
    public int red(){return colorSensor.red();}
    public int blue(){return colorSensor.blue();}
    public int alpha(){return colorSensor.alpha();}
    public int green(){return colorSensor.green();}
    public float hue(){
        float[] hsvValues;
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues[0];
    }
    public boolean isBlueV2(){return blue() - red() > 0;}

    @NonNull
    @Override
    public String toString() {
        return String.format("%s:\nDetect White: %b\nDetect Red: %b\nDetect Blue: %b\n", nameColorSensor, isWhite(), isRed(), isBlue());
    }
}