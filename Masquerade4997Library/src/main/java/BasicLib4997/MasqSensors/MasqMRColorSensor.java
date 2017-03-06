package BasicLib4997.MasqSensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

import java.util.Locale;

import BasicLib4997.MasqHardware;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 10/28/16.
 */

public class MasqMRColorSensor implements Sensor_Thresholds, MasqHardware {
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
    private String nameColorSensor;

    public MasqMRColorSensor(String name){
        this.nameColorSensor = name;
        colorSensor = FtcOpModeRegister.opModeManager.getHardwareMap().colorSensor.get(name);
    }
    public boolean isNotRed() {
        setPassiveMode();
        boolean isRed;
        float[] hsvValues;
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        isRed = (colorSensor.red() > RED_MIN) && (colorSensor.red() < RED_MAX);
        return isRed;
    }
    public boolean isNotBlue() {
        setPassiveMode();
        boolean isRed;
        float[] hsvValues;
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        isRed = (colorSensor.red() > BLUE_MIN) && (colorSensor.red() < BLUE_MAX);
        return isRed;
    }
    public boolean isNotWhite() {
        setActiveMode();
        boolean isRed;
        float[] hsvValues;
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        isRed = (hsvValues[0] > WHITE_MIN) && (hsvValues[0] < WHITE_MAX);
        return isRed;
    }
    public void setActiveMode () {
        colorSensor.enableLed(true);
    }
    public void setPassiveMode () {
        colorSensor.enableLed(true);
    }
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
    public float Other1(){
        float[] hsvValues;
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues[1];
    }
    public float Other2(){
        float[] hsvValues;
        hsvValues = new float[]{0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues[2];
    }
    public void setI2cAddress(I2cAddr adress) {
        colorSensor.setI2cAddress(adress);
    }
    public I2cAddr getI2CAdress() {
        return colorSensor.getI2cAddress();
    }
    public String getName() {
        return nameColorSensor;
    }

    public String getDash() {
        return String.format(Locale.US, "Color #: %d   ARGB:[%d,%d,%d,%d]  HSV:[%.3f,%.3f,%.3f]",
                alpha(), red(), green(), blue(), hue());
    }
}
