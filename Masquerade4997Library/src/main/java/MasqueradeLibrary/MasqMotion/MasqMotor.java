package MasqueradeLibrary.MasqMotion;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import MasqueradeLibrary.MasqSensors.MasqTouchSensor;

import static MasqueradeLibrary.MasqResources.MasqUtils.getHardwareMap;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.PI;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/11/2021
 */

public class MasqMotor {
    private DcMotor motor;
    private double wheelDiameter = 4, gearRatio = 1;
    private int zeroPos;
    private boolean hasMin, hasMax;
    private MasqTouchSensor minLim, maxLim = null;
    private String name;

    public MasqMotor(String name) {
        motor = getHardwareMap().get(DcMotor.class, name);
        resetEncoder();
        this.name = name;
    }
    public MasqMotor(String name, Direction direction) {
        motor = getHardwareMap().dcMotor.get(name);
        motor.setDirection(direction);
        resetEncoder();
        this.name = name;
    }

    public int getCurrentPosition() {return getAbsolutePosition() - zeroPos;}
    public double getInches () {return getCurrentPosition() / getClicksPerInch();}
    public int getAbsolutePosition () {return motor.getCurrentPosition();}
    public double getClicksPerInch() {return (motor.getMotorType().getTicksPerRev() / (wheelDiameter * PI)) * gearRatio;}
    public void resetEncoder() {zeroPos = getAbsolutePosition();}

    public void setLimits(MasqTouchSensor min, MasqTouchSensor max){
        maxLim = max;
        minLim = min;
        hasMax = hasMin = true;
    }
    public void setMinLimit(MasqTouchSensor min) {
        minLim = min;
        hasMin = true;
    }
    public void setMaxLimit(MasqTouchSensor max) {
        maxLim = max;
        hasMax = true;
    }

    public void setVelocityControl(boolean velocityControl) {
        if(velocityControl) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
        }
        else {
            motor.setMode(RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(FLOAT);
        }
    }

    public void setPower(double power) {
        if(power > 0 && hasMax && maxLim.isPressed() || power < 0 && hasMin && minLim.isPressed()) power = 0;
        power = clip(power, -1, 1);
        motor.setPower(power);
    }
    public double getPower() {return motor.getPower();}

    public void setWheelDiameter(double diameter) {wheelDiameter = diameter;}
    public void setGearRatio(double gearRatio) {this.gearRatio = gearRatio;}

    public DcMotorController getController () {return motor.getController();}
    public int getPortNumber () {return motor.getPortNumber();}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nPower: %.2f\nCurrent Inches: %f\nVelocity Control: %s",
                name, getPower(), getInches(), motor.getMode() == RUN_WITHOUT_ENCODER ? "No" : "Yes");
    }
}