package MasqLibrary.MasqMotion;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import MasqLibrary.MasqSensors.MasqTouchSensor;

import static MasqLibrary.MasqResources.MasqUtils.getHardwareMap;
import static MasqLibrary.MasqResources.MasqUtils.sleep;
import static MasqLibrary.MasqResources.MasqUtils.tolerance;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.MotorControlAlgorithm.PIDF;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.PI;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/11/2021
 */

public class MasqMotor {
    private final DcMotorEx motor;
    private double wheelDiameter = 4, gearRatio = 1;
    private int zeroPos;
    private boolean hasMin, hasMax;
    private MasqTouchSensor minLim, maxLim = null;
    private final String name;
    private double min = -1, max = 1;

    public MasqMotor(String name, Direction direction, double pidAdjust) {
        motor = getHardwareMap().get(DcMotorEx.class, name);
        motor.setDirection(direction);

        resetEncoder();
        this.name = name;

        double f = 32767 * pidAdjust / motor.getMotorType().getAchieveableMaxTicksPerSecond();
        motor.setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(f / 10, f / 100, 0, f, PIDF));
        motor.setPIDFCoefficients(RUN_TO_POSITION, new PIDFCoefficients(0.5,0,0,0));
    }
    public MasqMotor(String name, Direction direction) {this(name, direction, 1);}
    public MasqMotor(String name, double pidAdjust) {this(name, FORWARD, pidAdjust);}
    public MasqMotor(String name) {this(name, FORWARD, 1);}

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
    public void scalePower(double min, double max) {
        this.min = min;
        this.max = max;
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
        power = clip(power, min, max);
        motor.setPower(power);
    }
    public double getPower() {return motor.getPower();}
    public double getVelocity() {
        if(motor.getMode().isPIDMode()) return motor.getVelocity() / motor.getMotorType().getAchieveableMaxTicksPerSecond();
        else return getPower();
    }

    public void runToPosition(int position, double tolerance) {
        motor.setMode(RUN_TO_POSITION);
        motor.setTargetPosition(position);
        while(!tolerance(getCurrentPosition(), position, tolerance)) sleep(50);
        setVelocityControl(true);
    }

    public void setWheelDiameter(double diameter) {wheelDiameter = diameter;}
    public void setGearRatio(double gearRatio) {this.gearRatio = gearRatio;}

    public DcMotorController getController () {return motor.getController();}
    public int getPortNumber () {return motor.getPortNumber();}

    public PIDFCoefficients getPIDF() {return motor.getPIDFCoefficients(RUN_USING_ENCODER);}
    public void setPIDF(double p, double i, double d, double f) {motor.setVelocityPIDFCoefficients(p,i,d,f);}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nPower: %.2f\nCurrent Inches: %f\nVelocity Control: %s",
                name, getPower(), getInches(), motor.getMode() == RUN_WITHOUT_ENCODER ? "No" : "Yes");
    }
}