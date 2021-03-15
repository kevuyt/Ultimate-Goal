package MasqueradeLibrary;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Math.PI;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/11/2021
 */
public class MasqMotor {
    private DcMotor motor;
    private MasqMotorModel model;
    private double wheelDiameter = 4, gearRatio = 1, zeroPos;
    private boolean minLimit, maxLimit;
    private MasqTouchSensor minLim, maxLim = null;
    private String name;

    public enum MasqMotorModel {
        ORBITAL20 (560, 315), NEVEREST40(1120, 160), NEVEREST60(1680,105),
        USDIGITAL_E4T(1440, 0), REVHDHEX40(1120, 150), REVHDHEX20(560, 300),
        REVTHROUGHBORE(8192, 0), NEVERREST37(44.4, 1780), REVHDHEX1(38, 6000);

        public final double CPR;
        public final int RPM;

        MasqMotorModel(double CPR, int RPM) {
            this.CPR = CPR;
            this.RPM = RPM;
        }
    }

    public MasqMotor(String name, MasqMotorModel model, HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(BRAKE);
        this.model = model;
        resetEncoder();
        this.name = name;
    }
    public MasqMotor(String name, MasqMotorModel model, DcMotor.Direction direction, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(BRAKE);
        this.model = model;
        resetEncoder();
        this.name = name;
    }

    public double getCurrentPosition() {return getAbsolutePosition() - zeroPos;}
    public double getInches () {return getCurrentPosition() / getClicksPerInch();}
    public double getAbsolutePosition () {return motor.getCurrentPosition();}
    public double getClicksPerInch() {return (model.CPR / (wheelDiameter * PI)) * gearRatio;}
    public void resetEncoder() {zeroPos = (int) getAbsolutePosition();}

    public void setLimits(MasqTouchSensor min, MasqTouchSensor max){
        maxLim = max; minLim = min;
        maxLimit = true;
        minLimit = true;
    }
    public void setMinLimit(MasqTouchSensor min) {
        minLim = min;
        minLimit = true;
    }
    public void setMaxLimit(MasqTouchSensor max) {
        maxLim = max;
        maxLimit = true;
    }

    public void setVelocityControl(boolean velocityControl) {
        if(velocityControl) motor.setMode(RUN_USING_ENCODER);
        else motor.setMode(RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        if(power > 0 && maxLimit && maxLim.isPressed() || power < 0 && minLimit && minLim.isPressed()) power = 0;
        power = Range.clip(power, -1, 1);
        motor.setPower(power);
    }
    public double getPower() {return motor.getPower();}

    public void setWheelDiameter(double diameter) {wheelDiameter = diameter;}
    public void setGearRatio(double gearRatio) {this.gearRatio = gearRatio;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nPower: %.2f\nCurrent Inches: %f\nVelocity Control: %s", name, getPower(), getInches(), motor.getMode() == RUN_WITHOUT_ENCODER ? "No" : "Yes");
    }
}