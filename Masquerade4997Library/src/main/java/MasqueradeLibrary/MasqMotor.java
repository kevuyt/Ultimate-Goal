package MasqueradeLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.PI;

/**
 * Created by Keval Kataria on 3/11/2021
 */
public class MasqMotor {
    public DcMotor motor;
    private MasqMotorModel model;
    private double wheelDiameter = 4, gearRatio = 1, zeroPos;

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
        this.model = model;
    }
    public MasqMotor(String name, MasqMotorModel model, DcMotor.Direction direction, HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
        this.model = model;
    }

    public double getCurrentPosition() {return getAbsolutePosition() - zeroPos;}
    public double getInches () {return getCurrentPosition() / getClicksPerInch();}
    public double getAbsolutePosition () {return motor.getCurrentPosition();}
    public double getClicksPerInch() {return (model.CPR / (wheelDiameter * PI)) * gearRatio;}
    public void resetEncoder() {zeroPos = (int) getAbsolutePosition();}

    public void setWheelDiameter(double diameter) {wheelDiameter = diameter;}
    public void setGearRatio(double gearRatio) {this.gearRatio = gearRatio;}


}
