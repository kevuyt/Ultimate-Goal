package Library4997.MasqSensors;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorModel;

import static java.lang.Math.PI;

/**
 * Created by Archish on 3/14/18.
 */

public class MasqEncoder {
  private MasqMotorModel model;
  private MasqMotor motor;
  private double wheelDiameter = 4, gearRatio = 1, zeroPos;

  public MasqEncoder(MasqMotor motor, MasqMotorModel model) {
    this.model = model;
    this.motor = motor;
  }

  public double getRelativePosition() {return getAbsolutePosition() - zeroPos;}
  public double getInches () {return getRelativePosition() / getClicksPerInch();}
  public double getAbsolutePosition() {return motor.getAbsolutePosition();}
  public double getClicksPerInch() {return (model.CPR() / (wheelDiameter * PI)) * gearRatio;}
  public double getRPM () {return model.RPM();}
  public double getClicksPerRotation () {return model.CPR();}

  public void resetEncoder() {
    zeroPos = (int) getAbsolutePosition();
  }


  public void setWheelDiameter(double diameter) {wheelDiameter = diameter;}
  public void setGearRatio(double gearRatio) {this.gearRatio = gearRatio;}
  public void setModel(MasqMotorModel model) {this.model = model;}



}