package Library4997.MasqSensors;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorModel;

/**
 * Created by Archish on 3/14/18.
 */

public class MasqEncoder {
  private MasqMotorModel model;
  private MasqMotor motor;
  private double wheelDiameter = 4, gearRatio = 1;
  private double currentPosition, zeroPos;

  public MasqEncoder(MasqMotor motor, MasqMotorModel model) {
    this.model = model;
    this.motor = motor;
  }

  public double getRelativePosition() {
    currentPosition = (int) (getAbsolutePosition() - zeroPos);
    return currentPosition;
  }

  public double getInches () {
    return getRelativePosition() / getClicksPerInch();
  }

  public double getAbsolutePosition() {
    return motor.getAbsolutePosition();
  }

  public void resetEncoder() {
    zeroPos = (int) getAbsolutePosition();
    currentPosition = 0;
  }

  public double getWheelDiameter() {
    return wheelDiameter;
  }

  public double getClicksPerInch() {
    return (model.CPR() / (wheelDiameter * Math.PI)) * gearRatio;
  }

  public void setWheelDiameter(double diameter) {
    wheelDiameter = diameter;
  }

  public double getRPM () {
    return model.RPM();
  }
  public double getClicksPerRotation () {
    return model.CPR();
  }
  public double getGearRatio() {
    return gearRatio;
  }

  public void setGearRatio(double ratio) {
    gearRatio = ratio;
  }

  public void setModel(MasqMotorModel model) {
    this.model = model;
  }

  public MasqMotorModel getModel() {
    return model;
  }
}