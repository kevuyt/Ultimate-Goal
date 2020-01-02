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
  private double clicksPerInch;
  public MasqEncoder(MasqMotor motor, MasqMotorModel model) {
    clicksPerInch = (model.CPR() / (wheelDiameter * Math.PI));
    this.model = model;
    this.motor = motor;
  }

  public double getRelativePosition() {
    currentPosition = (int) (getAbsolutePosition() - zeroPos);
    return currentPosition;
  }

  public double getInches () {
    return (getRelativePosition()/getClicksPerInch()) * gearRatio;
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
    return clicksPerInch * gearRatio;
  }

  public void setWheelDiameter(double wheelDiameter) {
    this.wheelDiameter = wheelDiameter;
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

  public void setGearRatio(double gearRatio) {
    this.gearRatio = gearRatio;
  }

  public void setModel(MasqMotorModel model) {
    this.model = model;
  }

  public MasqMotorModel getModel() {
    return model;
  }
}