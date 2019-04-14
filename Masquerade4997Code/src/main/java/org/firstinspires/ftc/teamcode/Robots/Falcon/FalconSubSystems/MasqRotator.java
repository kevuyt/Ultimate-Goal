package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 10/17/18.
 * Project: MasqLib
 */

public class MasqRotator implements MasqSubSystem {
    private MasqLimitSwitch limitSwitch;
    public MasqMotorSystem rotator;
    private double holdPosition = 0;
    private double downRotationPosition = 380;
    private MasqPIDController holdController = new MasqPIDController(0.001, 0, 0.0001);
    private MasqPIDController downController = new MasqPIDController(0.001, 0, -5);
    private MasqPIDController upController = new MasqPIDController(0.001, 0.000001, 0);
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotorSystem("rotator1", DcMotor.Direction.FORWARD, "rotator2", DcMotor.Direction.REVERSE, MasqMotorModel.NEVERREST256, hardwareMap);
        rotator.setClosedLoop(true);
        rotator.setKp(0.01);
        rotator.resetEncoders();
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) {
            setVelocity(-0.8);
            holdPosition = rotator.motor2.getCurrentPosition();
        }
        else if (controller.rightTriggerPressed()) {
            rotator.motor2.setPower(downController.getOutput(rotator.motor2.getCurrentPosition(), downRotationPosition));
            holdPosition = rotator.motor2.getCurrentPosition();
        }
        else rotator.setPower(holdController.getOutput(rotator.motor2.getCurrentPosition(), holdPosition));
        if (limitSwitch.isPressed()) {
            downRotationPosition = 1500;
            rotator.motor2.resetEncoder();
        }
        DashBoard.getDash().create("Position: ", rotator.motor2.getCurrentPosition());
        DashBoard.getDash().create("Power: ", rotator.motor2.getPower());
        DashBoard.getDash().create("D: ", downController.getDeriv());
        DashBoard.getDash().create("D*Kd: ", downController.getDeriv() * downController.getKd());
    }

    public void setHoldPosition(double holdPosition) {
        this.holdPosition = holdPosition;
    }

    public void setLimitSwitch(MasqLimitSwitch limitSwitch) {
        this.limitSwitch = limitSwitch;
    }

    public void setVelocity(double velocity) {
        rotator.motor2.setVelocity(velocity);
        rotator.motor1.setPower(rotator.motor2.getPower());
    }

    public double getAngle() {
        double angle = -(rotator.motor2.getCurrentPosition() / 4400) * 360;
        if (angle < 0) return 0;
        else if (angle > 100) return 100;
        return angle;
    }

    public double getProportionalPower(double angle, double min) {
        double a = (min - 1) / 3025;
        double b = -(2 * (min - 1)) / 55;
        double y = (a * angle * angle) + (b * angle) + min;
        if (y < min) return min;
        return y;
    }

    @Override
    public String getName() {
        return null;
    }


    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }
}
