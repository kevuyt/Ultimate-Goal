package org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Archishmaan Peyyety on 11/7/18.
 * Project: MasqLib
 */

public class MasqElevator implements MasqSubSystem {
    private MasqMotor lift;
    private boolean magSwitch;
    private double scorePosition;

    public MasqElevator (HardwareMap hardwareMap) {
        lift = new MasqMotor("lift", MasqMotorModel.ORBITAL20, DcMotor.Direction.FORWARD, hardwareMap);
        lift.resetEncoder();
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightBumper()) lift.setPower(-1);
        else if (controller.rightTriggerPressed()) lift.setPower(1);
        else lift.setPower(-0.2);

        if (magSwitch) lift.resetEncoder();
    }

    public boolean isMagSwitch() {
        return magSwitch;
    }

    public void setMagSwitch(boolean magSwitch) {
        this.magSwitch = magSwitch;
    }

    public void runToPosition (Direction direction, double position) {
        double ticksRemaining = Math.abs(position - Math.abs(lift.getCurrentPosition()));
        while (ticksRemaining > 200 && MasqRobot.opModeIsActive()) {
            ticksRemaining = Math.abs(position - Math.abs(lift.getCurrentPosition()));
            lift.setPower(direction.value);
            DashBoard.getDash().create(ticksRemaining);
            DashBoard.getDash().update();
        }
        lift.setPower(0);
    }

    public double getCurrentPosition () {
        return lift.getCurrentPosition();
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
