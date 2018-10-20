package org.firstinspires.ftc.teamcode.Robots.FalconSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqWrappers.MasqController;
import SubSystems4997.MasqSubSystem;

/**
 * Created by Archishmaan Peyyety on 10/17/18.
 * Project: MasqLib
 */

public class MasqRotator implements MasqSubSystem {
    public MasqMotorSystem rotator;
    private double currentPosition, targetPosition, basePower = 0.3;
    private double liftPosition = 0;
    private MasqPIDController output = new MasqPIDController(0.01, 0.0, 0.00);
    public MasqRotator (HardwareMap hardwareMap) {
        rotator = new MasqMotorSystem("rotatorOne", DcMotor.Direction.FORWARD,"rotatorTwo",
                DcMotor.Direction.REVERSE, "rotator", hardwareMap, MasqMotorModel.NEVEREST40);
    }
    @Override
    public void DriverControl(MasqController controller) {
        basePower = (0.00002 * liftPosition) + 0.274;
        if (controller.rightTriggerPressed()) rotator.setPower(-basePower);
        else if (controller.leftTriggerPressed()) rotator.setPower(0.2);

        if (controller.rightTrigger() > 0 || controller.leftTrigger() > 0) targetPosition = rotator.getCurrentPosition();
        else {
            currentPosition = rotator.getCurrentPosition();
            rotator.setPower(output.getOutput(currentPosition, targetPosition));
        }
    }
    public void setLiftPosition (double liftPosition) {
        this.liftPosition = liftPosition;
    }
    public double getBasePower() {
        return basePower;
    }
    public double getKP () {
        return output.getKp();
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
