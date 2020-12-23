package org.firstinspires.ftc.teamcode.PlaceHolder.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Keval Kataria on 11/26/2020
 */
public class RotatingClaw implements MasqSubSystem {
    MasqServo claw, rotater;

    public RotatingClaw(HardwareMap hardwareMap) {
        claw = new MasqServo("claw",hardwareMap);
        rotater = new MasqServo("rotater",hardwareMap);

    }
    @Override
    public void DriverControl(MasqController controller) throws InterruptedException {
        if (controller.a()) claw.setPosition(1);
        else if (controller.b()) claw.setPosition(0);
        if (controller.x()) rotater.setPosition(1);
        else if (controller.y()) rotater.setPosition(0);
    }

    public void close() {claw.setPosition(0);}
    public void open() {claw.setPosition(0.3);}

    public void raise() {rotater.setPosition(0.1);}
    public void lower() {rotater.setPosition(.85);}

    public void reset() {
        close();
        raise();
    }

    @Override
    public String getName() {
        return "Claw";
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[0];
    }
}
