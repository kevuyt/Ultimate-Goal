package org.firstinspires.ftc.teamcode.Osiris.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqResources.MasqHelpers.MasqSubSystem;
import Library4997.MasqWrappers.MasqController;

/**
 * Created by Keval Kataria on 11/26/2020
 */
public class RotatingClaw implements MasqSubSystem {
    public MasqServo claw, rotater;

    public RotatingClaw(HardwareMap hardwareMap) {
        claw = new MasqServo("claw",hardwareMap);
        rotater = new MasqServo("rotater",hardwareMap);
    }
    @Override
    public void DriverControl(MasqController controller) throws InterruptedException {
        if (controller.a()) open();
        else if (controller.b()) close();
        if (controller.x()) lower();
        else if (controller.y()) raise();
    }

    public void close() {claw.setPosition(0.13);}
    public void open() {claw.setPosition(0.34);}

    public void raise() {rotater.setPosition(0.15);}
    public void lower() {rotater.setPosition(0.94);}

    public void reset() {
        close();
        raise();
    }

    @Override
    public String getName() {
        return "Rotating Claw";
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[] {claw, rotater};
    }
}
