package org.firstinspires.ftc.teamcode.Osiris.Robot;

import com.qualcomm.robotcore.hardware.*;

import Library4997.MasqResources.*;
import Library4997.MasqServos.MasqServo;

/**
 * Created by Keval Kataria on 11/26/2020
 */
public class RotatingClaw implements MasqSubSystem {
    private MasqServo claw, rotater;

    public RotatingClaw(HardwareMap hardwareMap) {
        claw = new MasqServo("claw",hardwareMap);
        rotater = new MasqServo("rotater",hardwareMap);
        reset();
    }

    @Override
    public void driverControl(Gamepad controller) {
        claw.toggle(controller.a);
        rotater.toggle(controller.b, 0.4, 1);
    }

    public void close() {claw.setPosition(0);}
    public void open() {claw.setPosition(1);}

    public void init() {rotater.setPosition(0.00469);}
    public void raise() {rotater.setPosition(0.4);}
    public void mid() {rotater.setPosition(0.9);}
    public void lower() {rotater.setPosition(1);}

    public void reset() {
        claw.scaleRange(0.05, 0.4);
        rotater.scaleRange(0.01, 0.96);
        close();
        //init();
    }

    public MasqServo getClaw() {return claw;}
    public MasqServo getRotater() {return rotater;}

    @Override
    public String getName() {return "Rotating Claw";}

    @Override
    public MasqHardware[] getComponents() {return new MasqHardware[] {claw, rotater};}
}