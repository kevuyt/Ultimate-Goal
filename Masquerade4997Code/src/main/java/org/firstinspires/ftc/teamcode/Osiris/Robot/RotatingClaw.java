package org.firstinspires.ftc.teamcode.Osiris.Robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import MasqLibrary.MasqMotion.MasqServo;

import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 11/26/2020
 */

public class RotatingClaw {
    private MasqServo claw, rotator;

    public RotatingClaw() {
        claw = new MasqServo("claw");
        rotator = new MasqServo("rotator");
        reset();
    }

    public void driverControl(Gamepad controller) {
        claw.toggle(controller.a);
        rotator.toggle(controller.b);
    }

    public void close() {claw.setPosition(0);}
    public void open() {claw.setPosition(1);}

    public void raise() {rotator.setPosition(0);}
    public void mid() {rotator.setPosition(0.7);}
    public void lower() {rotator.setPosition(1);}

    public void reset() {
        claw.scaleRange(0.1, 0.4);
        rotator.scaleRange(0.39, 0.96);
        close();
    }

    public MasqServo getClaw() {return claw;}
    public MasqServo getRotator() {return rotator;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US,"Rotating Claw:\nClaw Position: %.2f\nRotater Position: %.2f", claw.getPosition(), rotator.getPosition());
    }
}