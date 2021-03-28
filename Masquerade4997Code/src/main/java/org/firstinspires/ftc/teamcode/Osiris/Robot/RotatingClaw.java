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
        if(controller.a) open();
        else close();
        rotator.toggle(controller.b, 0, 0.683);
    }

    public void close() {claw.setPosition(0);}
    public void open() {claw.setPosition(1);}

    public void raise() {rotator.setPosition(0.683);}
    public void mid() {rotator.setPosition(0.2);}
    public void lower() {rotator.setPosition(0);}

    public void reset() {
        claw.scaleRange(0.09, 0.4);
        rotator.scaleRange(0.106, 1);
        close();
    }

    public MasqServo getClaw() {return claw;}
    public MasqServo getRotator() {return rotator;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US,"Rotating Claw:\nClaw Position: %.2f\nRotator Position: %.2f", claw.getPosition(), rotator.getPosition());
    }
}