package org.firstinspires.ftc.teamcode.Osiris.Robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.*;

import MasqueradeLibrary.MasqMotion.MasqServo;

import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 11/26/2020
 */
public class RotatingClaw {
    private MasqServo claw, rotater;

    public RotatingClaw() {
        claw = new MasqServo("claw");
        rotater = new MasqServo("rotater");
        reset();
    }

    public void driverControl(Gamepad controller) {
        claw.toggle(controller.a);
        rotater.toggle(controller.b);
    }

    public void close() {claw.setPosition(0);}
    public void open() {claw.setPosition(1);}

    public void raise() {rotater.setPosition(0);}
    public void mid() {rotater.setPosition(0.7);}
    public void lower() {rotater.setPosition(1);}

    public void reset() {
        claw.scaleRange(0.1, 0.4);
        rotater.scaleRange(0.39, 0.96);
        close();
    }

    public MasqServo getClaw() {return claw;}
    public MasqServo getRotater() {return rotater;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US,"Rotating Claw:\nClaw Position: %.2f\nRotater Position: %.2f", claw.getPosition(), rotater.getPosition());
    }
}