package org.firstinspires.ftc.teamcode.Osiris.Robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import MasqLibrary.MasqMotion.MasqServo;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 11/26/2020
 */

public class RotatingClaw {
    private final MasqServo claw, rotator;

    public RotatingClaw() {
        claw = new MasqServo("claw", REVERSE);
        rotator = new MasqServo("rotator");
        reset();
    }

    public void driverControl(Gamepad controller) {
        claw.toggle(controller.a);
        rotator.toggle(controller.b, 1, 0.3);
    }

    public void close() {claw.setPosition(1);}
    public void open() {claw.setPosition(0);}

    public void raise() {rotator.setPosition(0.3);}
    public void mid() {rotator.setPosition(0.7);}
    public void lower() {rotator.setPosition(1);}
    public void init() {rotator.setPosition(0);}

    public void reset() {
        claw.scaleRange(0.09, 0.4);
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