package SubSystems4997.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqServos.MasqServo;
import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqController;
import SubSystems4997.MasqSubSystem;

/**
 * Created by Archish on 2/12/18.
 */

public class Flipper implements MasqSubSystem {
    // Stick control may seem weird because the y axis is flipped
    public MasqServo flipperLeft, gripBottom;
    public MasqServo flipperRight, gripTop;
    public enum Position {
        OUT (new double[]{.77, 0.08}),
        IN (new double[]{.21, .63}),
        MID (new double[]{.37, .47}),
        RIGHT(new double[]{.61, .24});
        public final double[] pos;
        Position (double[] pos) {this.pos = pos;}
    }

    public enum Grip {
        CLAMP(new double[]{.9, 0.2}),
        OUT (new double[]{0.72, .34});
        public final  double[] grip;
        Grip (double[] grip){this.grip = grip;}
    }

    public Flipper (HardwareMap hardwareMap) {
        flipperLeft = new MasqServo("flipLeft", hardwareMap);
        flipperRight = new MasqServo("flipRight", hardwareMap);
        gripBottom = new MasqServo("gripBottom", hardwareMap);
        gripTop = new MasqServo("gripTop", hardwareMap);
    }

    public void setFlipperPosition(Position position) {
        flipperRight.setPosition(position.pos[0]);
        flipperLeft.setPosition(position.pos[1]);
    }

    public void setGripperPosition(Grip grip) {
        gripTop.setPosition(grip.grip[0]);
        gripBottom.setPosition(grip.grip[1]);
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightStickY() < -.5) {
            setFlipperPosition(Position.OUT);
        }
        else if (controller.rightStickY() > .5) {
            setFlipperPosition(Position.IN);
        }
        else if (controller.rightStickX() > .5) setFlipperPosition(Position.RIGHT);
        else if (controller.rightStickX() < -.5) setFlipperPosition(Position.MID);

        if (controller.a()) setGripperPosition(Grip.OUT);
        else if (controller.x()) setGripperPosition(Grip.CLAMP);
    }
    public void flip (double time) {
        setFlipperPosition(Flipper.Position.MID);
        setFlipperPosition(Flipper.Position.OUT);
        MasqUtils.sleep(time * 1000);
    }
    public void flip () {
        flip(2);
    }


    @Override
    public String getName() {
        return "Flipper";
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[]{flipperRight, flipperLeft};
    }
}
