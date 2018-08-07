package SubSystems4997.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqServos.MasqServo;
import Library4997.MasqUtilities.MasqHelpers.MasqHardware;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqController;
import SubSystems4997.MasqSubSystem;
import SubSystems4997.SubSystems.Gripper.Grip;

/**
 * Created by Archish on 2/12/18.
 */

public class Flipper implements MasqSubSystem {
    // Stick control may seem weird because the y axis is flipped
    public MasqServo flipperLeft, flipperRight;
    private Gripper gripper;
    public enum Position {
        OUT (new double[]{.77, 0.08}),
        IN (new double[]{.21, .63}),
        MID (new double[]{.37, .47}),
        RIGHT(new double[]{.61, .24});
        public final double[] pos;
        Position (double[] pos) {this.pos = pos;}
    }
    public Flipper (HardwareMap hardwareMap) {
        gripper = new Gripper(hardwareMap);
        flipperLeft = new MasqServo("flipLeft", hardwareMap);
        flipperRight = new MasqServo("flipRight", hardwareMap);
    }

    public void setFlipperPosition(Position position) {
        flipperRight.setPosition(position.pos[0]);
        flipperLeft.setPosition(position.pos[1]);
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

        if (controller.a()) gripper.setGripperPosition(Grip.OUT);
        else if (controller.x()) gripper.setGripperPosition(Grip.CLAMP);
    }
    public void flip (double time) {
        setFlipperPosition(Flipper.Position.MID);
        setFlipperPosition(Flipper.Position.OUT);
        MasqUtils.sleep(time * 1000);
    }
    public void flip () {
        flip(2);
    }

    public Gripper getGripper () {
        return gripper;
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
