package SubSystems4997.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqSensors.MasqREVColorSensor;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqController;
import SubSystems4997.MasqSubSystem;

/**
 * Created by Archish on 2/12/18.
 */

public class Flipper implements MasqSubSystem {
    public MasqServo flipperLeft, gripBottom;
    public MasqServo flipperRight, gripTop;
    private MasqREVColorSensor doubleBlock;
    private boolean overide = false;
    public enum Position {
        OUT (new double[]{.79, 0.16}),
        IN (new double[]{.24, .73}),
        MID (new double[]{.34, .62}),
        RIGHT(new double[]{.61, .36});
        public final double[] pos;
        Position (double[] pos) {this.pos = pos;}
    }

    public enum Grip {
        CLAMP(new double[]{.9, 0.2}),
        OUT (new double[]{0.72, .3});
        public final  double[] grip;
        Grip (double[] grip){this.grip = grip;}
    }

    public Flipper (HardwareMap hardwareMap) {
        flipperLeft = new MasqServo("flipLeft", hardwareMap);
        flipperRight = new MasqServo("flipRight", hardwareMap);
        gripBottom = new MasqServo("gripBottom", hardwareMap);
        gripTop = new MasqServo("gripTop", hardwareMap);
    }

    public void setPosition (Position position) {
        flipperRight.setPosition(position.pos[0]);
        flipperLeft.setPosition(position.pos[1]);
    }

    public void setPosition(Grip grip) {
        gripTop.setPosition(grip.grip[0]);
        gripBottom.setPosition(grip.grip[1]);
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightStickY() < -.5) {
            overide = true;
            setPosition(Position.OUT);
        }
        else if (controller.rightStickY() > .5) {
            overide = false;
            setPosition(Position.IN);
        }
        else if (controller.rightStickX() > .5) setPosition(Position.RIGHT);
        else if (controller.rightStickX() < -.5) setPosition(Position.MID);

        if (controller.a()) setPosition(Grip.OUT);
        else if (controller.x()) setPosition(Grip.CLAMP);
    }
    public void flip (double time) {
        setPosition(Flipper.Position.MID);
        setPosition(Flipper.Position.OUT);
        MasqUtils.sleep(time * 1000);
    }
    public void flip () {
        flip(2);
    }

    public void setBlockDetector (MasqREVColorSensor doubleBlock) {
        this.doubleBlock = doubleBlock;
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
