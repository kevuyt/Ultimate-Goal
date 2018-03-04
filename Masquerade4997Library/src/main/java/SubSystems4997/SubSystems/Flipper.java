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
    public MasqServo flipperLeft;
    public MasqServo flipperRight;

    public enum Position {
        OUT (new double[]{1, 0}),
        IN (new double[]{.47, .51}),
        MID (new double[]{.61, .43}),
        RIGHT(new double[]{.82, .29});
        public final double[] pos;
        Position (double[] pos) {this.pos = pos;}
    }

    public Flipper (HardwareMap hardwareMap) {
        flipperLeft = new MasqServo("flipLeft", hardwareMap);
        flipperRight = new MasqServo("flipRight", hardwareMap);
    }

    public void setPosition (Position position) {
        flipperRight.setPosition(position.pos[0]);
        flipperLeft.setPosition(position.pos[1]);
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.rightStickY() < -.5) {
            setPosition(Position.OUT);
        }
        else if (controller.rightStickY() > .5) {
            setPosition(Position.IN);
        }
        else if (controller.rightStickX() > .5) {
            setPosition(Position.RIGHT);
        }
        else if (controller.rightStickX() < -.5) {
            setPosition(Position.MID);
        }
    }
    public void flip (double time) {
        setPosition(Flipper.Position.MID);
        setPosition(Flipper.Position.OUT);
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
