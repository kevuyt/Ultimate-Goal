package SubSystems4997.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqServos.MasqServo;

/**
 * Created by Archish on 4/5/18.
 */

public class Gripper {
    public MasqServo gripBottom, gripTop;
    public enum Grip {
        CLAMP(new double[]{.9, 0.2}),
        OUT (new double[]{0.72, .34});
        public final  double[] grip;
        Grip (double[] grip){this.grip = grip;}
    }
    public Gripper (HardwareMap hardwareMap) {
        gripBottom = new MasqServo("gripBottom", hardwareMap);
        gripTop = new MasqServo("gripTop", hardwareMap);
    }
    public void setGripperPosition(Grip grip) {
        gripTop.setPosition(grip.grip[0]);
        gripBottom.setPosition(grip.grip[1]);
    }
}
