package MasqueradeSubSystems.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqServos.MasqServo;
import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqWrappers.MasqController;
import MasqueradeSubSystems.MasqSubSystem;

/**
 * Created by Archish on 2/12/18.
 */

public class IntakeDeployer implements MasqSubSystem {
    
    private MasqServo blueDeployer, redDeployer;
    private boolean state;
    public IntakeDeployer (HardwareMap hardwareMap) {
        blueDeployer = new MasqServo("blueDeployer", hardwareMap);
        redDeployer = new MasqServo("redDeployer", hardwareMap);
    }
    
    public enum Position {
        OUT(new double[]{1, 0}),
        IN(new double[]{0, 1});
        public final double[] pos;
        Position(double[] pos) {this.pos = pos;}
    }

    public void setPosition(Position position) {
        redDeployer.setPosition(position.pos[0]);
        blueDeployer.setPosition(position.pos[1]);
    }

    @Override
    public void DriverControl(MasqController controller) {
        if (controller.xOnPress() && state) {
            state = false;
            setPosition(Position.OUT);
            controller.update();
        }
        if (controller.xOnPress() && !state) {
            state = true;
            setPosition(Position.IN);
            controller.update();
        }
    }

    @Override
    public String getName() {
        return "Deployers";
    }

    @Override
    public MasqHardware[] getComponents() {
        return null;
    }
}
