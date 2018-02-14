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

public class IntakeDeployer implements MasqSubSystem {
    
    public MasqServo blueDeployer, redDeployer;
    private boolean state;
    public IntakeDeployer (HardwareMap hardwareMap) {
        blueDeployer = new MasqServo("blueDeployer", hardwareMap);
        redDeployer = new MasqServo("redDeployer", hardwareMap);
    }
    
    public enum Position {
        OUT(new double[]{.76, .2}),
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
        if (controller.yOnPress() && state) {
            state = false;
            redDeployer.setPosition(Position.OUT.pos[0]);
            MasqUtils.sleep(500);
            blueDeployer.setPosition(Position.OUT.pos[1]);
            controller.update();
        }
        if (controller.yOnPress() && !state) {
            state = true;
            blueDeployer.setPosition(Position.IN.pos[1]);
            MasqUtils.sleep(500);
            redDeployer.setPosition(Position.IN.pos[0]);
            controller.update();
        }
    }

    @Override
    public String getName() {
        return "Deployers";
    }

    @Override
    public MasqHardware[] getComponents() {
        return new MasqHardware[]{redDeployer, blueDeployer};
    }
}
