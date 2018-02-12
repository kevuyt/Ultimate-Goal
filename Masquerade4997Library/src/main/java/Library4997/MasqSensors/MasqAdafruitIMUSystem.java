package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqUtilities.MasqHardware;


/**
 * Created by Archish on 1/7/18.
 */

public class MasqAdafruitIMUSystem implements MasqHardware {
    private MasqAdafruitIMU imuHubOne;
    private MasqAdafruitIMU imuHubTwo;
    public MasqAdafruitIMUSystem (String hubOne, String hunTwo, HardwareMap hardwareMap) {
        imuHubOne = new MasqAdafruitIMU(hubOne, hardwareMap);
        imuHubTwo = new MasqAdafruitIMU(hunTwo, hardwareMap);
    }
    public double getHeading() {
        return (imuHubOne.getHeading() + imuHubTwo.getHeading())/2;
    }
    public double getYaw () {
        return (imuHubOne.getYaw() + imuHubTwo.getYaw())/2;
    }
    public void reset(){
        imuHubTwo.reset();
        imuHubTwo.reset();
    }
    public double adjustAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    @Override
    public String getName() {
        return "System";
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "Heading:" + Double.toString(getHeading()),
        };
    }
}
