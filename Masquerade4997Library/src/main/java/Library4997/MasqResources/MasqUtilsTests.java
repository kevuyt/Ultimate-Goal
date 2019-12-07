package Library4997.MasqResources;

import static Library4997.MasqResources.MasqUtils.adjustAngle;

/**
 * Created by Archishmaan Peyyety on 2019-12-07.
 * Project: MasqLib
 */
public class MasqUtilsTests {

    public static void main(String[] args) {
        System.out.println("Running Adjust Angle Tests");
        assert adjustAngle(70) == 70: "Adjust Angle on +70 Failed";
        assert adjustAngle(0) == 0: "Adjust Angle on +0 Failed";
        assert adjustAngle(180) == 180: "Adjust Angle on +180 Failed";
        assert adjustAngle(270) == -90: "Adjust Angle on +270 Failed";
        assert adjustAngle(720) == 0: "Adjust Angle on +270 Failed";
        assert adjustAngle(360) == 0: "Adjust Angle on +270 Failed";
        System.out.println("Adjust Angle Tests Passed");
    }

}
