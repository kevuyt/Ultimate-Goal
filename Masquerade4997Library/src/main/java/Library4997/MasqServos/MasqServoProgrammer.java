package Library4997.MasqServos;

import com.qualcomm.robotcore.util.Range;

import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;

import static Library4997.MasqResources.MasqUtils.getLinearOpMode;
import static Library4997.MasqWrappers.DashBoard.getDash;

/**
 * Created by Keval Kataria on 12/30/2020
 */

public class MasqServoProgrammer implements  Runnable{
    MasqController controller = getLinearOpMode().getDefaultController();
    MasqServo servo1, servo2;
    String name1, name2;
    double val1, val2;
    int numServos;
    DashBoard dash = getDash();
    public MasqServoProgrammer(MasqServo servo1, MasqServo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
        name1 = servo1.getName();
        name2 = servo2.getName();
        numServos = 2;
    }
    public MasqServoProgrammer(MasqServo servo1) {
        this.servo1 = servo1;
        name1 = servo1.getName();
        numServos = 1;
    }

    @Override
    public void run() {
        if (controller.a()) val1 += 0.001;
        else if (controller.b()) val1 -= 0.001;

        if (numServos == 2) {
            if (controller.y()) val2 += 0.001;
            else if (controller.x()) val2 -= 0.001;
        }

        val1 = Range.clip(val1, 0,1);
        val2 = Range.clip(val2, 0,1);

        servo1.setPosition(val1);
        dash.create(name1 + " (+A, -B): ", val1);

        if(numServos == 2) {
            servo2.setPosition(val2);
            dash.create(name2 + " (+Y, -X): ", val2);
        }

        dash.update();
    }
}
