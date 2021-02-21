package Library4997.MasqServos;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import Library4997.MasqResources.DashBoard;

import static Library4997.MasqUtils.getLinearOpMode;
import static Library4997.MasqResources.DashBoard.getDash;
import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

/**
 * Created by Keval Kataria on 12/30/2020
 */

public class MasqServoProgrammer {
    Gamepad controller = getLinearOpMode().getDefaultController();
    MasqServo[] servos;
    String[] names;
    double val1, val2, val3, val4, val5;
    int numServos;
    DashBoard dash = getDash();

    public MasqServoProgrammer(MasqServo... servos) {
        names = new String[servos.length];
        this.servos = servos;
        for (int i = 0; i < servos.length; i++) names[i] = servos[i].getName();
        numServos = servos.length;
        for (MasqServo servo : servos) {
            servo.scaleRange(0,1);
            servo.setDirection(FORWARD);
        }
    }

    public void run() {
        if (controller.a) val1 += 0.0001;
        else if (controller.b) val1 -= 0.0001;

        if (numServos > 1) {
            if (controller.y) val2 += 0.0001;
            else if (controller.x) val2 -= 0.0001;
        }

        if (numServos > 2) {
            if (controller.right_bumper) val3 += 0.0001;
            else if (controller.left_bumper) val3 -= 0.0001;
        }

        if (numServos > 3) {
            if (controller.right_trigger > 0) val4 += 0.0001;
            else if (controller.left_trigger > 0) val4 -= 0.0001;
        }

        if (numServos > 4) {
            if (controller.left_stick_y > 0) val5 += 0.0001;
            else if (controller.left_stick_y < 0) val5 -= 0.0001;
        }

        val1 = Range.clip(val1, 0,1);
        val2 = Range.clip(val2, 0,1);
        val3 = Range.clip(val3, 0,1);
        val4 = Range.clip(val4, 0,1);
        val5 = Range.clip(val5, 0,1);


        servos[0].setPosition(val1);
        dash.create(names[0] + " (+A, -B): ", val1);

        if(numServos > 1) {
            servos[1].setPosition(val2);
            dash.create(names[1] + " (+Y, -X): ", val2);
        }

        if(numServos > 2) {
            servos[2].setPosition(val2);
            dash.create(names[2] + " (+RB, -LB): ", val2);
        }

        if(numServos > 3) {
            servos[3].setPosition(val2);
            dash.create(names[3] + " (+RT, -LT): ", val2);
        }

        if(numServos > 4) {
            servos[4].setPosition(val2);
            dash.create(names[4] + " (+LUp, -LDown): ", val2);
        }

        dash.update();
    }
}
