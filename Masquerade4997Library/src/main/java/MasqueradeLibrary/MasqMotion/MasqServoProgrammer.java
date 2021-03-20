package MasqueradeLibrary.MasqMotion;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import MasqueradeLibrary.MasqMath.MasqVector;
import MasqueradeLibrary.MasqResources.DashBoard;

import static MasqueradeLibrary.MasqOdometry.MasqPositionTracker.DeadWheelPosition.TANK;
import static MasqueradeLibrary.MasqResources.DashBoard.getDash;
import static MasqueradeLibrary.MasqResources.MasqUtils.getLinearOpMode;
import static MasqueradeLibrary.MasqResources.MasqUtils.getTracker;
import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static java.util.Locale.US;

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

    public void init() {
        dash.create("Which servos do you want to test?");
        String[] testServos = {"No", "No", "No", "No", "No"};

        if (controller.a) testServos[0] = "Yes";
        else if (controller.b) testServos[0] = "No";
        dash.create(names[0] + " (Yes: A, No: B):", testServos[0]);

        if (numServos > 1) {
            if (controller.y) testServos[1] = "Yes";
            else if (controller.x) testServos[1] = "No";
            dash.create(names[1] + " (Yes: Y, No: X):", testServos[1]);
        }

        if (numServos > 2) {
            if (controller.right_bumper) testServos[2] = "Yes";
            else if (controller.left_bumper) testServos[2] = "No";
            dash.create(names[2] + " (Yes: RB, No: LB):", testServos[2]);
        }

        if (numServos > 3) {
            if (controller.right_trigger > 0) testServos[3] = "Yes";
            else if (controller.left_trigger > 0) testServos[3] = "No";
            dash.create(names[3] + " (Yes: RT, No: LT):", testServos[3]);
        }

        if (numServos > 4) {
            if (controller.left_stick_y > 0) testServos[4] = "Yes";
            else if (controller.left_stick_y < 0) testServos[4] = "No";
            dash.create(names[4] + " (Yes: LUp, No: LDown):", testServos[4]);
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
            servos[2].setPosition(val3);
            dash.create(names[2] + " (+RB, -LB): ", val3);
        }

        if(numServos > 3) {
            servos[3].setPosition(val4);
            dash.create(names[3] + " (+RT, -LT): ", val4);
        }

        if(numServos > 4) {
            servos[4].setPosition(val5);
            dash.create(names[4] + " (+LUp, -LDown): ", val5);
        }

        dash.update();
    }
}
