package MasqLibrary.MasqMotion;

import com.qualcomm.robotcore.hardware.Gamepad;

import MasqLibrary.MasqResources.DashBoard;

import static MasqLibrary.MasqResources.MasqUtils.*;
import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

/**
 * Created by Keval Kataria on 12/30/2020
 */

public class MasqServoProgrammer {
    Gamepad controller = getLinearOpMode().getDefaultController();
    MasqServo[] servos;
    String[] names;
    double[] values;
    int numServos;
    DashBoard dash = getDash();
    String[] testServos;
    boolean first = true;

    public MasqServoProgrammer(MasqServo... servos) {
        numServos = servos.length;
        values = new double[numServos];
        names = new String[numServos];
        testServos = new String[numServos];
        this.servos = servos;

        for (int i = 0; i < numServos; i++) {
            names[i] = servos[i].getName();
            servos[i].scaleRange(0,1);
            servos[i].setDirection(FORWARD);
            testServos[i] = "No";
        }

    }

    public void init() {
        dash.create("Which servos do you want to test?");

        if(numServos > 0) {
            if (controller.a) testServos[0] = "Yes";
            else if (controller.b) testServos[0] = "No";
            dash.create(names[0] + " (Yes: A, No: B):", testServos[0]);
        }

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
            if (controller.left_stick_y < 0) testServos[4] = "Yes";
            else if (controller.left_stick_y > 0) testServos[4] = "No";
            dash.create(names[4] + " (Yes: LUp, No: LDown):", testServos[4]);
        }

        dash.update();
    }

    public void run() {
        if(first) for(int i = 0; i < numServos; i++) if(!testServos[i].equals("Yes")) servos[i].disable();
        first = false;

        if(numServos > 0 && testServos[0].equals("Yes")) {
            if (controller.a) values[0] += 0.0001;
            else if (controller.b) values[0] -= 0.0001;
            dash.create(names[0] + " (+A, -B): ", values[0]);
        }

        if (numServos > 1 && testServos[1].equals("Yes")) {
            if (controller.y) values[1] += 0.0001;
            else if (controller.x) values[1] -= 0.0001;
            dash.create(names[1] + " (+Y, -X): ", values[1]);
        }

        if (numServos > 2 && testServos[2].equals("Yes")) {
            if (controller.right_bumper) values[2] += 0.0001;
            else if (controller.left_bumper) values[2] -= 0.0001;
            dash.create(names[2] + " (+RB, -LB): ", values[2]);
        }

        if (numServos > 3 && testServos[3].equals("Yes")) {
            if (controller.right_trigger > 0) values[3] += 0.0001;
            else if (controller.left_trigger > 0) values[3] -= 0.0001;
            dash.create(names[3] + " (+RT, -LT): ", values[3]);
        }

        if (numServos > 4 && testServos[4].equals("Yes")) {
            if (controller.left_stick_y < 0) values[4] += 0.0001;
            else if (controller.left_stick_y > 0) values[4] -= 0.0001;
            dash.create(names[4] + " (+LUp, -LDown): ", values[4]);
        }

        clip(values);
        for(int i = 0; i < numServos; i++) if(testServos[i].equals("Yes")) servos[i].setPosition(values[i]);

        dash.update();
    }
}