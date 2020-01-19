package Library4997.MasqControlSystems.MasqSplineGeneration;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Archishmaan Peyyety on 2020-01-19.
 * Project: MasqLib
 */
public class MasqPolynomial {
    //Coeffs are ordered in standard form i.e greatest to smallest degree
    private List<Integer> coeffs = new ArrayList<>();
    private int n;

    public double getOutput(double input) {
        int i = 0;
        int output = 0;
        while (i < n) {
            output += coeffs.get(i) * Math.pow(input, n - i - 1);
            i++;
        }
        return output;
    }

    public void setCoeff(int term, int coeff) {
        try {
            coeffs.set(term - 1, coeff);
        } catch (IndexOutOfBoundsException e) {
            n++;
            coeffs.add(term - 1, coeff);
        }
    }

    public void reset() {
        coeffs.clear();
    }
}
