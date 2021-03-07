package Library4997.MasqPath;

import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.exception.NoDataException;
import org.apache.commons.math3.exception.NullArgumentException;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static org.apache.commons.math3.analysis.integration.SimpsonIntegrator.SIMPSON_MAX_ITERATIONS_COUNT;

/**
 * Created by Keval Kataria on 3/7/2021
 */
public class MasqPolynomial extends PolynomialFunction {

    public MasqPolynomial(double[] c) throws NullArgumentException, NoDataException {
        super(c);
    }


    //Use this for arc length only. To find real value use "output(x);"
    @Override
    public double value(double x) {
        return sqrt(1 - pow(polynomialDerivative().value(x), 2));
    }

    public double output(double x) {return evaluate(getCoefficients(), x);}

    public static double getArcLength(PolynomialSplineFunction function, double maxX) {
        SimpsonIntegrator integrator = new SimpsonIntegrator();
        PolynomialFunction[] polynomials = function.getPolynomials();

        for (int i = 0; i < polynomials.length; i++) {
            polynomials[i] = new MasqPolynomial(polynomials[i].getCoefficients());
        }

        function = new PolynomialSplineFunction(function.getKnots(), polynomials);
        return integrator.integrate(SIMPSON_MAX_ITERATIONS_COUNT, function, 0, maxX);
    }
}
