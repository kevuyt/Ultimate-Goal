package MasqLibrary.MasqMath;

import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.polynomials.*;
import org.apache.commons.math3.exception.*;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static org.apache.commons.math3.analysis.integration.SimpsonIntegrator.SIMPSON_MAX_ITERATIONS_COUNT;

/**
 * Created by Keval Kataria on 3/7/2021
 */
public class MasqPolynomial extends PolynomialFunction {
    private final boolean arcLength;
    public MasqPolynomial(double[] c, boolean arcLength) throws NullArgumentException, NoDataException {
        super(c);
        this.arcLength = arcLength;
    }

    @Override
    public double value(double x) {
        return arcLength ? sqrt(1 + pow(polynomialDerivative().value(x), 2)) : super.value(x);
    }

    public static double getArcLength(PolynomialSplineFunction function, double maxX) {
        SimpsonIntegrator integrator = new SimpsonIntegrator();
        PolynomialFunction[] polynomials = function.getPolynomials();

        for(int i = 0; i < polynomials.length; i++) polynomials[i] = new MasqPolynomial(polynomials[i].getCoefficients(), true);

        function = new PolynomialSplineFunction(function.getKnots(), polynomials);
        return integrator.integrate(SIMPSON_MAX_ITERATIONS_COUNT, function, 0, maxX);
    }
}