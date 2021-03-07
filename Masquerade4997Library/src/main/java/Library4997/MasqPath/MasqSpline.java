package Library4997.MasqPath;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqMath.MasqWayPoint;

import static Library4997.MasqPath.MasqPolynomial.getArcLength;
import static Library4997.MasqUtils.negate;

/**
 * Created by Keval Kataria on 3/6/2021
 */
public class MasqSpline {
    private static PolynomialSplineFunction[] functions = new PolynomialSplineFunction[5];

    public PolynomialSplineFunction getSpline(MasqWayPoint... points) {
        PolynomialSplineFunction[] potentials = getPotentialSplines(points);
        PolynomialSplineFunction function = potentials[0];
        for(PolynomialSplineFunction spline : potentials) if(getArcLength(spline, points[points.length - 1].getX()) < getArcLength(function, points[points.length - 1].getX())) function = spline;
        return function;
    }

    public static PolynomialSplineFunction[] getPotentialSplines(MasqWayPoint... points) {
        double[] x = new double[points.length];
        double[] y = new double[points.length];
        SplineInterpolator interpolator = new SplineInterpolator();
        x[0] = y[0] = 0;

        for(int i = 0; i< points.length; i++) x[i + 1] = points[i].getX();
        for(int i = 0; i< points.length; i++) y[i + 1] = points[i].getY();
        try{
            functions[0] = interpolator.interpolate(x, y);
        } catch(Exception e) {
            functions[0] = null;
        }

        try{
            functions[1] = interpolator.interpolate(y, x);
        } catch(Exception e) {
            functions[1] = null;
        }

        try{
            functions[2] = interpolator.interpolate(negate(y), x);
        } catch(Exception e) {
            functions[2] = null;
        }
        try{
            functions[3] = interpolator.interpolate(y,negate(x));
        } catch(Exception e) {
            functions[3] = null;
        }
        try{
            functions[4] = interpolator.interpolate(negate(y),negate(x));
        } catch(Exception e) {
            functions[4] = null;
        }

        List<PolynomialSplineFunction> polynomials = new ArrayList<>();
        for (PolynomialSplineFunction function : functions) if (function != null) polynomials.add(function);
        polynomials.toArray(functions);

        return functions;
    }
}