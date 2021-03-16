package MasqueradeLibrary.MasqMath;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.*;

import static MasqueradeLibrary.MasqMath.MasqPolynomial.getArcLength;
import static MasqueradeLibrary.MasqResources.MasqUtils.negate;

/**
 * Created by Keval Kataria on 3/6/2021
 */
public class MasqSpline {
    public static PolynomialSplineFunction getSpline(MasqWayPoint... points) {
        PolynomialSplineFunction[] potentials = getPotentialSplines(points);
        double maxX = points[points.length - 1].getX();

        PolynomialSplineFunction function = potentials[0];
        for(PolynomialSplineFunction spline : potentials) if(getArcLength(spline, maxX) < getArcLength(function, maxX)) function = spline;

        return function;
    }

    public static PolynomialSplineFunction[] getPotentialSplines(MasqWayPoint... points) {
        double[] x = new double[points.length];
        double[] y = new double[points.length];
        PolynomialSplineFunction[] functions = new PolynomialSplineFunction[5];
        SplineInterpolator interpolator = new SplineInterpolator();

        x[0] = y[0] = 0;
        for(int i = 0; i< points.length; i++) x[i + 1] = points[i].getX();
        for(int i = 0; i< points.length; i++) y[i + 1] = points[i].getY();

        try{functions[0] = interpolator.interpolate(x, y);}
        catch(Exception e) {functions[0] = null;}

        try{functions[1] = interpolator.interpolate(y, x);}
        catch(Exception e) {functions[1] = null;}

        try{functions[2] = interpolator.interpolate(negate(y), x);}
        catch(Exception e) {functions[2] = null;}
        try{functions[3] = interpolator.interpolate(y,negate(x));}
        catch(Exception e) {functions[3] = null;}

        try{functions[4] = interpolator.interpolate(negate(y),negate(x));}
        catch(Exception e) {functions[4] = null;}

        List<PolynomialSplineFunction> polynomials = new ArrayList<>();
        for (PolynomialSplineFunction function : functions) if (function != null) polynomials.add(function);
        polynomials.toArray(functions);

        return functions;
    }
}