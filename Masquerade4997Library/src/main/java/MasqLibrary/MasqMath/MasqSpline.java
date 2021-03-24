package MasqLibrary.MasqMath;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.*;

import MasqLibrary.MasqOdometry.MasqWayPoint;

import static MasqLibrary.MasqMath.MasqPolynomial.getArcLength;
import static MasqLibrary.MasqResources.MasqUtils.negate;

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
        List<PolynomialSplineFunction> functions = new ArrayList<>();
        SplineInterpolator interpolator = new SplineInterpolator();

        x[0] = y[0] = 0;
        for(int i = 0; i< points.length; i++) x[i + 1] = points[i].getX();
        for(int i = 0; i< points.length; i++) y[i + 1] = points[i].getY();

        try{functions.add(interpolator.interpolate(x, y));} catch(Exception e) {e.printStackTrace();}
        try{functions.add(interpolator.interpolate(y, x));} catch(Exception e) {e.printStackTrace();}
        try{functions.add(interpolator.interpolate(negate(y), x));} catch(Exception e) {e.printStackTrace();}
        try{functions.add(interpolator.interpolate(y,negate(x)));} catch(Exception e) {e.printStackTrace();}
        try{functions.add(interpolator.interpolate(negate(y),negate(x)));} catch(Exception e) {e.printStackTrace();}

        return functions.toArray(new PolynomialSplineFunction[0]);
    }
}