package Library4997.MasqPath;


import Jama.Matrix;
import Library4997.MasqSensors.MasqPositionTracker.MasqWayPoint;

/**
 * Created by Archishmaan Peyyety on 2020-01-19.
 * Project: MasqLib
 */
public class MasqSegmentGenerator {
    private MasqPolynomial x, y, h;
    private Matrix m = new Matrix(
            new double[][]{
                    {0, 0, 0, 0, 0, 1},
                    {0, 0, 0, 0, 1, 0},
                    {0, 0, 0, 2, 0, 0},
                    {1, 1, 1, 1, 1, 1},
                    {5, 4, 3, 2, 1, 0},
                    {20, 12, 6, 2, 0, 0},
            }
    );

    public void updateSegmentPolynomials(MasqWayPoint current, MasqWayPoint destination) {
        Matrix xConstraints = new Matrix(
                new double[][]{
                        {current.getX()},
                        {current.getMinVelocity()},
                        {0},
                        {destination.getX()},
                        {destination.getMinVelocity()},
                        {0}
                }
        );
        Matrix yConstraints = new Matrix(
                new double[][]{
                        {current.getY()},
                        {current.getMinVelocity()},
                        {0},
                        {destination.getY()},
                        {destination.getMinVelocity()},
                        {0}
                }
        );
        Matrix hConstraints = new Matrix(
                new double[][]{
                        {current.getH()},
                        {0},
                        {0},
                        {destination.getH()},
                        {0},
                        {0}
                }
        );
        x = parseMatrix(m.solve(xConstraints));
        y = parseMatrix(m.solve(yConstraints));
        h = parseMatrix(m.solve(hConstraints));
    }

    public MasqPolynomial parseMatrix(Matrix m) {
        MasqPolynomial solution = new MasqPolynomial();
        solution.reset();
        solution.setCoeff(1, m.get(0, 0));
        solution.setCoeff(2, m.get(1, 0));
        solution.setCoeff(3, m.get(2, 0));
        solution.setCoeff(4, m.get(3, 0));
        solution.setCoeff(5, m.get(4, 0));
        solution.setCoeff(6, m.get(5, 0));
        return solution;
    }

    public MasqPolynomial getX() {
        return x;
    }

    public MasqPolynomial getY() {
        return y;
    }

    public MasqPolynomial getH() {
        return h;
    }
}
