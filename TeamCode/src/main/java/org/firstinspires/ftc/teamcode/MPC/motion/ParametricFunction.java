package org.firstinspires.ftc.teamcode.MPC.motion;

import org.firstinspires.ftc.teamcode.MPC.geometry.Translation2d;

public interface ParametricFunction {
    Translation2d evaluate(double parameter);

    Translation2d getDerivative(double parameter);

    Translation2d getSecondDerivative(double parameter);

    double getCurvature(double parameter);

    double getDCurvature(double parameter);

    double getMeanCurvature();

    double getMeanDCurvature();
}
