package org.firstinspires.ftc.teamcode.MPC.geometry;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
