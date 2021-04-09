package org.firstinspires.ftc.teamcode.MPC.control;

import org.ejml.simple.SimpleMatrix;

public interface Costable {
    SimpleMatrix getQuadraticCost(SimpleMatrix state, int timeStep, double dt);
    SimpleMatrix getLinearCost(SimpleMatrix state, int timeStep, double dt);
}
