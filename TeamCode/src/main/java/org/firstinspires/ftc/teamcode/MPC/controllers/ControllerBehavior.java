package org.firstinspires.ftc.teamcode.MPC.controllers;

public enum ControllerBehavior {
    AGRESSIVE(1000d), STANDARD(100d), SLOW(10d);

    final double costFactor;

    ControllerBehavior(final double costFactor) {
        this.costFactor = costFactor;
    }

    public double getCostFactor() {
        return costFactor;
    }
}
