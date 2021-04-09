package org.firstinspires.ftc.teamcode.MPC.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.MPC.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.MPC.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUnits;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUtil;

public class Waypoint implements Costable {
    private SimpleMatrix costWeight;
    private double temporalSpread;
    private double desiredTime;
    private SimpleMatrix waypointState;
    private Translation2d location;

    public Waypoint(SimpleMatrix costWeight, double temporalSpread, double desiredTime, Pose2d pose) {
        setCostWeight(costWeight);
        setTemporalSpread(temporalSpread);
        setDesiredTime(desiredTime);
        setLocation(pose.getTranslation());
        setWaypointState(new SimpleMatrix(6, 1, true, new double[] {
                pose.getTranslation().x() * 0.0254d, 0d, pose.getTranslation().y() * 0.0254d, 0d, pose.getRotation().getRadians(), 0d
        }));
    }

    public double getTemporalCostFactor(double timeStamp) {
        return Math.sqrt(getTemporalSpread() / (2d * Math.PI)) * Math.exp(-getTemporalSpread() *
                (timeStamp - getDesiredTime()) * (timeStamp - getDesiredTime()) / 2d);
    }

    public SimpleMatrix getQuadraticCost() {
        return getQuadraticCost(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS));
    }

    public SimpleMatrix getQuadraticCost(double timeStamp) {
        return getCostWeight().scale(getTemporalCostFactor(timeStamp));
    }

    public SimpleMatrix getCostWeight() {
        return costWeight;
    }

    public void setCostWeight(SimpleMatrix costWeight) {
        this.costWeight = costWeight;
    }

    public double getTemporalSpread() {
        return temporalSpread;
    }

    public void setTemporalSpread(double temporalSpread) {
        this.temporalSpread = temporalSpread;
    }

    public double getDesiredTime() {
        return desiredTime;
    }

    public void setDesiredTime(double desiredTime) {
        this.desiredTime = desiredTime;
    }

    public SimpleMatrix getWaypointState() {
        return waypointState;
    }

    public void setWaypointState(SimpleMatrix waypointState) {
        this.waypointState = waypointState;
    }

    public Translation2d getLocation() {
        return location;
    }

    public void setLocation(Translation2d location) {
        this.location = location;
    }

    @Override
    public SimpleMatrix getQuadraticCost(SimpleMatrix state, int timeStep, double dt) {
        return getQuadraticCost(timeStep * dt);
    }

    @Override
    public SimpleMatrix getLinearCost(SimpleMatrix state, int timeStep, double dt) {
        return null;
    }
}
