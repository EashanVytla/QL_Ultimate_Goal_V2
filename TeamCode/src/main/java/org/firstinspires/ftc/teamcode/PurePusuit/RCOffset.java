package org.firstinspires.ftc.teamcode.PurePusuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Math.Vector2;

public class RCOffset {
    public double left = 0.0;
    public double right = 0.0;
    public double x = 0.0;
    public double y = 0.0;
    public double dtheta = 0.0;

    public RCOffset(double left, double right, double x, double y, double dtheta){
        this.left = left;
        this.right = right;
        this.x = x;
        this.y = y;
        this.dtheta = dtheta;
    }

    public Vector2 toVec(){
        return new Vector2(
                this.x,
                this.y
        );
    }

    public Pose2d toPose(){
        return new Pose2d(
                this.x,
                this.y,
                dtheta
        );
    }
}
