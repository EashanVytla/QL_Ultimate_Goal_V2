package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class PurePursuitTester extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();
    }

    @Override
    public void loop() {
        robot.updateBulkData();
        ArrayList<CurvePoint> points = new ArrayList<>();

        points.add(new CurvePoint(new Pose2d(20, 55, 0), 1, 1, 10));
        points.add(new CurvePoint(new Pose2d(0, 55, 0), 1, 1, 10));
        points.add(new CurvePoint(new Pose2d(0, 0, 0), 1, 1, 10));

        RobotMovement.followCurve(points, robot, telemetry);
    }
}
