package org.firstinspires.ftc.teamcode.PurePusuit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.opencv.core.Point;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous
public class Pure_Pursuit_Tester extends OpMode {
    Robot robot;
    ArrayList<CurvePoint> allPoints = new ArrayList<>();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();
        allPoints.add(new CurvePoint(new Pose2d(Positions.Position1.x, Positions.Position1.y, 0), 1, 1, 15));
        allPoints.add(new CurvePoint(new Pose2d(Positions.Position2.x, Positions.Position2.y, 0), 1, 1, 15));
        allPoints.add(new CurvePoint(new Pose2d(Positions.Position3.x, Positions.Position3.y, 0), 1, 1, 15));
        allPoints.add(new CurvePoint(new Pose2d(Positions.Position4.x, Positions.Position4.y, 0), 1, 1, 15));

    }

    @Override
    public void loop() {
        robot.updateBulkData();

        RobotMovement.followCurve(allPoints, robot, telemetry);
    }
}

@Config
class Positions {
    public static Point Position1 = new Point(0, 0);
    public static Point Position2 = new Point(0, 30);
    public static Point Position3 = new Point(12, 40);
    public static Point Position4 = new Point(-12, 40);
}

