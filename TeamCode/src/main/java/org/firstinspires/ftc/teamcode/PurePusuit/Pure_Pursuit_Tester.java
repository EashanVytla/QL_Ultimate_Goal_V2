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
        allPoints.add(new CurvePoint(new Pose2d(0, 0, 0), 1, 1, 15));
        allPoints.add(new CurvePoint(new Pose2d(0, 5, 0), 1, 1, 15));
        allPoints.add(new CurvePoint(new Pose2d(8, 24, 0), 1, 1, 15));
        allPoints.add(new CurvePoint(new Pose2d(12, 63, 0), 1, 1, 15));
        allPoints.add(new CurvePoint(new Pose2d(-24, 40, 0), 1, 1, 15));
    }

    @Override
    public void loop() {
        robot.updateBulkData();

        RobotMovement.followCurve(allPoints, robot, telemetry);
    }
}

@Config
class Positions {
    public static Point CLEAR_STACK = new Point(8, 24);
    public static Point ZONE_1 = new Point(13.464, 61.245);
    public static Point ZONE_2 = new Point(-5.923, 87.501);
    public static Point ZONE_3 = new Point(15.898, 106.822);
    public static Point POWER_SHOTS = new Point(28.505, -46.955);
    public static Point WOBBLE_GOAL_2 = new Point(0, 0);
    public static Point CLEAR_STACK_2 = new Point(0, 0);
    public static Point PARK = new Point(0, 0);

    public static double CLEAR_STACK_HEADING = 0;
    public static double ZONE_1_HEADING = 0;
    public static double ZONE_2_HEADING = 0;
    public static double ZONE_3_HEADING = 0;
    public static double POWER_SHOTS_HEADING = Math.toRadians(180);
    public static double WOBBLE_GOAL_2_HEADING = 0;
    public static double CLEAR_STACK_2_HEADING = 0;
    public static double PARK_HEADING = 0;
}

