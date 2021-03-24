package org.firstinspires.ftc.teamcode.PurePusuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous
public class PurePursuitTesterLinear extends LinearOpMode {
    Robot robot;
    ArrayList<CurvePoint> allPoints = new ArrayList<>();

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()){
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(new Pose2d(0, 0, 0), 1, 1, 15));
            allPoints.add(new CurvePoint(new Pose2d(0, 5, 0), 1, 1, 15));
            allPoints.add(new CurvePoint(new Pose2d(8, 24, 0), 1, 1, 15));
            allPoints.add(new CurvePoint(new Pose2d(12, 63, 0), 1, 1, 15));
            allPoints.add(new CurvePoint(new Pose2d(-24, 40, 0), 1, 1, 15));

            robot.updateBulkData();

            RobotMovement.followCurve(allPoints, robot, telemetry);
        }
    }
}
