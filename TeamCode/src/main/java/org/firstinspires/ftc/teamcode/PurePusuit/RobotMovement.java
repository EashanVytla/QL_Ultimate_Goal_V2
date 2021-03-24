package org.firstinspires.ftc.teamcode.PurePusuit;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Robot;

import java.util.ArrayList;

public class RobotMovement {
    private static int index = 0;
    private static int previous_index = 0;
    private static CurvePoint followMe = new CurvePoint(0, 0, 0, 0, 0, 0);
    private static Telemetry thisTelemetry;

    public static void followCurve(ArrayList<CurvePoint> allPoints, Robot robot, Telemetry telemetry){
        thisTelemetry = telemetry;
        try{
            if(index >= allPoints.size() - 2 && robot.getPos().vec().distTo(new Vector2d(allPoints.get(allPoints.size() - 1).x, allPoints.get(allPoints.size() - 1).y)) <= 25){

                followMe = allPoints.get(allPoints.size() - 1);
            }else{
                followMe = getFollowPointPath(allPoints, new Pose2d(robot.getPos().getX(), robot.getPos().getY(), robot.getPos().getHeading()), allPoints.get(index).followDistance);

            }

            index = getCurrentLine(followMe.toVec(), allPoints);
            telemetry.addData("Current Line: ", index);
            previous_index = index;

            telemetry.addData("PURE PURESUIT POS", robot.getPos());

            robot.GoTo(followMe.x, followMe.y, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).heading, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).turnSpeed);
        }catch(Exception e){
            System.out.println("Error: " + e);
        }
    }

    public static void followCurveAngled(ArrayList<CurvePoint> allPoints, Robot robot, Telemetry telemetry){
        thisTelemetry = telemetry;
        try{
            if(index >= allPoints.size() - 2 && robot.getPos().vec().distTo(new Vector2d(allPoints.get(allPoints.size() - 1).x, allPoints.get(allPoints.size() - 1).y)) <= 25){

                followMe = allPoints.get(allPoints.size() - 1);
            }else{
                followMe = getFollowPointPath(allPoints, new Pose2d(robot.getPos().getX(), robot.getPos().getY(), robot.getPos().getHeading()), allPoints.get(index).followDistance);

            }

            index = getCurrentLine(followMe.toVec(), allPoints);
            telemetry.addData("Current Line: ", index);
            previous_index = index;

            telemetry.addData("PURE PURESUIT POS", robot.getPos());

            robot.GoTo(followMe.x, followMe.y, getFollowAngle(0, allPoints.get(index), allPoints.get(Math.min(index + 1, allPoints.size() - 1))), allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).moveSpeed, allPoints.get(Math.min(index + 1, allPoints.size() - 1)).turnSpeed);
        }catch(Exception e){
            System.out.println("Error: " + e);
        }
    }

    public static double getFollowAngle(Pose2d p, CurvePoint followMe, double preferredAngle){
        double absoluteAngleToTarget = Math.atan2(followMe.y-p.getY(),followMe.x-p.getX());
        double relativeAngleToPoint = Math_Functions.AngleWrap(absoluteAngleToTarget - (p.getHeading() - Math.toRadians(90)));

        return relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
    }

    public static double getFollowAngle(double preferredAngle, CurvePoint p1, CurvePoint p2){
        double absoluteAngle = Math_Functions.AngleWrap(Math.atan2(p2.y - p1.y, p2.x - p1.x) + preferredAngle);
        return absoluteAngle;
    }

    public static void resetIndex(){
        previous_index = 0;
        index = 0;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Pose2d robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        double runningDistance = 0;
        double previous_dist = 0;
        Vector2d maxIntersection = new Vector2d(0, 0);

        for (int i = index; i < pathPoints.size() - 1; i++){
            CurvePoint start = pathPoints.get(i);
            CurvePoint end = pathPoints.get(i + 1);
            runningDistance += previous_dist;

            ArrayList<Vector2d> intersections = Math_Functions.lineCircleIntersection(new Vector2d(robotLocation.getX(), robotLocation.getY()), followRadius, start.toVec(), end.toVec());

            double closestDistance = Double.MAX_VALUE;

            for (Vector2d thisIntersection : intersections){
                double dist = Math.hypot(thisIntersection.getX() - pathPoints.get(i + 1).x, thisIntersection.getY() - pathPoints.get(i + 1).y);

                //System.out.println("INTERSECTIONS Subtraction: " + Math.abs(getCurrentLine(thisIntersection, pathPoints) - index));
                //System.out.println("Current Line of intersection: " + getCurrentLine(thisIntersection, pathPoints));
                //System.out.println("INDEX: " + index);
                if (dist < closestDistance/* && Math.abs(getCurrentLine(thisIntersection, pathPoints) - index) <= 1*/){
                    /*if(getCurrentLine(thisIntersection, pathPoints) > getCurrentLine(maxIntersection, pathPoints)){
                        maxIntersection = thisIntersection;
                    }*/
                    closestDistance = dist;
                    followMe.setPoint(thisIntersection);
                    //return followMe;
                }
            }
            previous_dist = Math.hypot(end.x - start.x, end.y - start.y);
        }
        return followMe;
    }


    public static int getCurrentLine(Vector2d intersection, ArrayList<CurvePoint> allPoints){
        int currentline = 0;

        for (int j = 0; j < allPoints.size() - 1; j++){
            Vector2d start = new Vector2d(allPoints.get(j).x, allPoints.get(j).y);
            Vector2d end = new Vector2d(allPoints.get(j + 1).x, allPoints.get(j + 1).y);
            thisTelemetry.addData("Start to vector: ", start.distTo(end));

            if (Math.abs(start.distTo(end) - (start.distTo(intersection) + end.distTo(intersection))) <= 0.003){
                if (Math.abs(j - previous_index) < 2) {
                    currentline = j;
                    return currentline;
                }
            }
        }
        return currentline;
    }
}
