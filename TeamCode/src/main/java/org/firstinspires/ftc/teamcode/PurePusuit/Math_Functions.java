package org.firstinspires.ftc.teamcode.PurePusuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;


import java.util.ArrayList;

import static java.lang.Math.*;

public class Math_Functions {
    /**
     * Makes sure an angle is within the range -180 to 180 degrees
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static Vector2d poseToVector2d(Pose2d p){
        return new Vector2d(p.getX(), p.getY());
    }

    public static ArrayList<Vector2d> lineCircleIntersection(Vector2d circleCenter, double radius,
                                                            Vector2d linePoint1, Vector2d linePoint2){
        if(Math.abs(linePoint1.getY() - linePoint2.getY()) < 0.003){
            linePoint1 = new Vector2d(linePoint1.getX(), linePoint2.getY() + 0.003);
        }
        if(Math.abs(linePoint1.getX() - linePoint2.getX()) < 0.003){
            linePoint1 = new Vector2d(linePoint2.getX() + 0.003, linePoint2.getY());
        }

        double m1 = (linePoint2.getY() - linePoint1.getY())/(linePoint2.getX() - linePoint1.getX());

        double quadraticA = 1.0 + pow(m1,2);

        double x1 = linePoint1.getX() - circleCenter.getX();
        double y1 = linePoint1.getY() - circleCenter.getY();


        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);

        double quadraticC = ((pow(m1,2) * pow(x1,2))) - (2.0*y1*m1*x1) + pow(y1,2) - pow(radius,2);

        ArrayList<Vector2d> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;


            //put back the offset
            xRoot1 += circleCenter.getX();
            yRoot1 += circleCenter.getY();

            double minX = linePoint1.getX() < linePoint2.getX() ? linePoint1.getX() : linePoint2.getX();
            double maxX = linePoint1.getX() > linePoint2.getX() ? linePoint1.getX() : linePoint2.getX();

            //System.out.println(minX);
            //System.out.println(maxX);
            //System.out.println(xRoot1);
            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Vector2d(xRoot1,yRoot1));
            }

            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;


            xRoot2 += circleCenter.getX();
            yRoot2 += circleCenter.getY();

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Vector2d(xRoot2,yRoot2));
            }
        }catch(Exception e){

        }
        return allPoints;
    }
}