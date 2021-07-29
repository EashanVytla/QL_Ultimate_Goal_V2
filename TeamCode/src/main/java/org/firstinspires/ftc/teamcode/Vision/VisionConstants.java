package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;

@Config
public class VisionConstants {
    public static Point REGION_TOPLEFT_ANCHOR_POINT = new Point(145, 100);

    public static int REGION_WIDTH = 60;
    public static int REGION_HEIGHT = 27;

    public static int FOUR_RING_THRESHOLD = 4000;
    public static int ONE_RING_THRESHOLD = 1000;

    public static double MIN_CIRCULARITY = 0.5;
    public static double MIN_INERTIA_RATIO = 0.06;
    public static double MIN_CONVEXITY = 0.9;

    public static double lowerH = 15;
    public static double lowerS = 90;
    public static double lowerV = 100;
    public static double upperH = 30;
    public static double upperS = 255;
    public static double upperV = 255;

    public static double MIN_AREA = 200;

    public static double blurConstant = 5;
    public static double dilationConstant = 2;
    public static double erosionConstant = 1;
}
