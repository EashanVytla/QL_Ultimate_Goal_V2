package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;

@Config
public class VisionConstants {
    public static Point REGION_TOPLEFT_ANCHOR_POINT = new Point(145, 100);

    public static int REGION_WIDTH = 60;
    public static int REGION_HEIGHT = 27;

    public static int FOUR_RING_THRESHOLD = 120;
    public static int ONE_RING_THRESHOLD = 120;
}
