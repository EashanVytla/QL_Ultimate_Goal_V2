package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

public class CalibUtil {
    public static MatOfPoint3f generateChessboardPoints(int horizontalCount, int verticalCount,
                                                        double squareLength) {
        MatOfPoint3f points = new MatOfPoint3f();
        for (int i = 0; i < horizontalCount * verticalCount; i++) {
            points.push_back(new MatOfPoint3f(
                    new Point3((i / horizontalCount) * squareLength, (i % horizontalCount) * squareLength, 0)));
        }
        return points;
    }
}