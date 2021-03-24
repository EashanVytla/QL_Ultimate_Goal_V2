package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector extends OpenCvPipeline {
    private int numRings;
    private Telemetry telemetry;
    Mat matrix = new Mat();
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    static final Rect LOWER_BOUNDING_BOX = new Rect(
            new Point(60, 35),
            new Point(120, 48)
    );

    static final Rect UPPER_BOUNDING_BOX = new Rect(
            new Point(60, 48.3),
            new Point(120, 75)
    );

    public RingDetector(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, matrix, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(matrix, lowHSV, highHSV, matrix);

        Mat upperRingMatrix = matrix.submat(UPPER_BOUNDING_BOX);
        double upperRingValue = Core.sumElems(upperRingMatrix).val[0] / UPPER_BOUNDING_BOX.area() / 255;

        Mat lowerRingMatrix = matrix.submat(LOWER_BOUNDING_BOX);
        double lowerRingValue = Core.sumElems(lowerRingMatrix).val[0] / UPPER_BOUNDING_BOX.area() / 255;

        upperRingMatrix.release();
        lowerRingMatrix.release();

        telemetry.addData("Upper Ring raw value: ", (int)Core.sumElems(upperRingMatrix).val[0]);
        telemetry.addData("Lower Ring raw value: ", (int)Core.sumElems(lowerRingMatrix).val[0]);

        return upperRingMatrix;
    }
}
