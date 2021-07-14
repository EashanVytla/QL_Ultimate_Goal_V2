package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Size;

import java.util.Arrays;

public class CalibrationParameters {
    public static class InvalidParametersException extends RuntimeException {
        public InvalidParametersException(String message) {
            super(message);
        }
    }

    private enum DistParamSize {
        NONE(0),
        FOUR(4),
        FIVE(5),
        EIGHT(8),
        TWELVE(12),
        FOURTEEN(14);

        private final int size;

        DistParamSize(int size) {
            this.size = size;
        }

        public int getSize() {
            return size;
        }
    }

    // matrix parameters
    public double fx;
    public double fy;
    public double cx;
    public double cy;

    // distortion parameters
    public double k1;
    public double k2;
    public double p1;
    public double p2;
    public double k3;
    public double k4;
    public double k5;
    public double k6;
    public double s1;
    public double s2;
    public double s3;
    public double s4;
    public double taux;
    public double tauy;

    private Mat cameraMat = new Mat(3, 3, CvType.CV_64FC1);

    public CalibrationParameters(double fx, double fy, double cx, double cy) {
        this(fx, fy, cx, cy, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public CalibrationParameters(double fx, double fy, double cx, double cy, double k1,
                                 double k2, double p1, double p2) {
        this(fx, fy, cx, cy, k1, k2, p1, p2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public CalibrationParameters(double fx, double fy, double cx, double cy, double k1,
                                 double k2, double p1, double p2, double k3) {
        this(fx, fy, cx, cy, k1, k2, p1, p2, k3, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public CalibrationParameters(double fx, double fy, double cx, double cy, double k1,
                                 double k2, double p1, double p2, double k3, double k4, double k5, double k6) {
        this(fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6, 0, 0, 0, 0, 0, 0);
    }

    public CalibrationParameters(double fx, double fy, double cx, double cy, double k1,
                                 double k2, double p1, double p2, double k3, double k4, double k5, double k6,
                                 double s1, double s2, double s3, double s4) {
        this(fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, 0, 0);
    }

    public CalibrationParameters(double fx, double fy, double cx, double cy, double k1,
                                 double k2, double p1, double p2, double k3, double k4, double k5, double k6,
                                 double s1, double s2, double s3, double s4, double taux, double tauy) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.k1 = k1;
        this.k2 = k2;
        this.p1 = p1;
        this.p2 = p2;
        this.k3 = k3;
        this.k4 = k4;
        this.k5 = k5;
        this.k6 = k6;
        this.s1 = s1;
        this.s2 = s2;
        this.s3 = s3;
        this.s4 = s4;
        this.taux = taux;
        this.tauy = tauy;
    }

    public static CalibrationParameters fromOCVParams(Mat cameraMat, MatOfDouble distCoeffs) {
        if (!cameraMat.size().equals(new Size(3, 3))) {
            throw new InvalidParametersException("cameraMat is not a 3x3 size");
        }
        double fx = cameraMat.get(0, 0)[0];
        double fy = cameraMat.get(1, 1)[0];
        double cx = cameraMat.get(0, 2)[0];
        double cy = cameraMat.get(1, 2)[0];

        double[] distCoeffsArr = distCoeffs.toArray();
        double[] c = Arrays.copyOf(distCoeffsArr, 14);
        return new CalibrationParameters(fx, fy, cx, cy, c[0], c[1], c[2], c[3], c[4], c[5],
                c[6], c[7], c[8], c[9], c[10], c[11], c[12], c[13]);
    }

    private DistParamSize getDistParamSize() {
        if (taux != 0 || tauy != 0) return DistParamSize.FOURTEEN;
        else if (s1 != 0 || s2 != 0 || s3 != 0 || s4 != 0) return DistParamSize.TWELVE;
        else if (k4 != 0 || k5 != 0 || k6 != 0) return DistParamSize.EIGHT;
        else if (k3 != 0) return DistParamSize.FIVE;
        else if (k1 != 0 || k2 != 0 || p1 != 0 || p2 != 0) return DistParamSize.FOUR;
        else return DistParamSize.NONE;
    }

    public MatOfDouble getDistCoeffs() {
        DistParamSize distParamSize = getDistParamSize();

        double[] params = new double[distParamSize.getSize()];

        switch (distParamSize) {
            case FOURTEEN:
                params[12] = taux;
                params[13] = tauy;
            case TWELVE:
                params[8] = s1;
                params[9] = s2;
                params[10] = s3;
                params[11] = s4;
            case EIGHT:
                params[5] = k4;
                params[6] = k5;
                params[7] = k6;
            case FIVE:
                params[4] = k3;
            case FOUR:
                params[0] = k1;
                params[1] = k2;
                params[2] = p1;
                params[3] = p2;
            case NONE:
        }

        return new MatOfDouble(params);
    }

    public Mat getCameraMatrix() {
        double[] cameraData = new double[] { fx, 0, cx, 0, fy, cy, 0, 0, 1 };
        cameraMat.put(0, 0, cameraData);
        return cameraMat;
    }

    @Override
    public String toString() {
        return String.format("(fx=%g, fy=%g, cx=%g, cy=%g, k1=%g, k2=%g, p1=%g, p2=%g, k3=%g, "
                        + "k4=%g, k5=%g, k6=%g, s1=%g, s2=%g, s3=%g, s4=%g, taux=%g, tauy=%g)",
                fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, taux, tauy);
    }
}
