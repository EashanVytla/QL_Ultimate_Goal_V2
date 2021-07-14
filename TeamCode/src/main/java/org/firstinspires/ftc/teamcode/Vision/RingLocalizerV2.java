package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector3;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

@Config
public class RingLocalizerV2 extends OpenCvPipeline {
    List<MatOfPoint> contoursList = new ArrayList<>(); //Array of all contours
    int numContoursFound = 0;

    public static Point3 relPoint = new Point3();

    //Extrinsic Calibration Parameters
    public static Mat RVEC = new MatOfDouble(2.035399804462858, 1.041272664535304, -0.5618792670272742); // Rotation Vector Of Camera found through Calibration
    public static Mat TVEC = new MatOfDouble(-11.36860632737538, -1.7354899166739, 31.81319062907322); // Translation Vector Of Camera from origin found through Calibration

    //Intrinsic Calibration Parameters
    public static CalibrationParameters CALIB_PARAMS = new CalibrationParameters(1010.7, 1010.7,
            638.2174071943463, 340.2355228022204, 0.2126095014133333, -1.001796829192392,
            0.000504850246603286, -0.0001913578573509387, 1.419425306492814);

    public double RING_HEIGHT = 1.0;

    private Mat rotation = new Mat(3, 3, CvType.CV_64FC1);
    private Mat uvPoint = new Mat(3, 1, CvType.CV_64FC1); //Pizel point on camera view
    private Mat lhs = new Mat();
    private Mat rhs = new Mat();
    private Mat pointMat = new Mat();

    //Simply for FTC Dashboard Purposes
    public static double lowerH = 15;
    public static double lowerS = 90;
    public static double lowerV = 150;
    public static double upperH = 25;
    public static double upperS = 255;
    public static double upperV = 255;

    //Parameters to define the object as a ring
    public static double MIN_AREA = 100;
    public static double MIN_CIRCULARITY = 0.5;
    public static double MIN_INERTIA_RATIO = 0.06;
    public static double MIN_CONVEXITY = 0.9;

    //HSV Range
    private Scalar lowerHSV = new Scalar(lowerH, lowerS, lowerV);
    private Scalar upperHSV = new Scalar(upperH, upperS, upperV);

    //Blur and Dilation Constants for ring detection
    public static double blurConstant = 5;
    public static double dilationConstant = 2;

    private MatOfPoint2f contour2f = new MatOfPoint2f();
    private MatOfInt hullIndices = new MatOfInt();
    private MatOfPoint convexHull = new MatOfPoint();
    private MatOfPoint2f undistortedPoints = new MatOfPoint2f();

    private Bitmap image;
    private int viewmode = 0;
    private Telemetry telemetry;

    private Mat HSVMat = new Mat();
    private Mat rangeMat = new Mat();
    private Mat denoisedMat = new Mat();
    private Mat outputMat = new Mat();

    private final Mat EMPTY_MAT = new Mat();

    public RingLocalizerV2(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public void onViewportTapped() {
        //Toggling through view modes on viewport tapping
        viewmode++;
    }


    //Measures how close to a circle the blob is.
    private double calculateCircularity(MatOfPoint contour, double area) {
        contour2f.fromArray(contour.toArray());
        double perimeter = Imgproc.arcLength(contour2f, true);
        return 4 * Math.PI * area / (perimeter * perimeter);
    }

    //measures how elongated a shape is. E.g. for a circle, this value is 1, for an ellipse it is between 0 and 1, and for a line it is 0.
    private double calculateInertiaRatio(Moments moments) {
        double denominator = Math.hypot(2 * moments.mu11, moments.mu20 - moments.mu02);
        double epsilon = 1e-2;

        if (denominator > epsilon) {
            double cosmin = (moments.mu20 - moments.mu02) / denominator;
            double sinmin = 2 * moments.mu11 / denominator;
            double cosmax = -cosmin;
            double sinmax = -sinmin;
            double component = 0.5 * (moments.mu20 + moments.mu02);
            double imin = component - 0.5 * (moments.mu20 - moments.mu02) * cosmin - moments.mu11 * sinmin;
            double imax = component - 0.5 * (moments.mu20 - moments.mu02) * cosmax - moments.mu11 * sinmax;
            return imin / imax;
        } else {
            return 1;
        }
    }

    //Convexity is defined as the (Area of the Blob / Area of it’s convex hull).
    private double calculateConvexity(MatOfPoint contour, double area) {
        Imgproc.convexHull(contour, hullIndices);
        Point[] contourArray = contour.toArray();
        convexHull.fromList(
                hullIndices.toList().stream().map(idx -> contourArray[idx]).collect(Collectors.toList()));
        double hullArea = Imgproc.contourArea(convexHull);
        return area / hullArea;
    }

    //Calculating the center of all points
    private Point calculateCentroid(Moments moments) {
        double x = moments.m10 / moments.m00;
        double y = moments.m01 / moments.m00;
        return new Point(x, y);
    }

    @Override
    public Mat processFrame(Mat input) {
        //Clearing the array list every loop cycle to prevent build up of elements
        contoursList.clear();

        //Converting the color space from RBG to HSV
        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV_FULL);

        //Filtering our every color but the one we want(rings)
        Core.inRange(HSVMat, lowerHSV, upperHSV, HSVMat);

        //Creating a Gaussian Blur on the image to reduce noise
        Imgproc.GaussianBlur(HSVMat, HSVMat, new Size(blurConstant, blurConstant), 0);

        //Creating the kernal of 15, 15 structuring element
        Mat kernal = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2 * dilationConstant + 1, 2 * dilationConstant + 1));
        //Dilating the image parameters are source, destination, and kernal
        Imgproc.dilate(HSVMat, HSVMat, kernal);

        //Fingding the contours based on the HSV ranges
        Imgproc.findContours(HSVMat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();

        ArrayList<Rect> rings = new ArrayList<>();

        for (MatOfPoint contour : contoursList) {
            Moments moments = Imgproc.moments(contour);

            double area = moments.m00;

            if (area < MIN_AREA) {
                continue;
            }

            //For more description on this criteria:
            //https://learnopencv.com/blob-detection-using-opencv-python-c/

            double circularity = calculateCircularity(contour, area);

            if (circularity < MIN_CIRCULARITY) {
                continue;
            }

            double inertiaRatio = calculateInertiaRatio(moments);

            if (inertiaRatio < MIN_INERTIA_RATIO) {
                continue;
            }

            double convexity = calculateConvexity(contour, area);

            if (convexity < MIN_CONVEXITY) {
                continue;
            }

            Point centroid = calculateCentroid(moments);

            // undistort centroid based on camera parameters and reproject with identity matrix
            Mat cameraMatrix = CALIB_PARAMS.getCameraMatrix();
            MatOfDouble distCoeffs = CALIB_PARAMS.getDistCoeffs();

            Calib3d.undistortPoints(new MatOfPoint2f(centroid), undistortedPoints, cameraMatrix, distCoeffs);
            Point undistortedPoint = undistortedPoints.toArray()[0];

            // rotation matrix from rotation vector
            Calib3d.Rodrigues(RVEC, rotation);

            // this based on the projection math in https://stackoverflow.com/questions/12299870 (the question)
            //Essentially reverse engineering the formula to calculate the world -> camera to go from camera -> world
            // by producing a line(similar triangles) to account for the reproduction of the loss of z axis
            uvPoint.put(0, 0, new double[] { undistortedPoint.x, undistortedPoint.y, 1 });

            // camera matrix not involved as undistortPoints reprojects with identity camera matrix
            Core.gemm(rotation.inv(), uvPoint, 1, EMPTY_MAT, 0, lhs);
            Core.gemm(rotation.inv(), TVEC, 1, EMPTY_MAT, 0, rhs);

            double s = (RING_HEIGHT / 2 + rhs.get(2, 0)[0]) / lhs.get(2, 0)[0];
            Core.scaleAdd(uvPoint, -s, TVEC, pointMat);
            Core.gemm(rotation.inv(), pointMat, -1, EMPTY_MAT, 0, pointMat);

            Imgproc.drawContours(outputMat, Collections.singletonList(contour), -1, new Scalar(255, 0, 0), 3);

            double[] buff = new double[3];
            pointMat.get(0, 0, buff);
            Point3 point = new Point3(buff);
            Point3 adjustedPoint = new Point3(point.x + relPoint.x, point.y + relPoint.y, point.z + relPoint.z);

            telemetry.addData("point", point);
            telemetry.update();
        }

        Mat[] mats = new Mat[] { outputMat, denoisedMat, input };
        image = Bitmap.createBitmap(mats[viewmode % mats.length].cols(), mats[viewmode % mats.length].rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mats[viewmode % mats.length], image);
        return mats[viewmode % mats.length];
    }

    public Bitmap getImage(){
        if(image != null){
            return image;
        }

        return Bitmap.createBitmap(10, 10, Bitmap.Config.ARGB_8888);
    }
}
