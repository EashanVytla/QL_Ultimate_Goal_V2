package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Math.Vector3;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class RingLocalizerV2 extends OpenCvPipeline {
    List<MatOfPoint> contoursList = new ArrayList<>(); //Array of all contours
    int numContoursFound = 0;

    public Point3 relPoint = new Point3(-36, 0, 0);

    //Extrinsic Calibration Parameters
    public Mat RVEC; // Rotation Vector Of Camera found through Calibration
    public Mat TVEC; // Translation Vector Of Camera from origin found through Calibration

    //Intrinsic Calibration Parameters
    public CalibrationParameters CALIB_PARAMS;
    private ArrayList<Vector2d> ringPositions = new ArrayList<>();

    public double RING_HEIGHT = 1.0;

    private Mat rotation;
    private Mat uvPoint; //Pizel point on camera view
    private Mat lhs;
    private Mat rhs;
    private Mat pointMat;

    //HSV Range
    private Scalar lowerHSV = new Scalar(VisionConstants.lowerH, VisionConstants.lowerS, VisionConstants.lowerV);
    private Scalar upperHSV = new Scalar(VisionConstants.upperH, VisionConstants.upperS, VisionConstants.upperV);

    private MatOfPoint2f contour2f = new MatOfPoint2f();
    private MatOfInt hullIndices = new MatOfInt();
    private MatOfPoint convexHull = new MatOfPoint();
    private MatOfPoint2f undistortedPoints = new MatOfPoint2f();

    private Mat HSVMat;
    private Mat outputMat;

    private final Mat EMPTY_MAT;

    public RingLocalizerV2(Telemetry telemetry){
        outputMat = new Mat();
        EMPTY_MAT = new Mat();
        HSVMat = new Mat();

        RVEC = new MatOfDouble(1.573170512100423, 1.55992815372063, -0.8995313429429516);
        TVEC = new MatOfDouble(-0.4223866364049191, 0.5307423920270103, 36.44426518615748);
        CALIB_PARAMS = new CalibrationParameters(496.222, 499.292,
                322.836, 176.195, 0.0597197, -0.0908114, 0.0153578, -0.00202418, 0.0395567);

        rotation = new Mat(3, 3, CvType.CV_64FC1);
        uvPoint = new Mat(3, 1, CvType.CV_64FC1);
        lhs = new Mat();
        rhs = new Mat();
        pointMat = new Mat();
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
    public Mat processFrame(Mat inputMat) {
        contoursList.clear();

        //Converting the color space from RBG to HSV
        Imgproc.cvtColor(inputMat, HSVMat, Imgproc.COLOR_RGB2HSV_FULL);

        //Filtering our every color but the one we want(rings)
        Core.inRange(HSVMat, lowerHSV, upperHSV, HSVMat);

        //Creating the kernal of 15, 15 structuring element
        Mat kernal = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2 * VisionConstants.dilationConstant + 1, 2 * VisionConstants.dilationConstant + 1));
        Mat kernal2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(2 * VisionConstants.erosionConstant + 1, 2 * VisionConstants.erosionConstant + 1));
        //Dilating the image parameters are source, destination, and kernal
        Imgproc.erode(HSVMat, HSVMat, kernal2);
        Imgproc.dilate(HSVMat, HSVMat, kernal);

        //Fingding the contours based on the HSV ranges
        Imgproc.findContours(HSVMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();


        inputMat.copyTo(outputMat);

        ArrayList<Vector2d> ringList = new ArrayList<>();

        Imgproc.drawContours(outputMat, contoursList, -1, new Scalar(0, 255, 0), 3);

        for (MatOfPoint contour : contoursList) {
            Moments moments = Imgproc.moments(contour);

            double area = moments.m00;

            if (area < VisionConstants.MIN_AREA) {
                continue;
            }

            //For more description on this criteria:
            //https://learnopencv.com/blob-detection-using-opencv-python-c/

            double circularity = calculateCircularity(contour, area);

            if (circularity < VisionConstants.MIN_CIRCULARITY) {
                continue;
            }

            double inertiaRatio = calculateInertiaRatio(moments);

            if (inertiaRatio < VisionConstants.MIN_INERTIA_RATIO) {
                continue;
            }

            double convexity = calculateConvexity(contour, area);

            if (convexity < VisionConstants.MIN_CONVEXITY) {
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
            ringList.add(new Vector2d(adjustedPoint.x, adjustedPoint.y));
        }

        ringPositions = ringList;

        //image = Bitmap.createBitmap(outputMat.cols(), outputMat.rows(), Bitmap.Config.ARGB_8888);
        //Utils.matToBitmap(outputMat, image);
        return outputMat;
    }

    public ArrayList<Vector2d> getRingPositions(Pose2d currentPos){
        ArrayList<Vector2d> positions = new ArrayList<>();

        for(int i = 0; i < ringPositions.size(); i++){
            if(currentPos.getY() - ringPositions.get(i).getX() < 135) {
                positions.add(new Vector2d(currentPos.getX() + (Robot.isBlue() ? -ringPositions.get(i).getY() : ringPositions.get(i).getY()), currentPos.getY() - ringPositions.get(i).getX()));
            }
        }

        return positions;
    }
}
