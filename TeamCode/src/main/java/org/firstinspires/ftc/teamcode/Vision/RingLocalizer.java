package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingLocalizer extends OpenCvPipeline {
    Mat HSVMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound = 0;
    //public Scalar lowerHSV = new Scalar(20, 90, 255);
    //public Scalar upperHSV = new Scalar(36, 230, 255);

    public Scalar lowerHSV = new Scalar(25, 180, 240);
    public Scalar upperHSV = new Scalar(45, 255, 255);

    public double threshold = 100;

    public double blurConstant = 15;
    public double dilationConstant = 10;
    private boolean first = true;
    ElapsedTime time = new ElapsedTime();

    private Bitmap image;

    private boolean isCenter = false;
    private double minDist = 0;

    @Override
    public Mat processFrame(Mat input) {
        if(first){
            time.startTime();
            first = false;
        }

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

        Imgproc.findContours(HSVMat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);

        ArrayList<Rect> rings = new ArrayList<>();
        minDist = 1000000000;
        int closestRingIndex = 0;
        int counter = 0;

        for (MatOfPoint contour : contoursList) {
            Rect rect = Imgproc.boundingRect(contour);

            // Show chosen result
            if(rect.area() >= threshold) {
                rings.add(rect);
                double F = (180 * 9.275)/5;
                double dist = (5 * F)/rect.width;
                Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(contoursOnFrameMat, String.valueOf(dist), rect.tl(), 0, 0.5, new Scalar(255, 255, 255));
                if(dist < minDist){
                    minDist = dist;
                    closestRingIndex = counter;
                }
                counter++;
            }
        }

        if(rings.size() != 0){
            double distToCenter = Math.abs((rings.get(closestRingIndex).x + (Math.abs(rings.get(closestRingIndex).width)/2)) - (input.width()/2));

            if(distToCenter < 10){
                isCenter = true;
                Imgproc.putText(contoursOnFrameMat, "in center: " + distToCenter, rings.get(closestRingIndex).br(), 0, 0.5, new Scalar(255, 255, 255));
            }else{
                isCenter = false;
                Imgproc.putText(contoursOnFrameMat, "not in center: " + distToCenter, rings.get(closestRingIndex).br(), 0, 0.5, new Scalar(255, 255, 255));
            }
        }

        image = Bitmap.createBitmap(contoursOnFrameMat.cols(), contoursOnFrameMat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(contoursOnFrameMat, image);

        return input;
    }

    public boolean isCenter()
    {
        return isCenter;
    }

    public Bitmap getImage(){
        return image;
    }

    public double getMinDist(){
        return Math.sqrt(Math.pow(minDist, 2) - Math.pow(9.25, 2));
    }
}
