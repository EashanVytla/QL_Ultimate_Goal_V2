package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetectionPipelineV2 extends OpenCvPipeline
{
    OpenCvCamera webcam;
    public boolean viewportPaused;

    Mat HSVMat = new Mat();
    Mat HMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private volatile int ringCase = 0;

    Rect ringRect = new Rect();

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();

        Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);

        //FOR RINGS:
        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);

        //FOR RINGS:
        Core.inRange(HSVMat, new Scalar(0, 96, 0), new Scalar(77, 255, 255), thresholdMat);

        //Imgproc.threshold(yCbCrChan2MatRing, thresholdMatRing, 50, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);
        //Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 2, 8);


        /////////////////////////////////////////////////////////////////////////////////////////////////////////

        //FOR RINGS:
        //Core.inRange(HSVMat, new Scalar(2, 255, 170), new Scalar(32, 255, 255), thresholdMat);

        //Core.extractChannel(HSVMat, HMat, 0);

        //Imgproc.threshold(HMat, thresholdMat, 32, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);
        //Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 2, 8);
        boolean contoursExist = false;

        for (MatOfPoint contour : contoursList) {
            Rect rect = Imgproc.boundingRect(contour);


            //[0, 96, 0], [77, 255, 255]
            // Show chosen result
            if(rect.area() >= 2000) {
                Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.area()), rect.tl(), 0, 0.5, new Scalar(255, 255, 255));
            }

            if(rect.area() < 7000 && rect.area() > 2000){
                contoursExist = true;
                ringRect = rect;
            }
        }

        dashboardTelemetry.addData("rinlg rect area", ringRect.area());

        if(ringRect.area() > VisionConstants.FOUR_RING_THRESHOLD){
            ringCase = 4;
        }else if(ringRect.area() > VisionConstants.ONE_RING_THRESHOLD){
            ringCase = 1;
        }else{
            ringCase = 0;
        }

        if(!contoursExist){
            ringCase = 0;
        }

        dashboardTelemetry.update();

        return contoursOnFrameMat;
    }

    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }

    public int getAnalysis()
    {
        return ringCase;
    }
}
