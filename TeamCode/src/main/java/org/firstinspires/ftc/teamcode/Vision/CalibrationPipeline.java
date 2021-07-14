package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// Adapted from https://github.com/opencv-java/camera-calibration/blob/master/src/application/CC_Controller.java
// and https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
public class CalibrationPipeline extends OpenCvPipeline {
    public static double FRAME_INTERVAL = 1.0;
    public static int CALIB_FRAMES = 60;
    public static int HORIZONTAL_CORNERS = 9;
    public static int VERTICAL_CORNERS = 7;
    public static double SQUARE_SIZE = 0.98425197;
    public static boolean IS_CAPTURING = true;
    public static boolean USE_RO_METHOD = false;
    // https://docs.opencv.org/4.5.2/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d for details on flags
    public static int CALIB_FLAGS = Calib3d.CALIB_USE_LU;

    private ElapsedTime timer = new ElapsedTime();

    private List<Mat> imagePoints = new ArrayList<>();
    private MatOfPoint2f imageCorners = new MatOfPoint2f();
    private CalibrationParameters calibParams;
    private double rms = 0;
    private boolean isCalibrated;

    private Mat grayimage = new Mat();
    private Mat output = new Mat();

    private Telemetry telemetry;

    Bitmap image;

    public CalibrationPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(output);

        // enough samples, calibrate
        if (imagePoints.size() >= CALIB_FRAMES && !isCalibrated) {
            // generate chessboard corners and create array with copies of those corners
            MatOfPoint3f object = CalibUtil.generateChessboardPoints(HORIZONTAL_CORNERS, VERTICAL_CORNERS, SQUARE_SIZE);
            List<Mat> objectPoints = Collections.nCopies(imagePoints.size(), object);
            // initialize blank parameters for calibration method
            Mat cameraMat = new Mat();
            MatOfDouble distCoeffs = new MatOfDouble();
            List<Mat> rvecs = new ArrayList<>();
            List<Mat> tvecs = new ArrayList<>();
            MatOfPoint3f newObject = new MatOfPoint3f();

            // start timer to measure calibration time
            timer.reset();

            // perform calibration with ocv method, 4th param at -1 disables using ro method
            rms = Calib3d.calibrateCameraRO(objectPoints, imagePoints, input.size(),
                    USE_RO_METHOD ? HORIZONTAL_CORNERS - 1 : -1,
                    cameraMat, distCoeffs, rvecs, tvecs, newObject, CALIB_FLAGS);
            calibParams = CalibrationParameters.fromOCVParams(cameraMat, distCoeffs);

            System.out.printf("Calibration complete in %fms%nError (rms) = %f%nParameters = %s%n",
                    timer.milliseconds(), rms, calibParams.toString());

            isCalibrated = true;
        }

        // calibrated, show undistorted image
        if (isCalibrated) {
            telemetry.addData("Calibration Parameters", calibParams);
            telemetry.addData("Error (rms)", rms);
            telemetry.update();

            Calib3d.undistort(input, output, calibParams.getCameraMatrix(), calibParams.getDistCoeffs());
            return output;
        }

        Size boardSize = new Size(HORIZONTAL_CORNERS, VERTICAL_CORNERS);

        // convert to grayscale
        Imgproc.cvtColor(input, grayimage, Imgproc.COLOR_RGB2GRAY);

        // initial corner search
        boolean foundCorners = Calib3d.findChessboardCorners(grayimage, boardSize, imageCorners,
                Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_FAST_CHECK | Calib3d.CALIB_CB_NORMALIZE_IMAGE);

        if (foundCorners) {
            // subpixel corner optimization
            TermCriteria criteria = new TermCriteria(TermCriteria.MAX_ITER | TermCriteria.EPS, 30, 1e-3);
            Imgproc.cornerSubPix(grayimage, imageCorners, new Size(6, 6), new Size(4, 4), criteria);
            // show the chessboard inner corners on screen
            Calib3d.drawChessboardCorners(output, boardSize, imageCorners, foundCorners);

            if (IS_CAPTURING && timer.seconds() > FRAME_INTERVAL) {
                imagePoints.add(imageCorners.clone());
                timer.reset();
            }
        }

        // display status
        if (imagePoints.size() >= CALIB_FRAMES && !isCalibrated) {
            Imgproc.putText(output, "Calibrating...", new Point(output.width() * 0.1, output.height() * 0.6),
                    Imgproc.FONT_HERSHEY_COMPLEX, output.width() / 300.0, new Scalar(200, 30, 30), 3);
        } else {
            Imgproc.putText(output, String.format("Found: %d/%d", imagePoints.size(), CALIB_FRAMES),
                    new Point(output.width() * 0.5, output.height() * 0.96), Imgproc.FONT_HERSHEY_COMPLEX,
                    output.width() / 500.0, new Scalar(30, 200, 30), 2);
        }

        image = Bitmap.createBitmap(output.cols(), output.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(output, image);
        return output;
    }

    public Bitmap getImage(){
        if(image != null){
            return image;
        }

        return Bitmap.createBitmap(10, 10, Bitmap.Config.ARGB_8888);
    }
}
