package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ExtrinsicsFinder extends OpenCvPipeline {
    public static int HORIZONTAL_CORNERS = 8;
    public static int VERTICAL_CORNERS = 6;
    public static double SQUARE_SIZE = 0.98425197;
    public static CalibrationParameters CALIB_PARAMS = new CalibrationParameters(496.222, 499.292,
            322.836, 176.195, 0.0597197, -0.0908114, 0.0153578, -0.00202418, 0.0395567);

    private MatOfPoint2f imageCorners = new MatOfPoint2f();

    private Mat rvec = new Mat();
    private Mat tvec = new Mat();

    private Mat grayimage = new Mat();
    private Mat output = new Mat();

    Bitmap image;
    private Telemetry telemetry;

    public ExtrinsicsFinder(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(output);

        Size boardSize = new Size(HORIZONTAL_CORNERS, VERTICAL_CORNERS);

        // convert to grayscale
        Imgproc.cvtColor(input, grayimage, Imgproc.COLOR_RGB2GRAY);

        // initial corner search
        boolean foundCorners = Calib3d.findChessboardCorners(grayimage, boardSize, imageCorners,
                Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_FAST_CHECK | Calib3d.CALIB_CB_NORMALIZE_IMAGE);

        if (foundCorners) {
            // subpixel corner optimization
            TermCriteria criteria = new TermCriteria(
                    TermCriteria.MAX_ITER | TermCriteria.EPS, 30, 1e-3);
            Imgproc.cornerSubPix(
                    grayimage, imageCorners, new Size(6, 6), new Size(4, 4), criteria);
            // show the chessboard inner corners on screen
            Calib3d.drawChessboardCorners(output, boardSize, imageCorners, foundCorners);

            MatOfPoint3f objectPoints =
                    CalibUtil.generateChessboardPoints(HORIZONTAL_CORNERS, VERTICAL_CORNERS, SQUARE_SIZE);

            // localize chessboard
            Mat cameraMatrix = CALIB_PARAMS.getCameraMatrix();
            MatOfDouble distCoeffs = CALIB_PARAMS.getDistCoeffs();

            Calib3d.solvePnPRansac(objectPoints, imageCorners, cameraMatrix, distCoeffs, rvec, tvec);

            // drawing 3d coordinate axes
            MatOfPoint3f axesPoints = new MatOfPoint3f(new Point3(), new Point3(SQUARE_SIZE * 3, 0, 0),
                    new Point3(0, SQUARE_SIZE * 3, 0), new Point3(0, 0, SQUARE_SIZE * 3));
            MatOfPoint2f imagePoints = new MatOfPoint2f();

            Calib3d.projectPoints(axesPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

            Point[] pointArray = imagePoints.toArray();

            Imgproc.line(output, pointArray[0], pointArray[1], new Scalar(255, 0, 0), 5);
            Imgproc.line(output, pointArray[0], pointArray[2], new Scalar(0, 255, 0), 5);
            Imgproc.line(output, pointArray[0], pointArray[3], new Scalar(0, 0, 255), 5);

            System.out.printf("rvec = %s%ntvec = %s%n", rvec.dump(), tvec.dump());

            telemetry.addData("rvec", rvec.dump());
            telemetry.addData("tvec", tvec.dump());
            telemetry.update();
        }

        image = Bitmap.createBitmap(output.cols(), output.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(output, image);

        return output;
    }

    public Bitmap getImage(){
        if(image != null){
            return image;
        }

        return Bitmap.createBitmap(640, 360, Bitmap.Config.ARGB_8888);
    }
}
