package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Vision.CalibrationPipeline;
import org.firstinspires.ftc.teamcode.Vision.RingLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
public class CameraCalibrator extends OpMode {
    ExtrinsicsFinder pipeline;

    OpenCvCamera webcam;

    @Override
    public void init() {
        pipeline = new ExtrinsicsFinder(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //Tells EasyOpenCv to start streaming the webcam feed and process the information
                //The resolution is set here
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
        });

    }

    public void start(){
        //pipeline.startTimer();
    }

    @Override
    public void loop() {
        FtcDashboard.getInstance().sendImage(pipeline.getImage());
    }
}