    package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.MPC.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Vision.RingDetectionPipelineV2;
import org.firstinspires.ftc.teamcode.Vision.RingLocalizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Ring_Intake_Tester extends OpMode {
    //Mecanum_Drive drive;
    RingLocalizer pipeline;

    OpenCvCamera webcam;

    private boolean searching = true;
    private boolean prevIsCenter = false;
    //private Robot robot;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        //drive = new Mecanum_Drive(hardwareMap, telemetry);
        pipeline = new RingLocalizer();
        //robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();

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
        time.startTime();
    }
    com.acmerobotics.roadrunner.geometry.Pose2d target;
    boolean isInRange = false;

    @Override
    public void loop() {
        //robot.updateBulkData();
        //robot.updatePos();

        /*telemetry.addData("Min Dist", pipeline.getMinDist());

        if(!searching){
            if(isInRange){
                drive.setPower(0, -0.2, 0);

                if(time.time() > 0.5){
                    searching = true;
                }
            }else{
                if(target.vec().distTo(robot.getPos().vec()) <= 2){
                    isInRange = true;
                }else{
                    drive.goToPoint(target, robot.getPos(), 0.2, 0.2, 1.0);
                    time.reset();
                }
            }
        }else{
            if(pipeline.isCenter() && !prevIsCenter){
                target = new com.acmerobotics.roadrunner.geometry.Pose2d(Math.sin(robot.getPos().getHeading() + Math.toRadians(5)) * pipeline.getMinDist(), Math.cos(robot.getPos().getHeading() + Math.toRadians(5)) * pipeline.getMinDist(), robot.getPos().getHeading() + Math.toRadians(5));
                searching = false;
            }
            drive.setPower(0, 0, -0.2);
        }

        telemetry.addData("isCenter? ", pipeline.isCenter());

        telemetry.update();*/

        //drive.write();
        FtcDashboard.getInstance().sendImage(pipeline.getImage());
        prevIsCenter = pipeline.isCenter();
    }
}