package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Vision.RingLocalizer;
import org.firstinspires.ftc.teamcode.Vision.RingLocalizerV2;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous
public class RingLocalizerTester extends OpMode {
    RingLocalizerV2 pipeline;
    Robot robot;
    GamepadEx gamepadEx;

    OpenCvCamera webcam;
    ArrayList<Pose2d> targets = new ArrayList<>();
    ArrayList<Pose2d> prevTarget = new ArrayList<>();
    boolean toggle = false;

    @Override
    public void init() {
        pipeline = new RingLocalizerV2(telemetry);
        robot = new Robot(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        robot.localizer.reset();

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

    @Override
    public void loop() {
        robot.updateBulkData();
        FtcDashboard.getInstance().sendImage(pipeline.getImage());
        if(gamepadEx.isPress(GamepadEx.Control.a)){
            toggle = true;
        }

        if(toggle){
            robot.intake.setPower(1.0);
            //robot.GoTo(target, new Pose2d(1.0, 1.0, 1.0));
        }else{
            try{
                targets = pipeline.getRingPositions(robot.getPos());
            }catch(Exception e){
                telemetry.addLine("No rings found...");
            }

            /*if(targets.size() == 0){
                targets = prevTarget;
            }*/

            robot.intake.setPower(0.0);
        }

        telemetry.addData("targets", toggle);

        for (int i = 0; i < targets.size(); i++){
            telemetry.addData("Ring Position: " + i, targets.get(i));
        }

        if(targets.size() == 0){
            telemetry.addLine("SIZE IS 0");
        }

        prevTarget = targets;

        //robot.intake.write();
        gamepadEx.loop();
    }
}