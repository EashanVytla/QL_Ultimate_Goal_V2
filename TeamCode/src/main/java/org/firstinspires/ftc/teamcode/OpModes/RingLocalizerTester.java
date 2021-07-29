package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Vision.RingLocalizerV2;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous
public class RingLocalizerTester extends OpMode {
    RingLocalizerV2 pipeline;
    Robot robot;
    GamepadEx gamepadEx;

    OpenCvCamera webcam;
    ArrayList<Vector2d> targets = new ArrayList<>();
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
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    int counter = 0;
    ArrayList<CurvePoint> allPoints = new ArrayList<>();

    @Override
    public void loop() {
        robot.updateBulkData();
        robot.updatePos();
        //FtcDashboard.getInstance().sendImage(pipeline.getImage());
        if(gamepadEx.isPress(GamepadEx.Control.a)){
            toggle = true;
            webcam.stopStreaming();
            counter = 0;
            sortTargets();
            for(int i = 0; i < targets.size(); i++){
                allPoints.add(new CurvePoint(targets.get(i).getX(), targets.get(i).getY() - 13, 1.0, 1.0, 5.0, 0));
            }
        }

        if(toggle){
            robot.intake.setPower(1.0);

            /*Pose2d target = new Pose2d(targets.get(counter).getX(), targets.get(counter).getY() - 13);

            if(Math.abs(target.getX() - robot.getPos().getX()) > 4){
                robot.GoTo(target, new Pose2d(1, 0, 1));
            }else{
                robot.GoTo(target, new Pose2d(1, 1, 1));
            }

            if(target.vec().distTo(robot.getPos().vec()) < 6){
                if(counter != targets.size() - 1){
                    counter++;
                }
            }*/
            RobotMovement.followCurve(allPoints, robot, telemetry);
        }else{
            try{
                targets = pipeline.getRingPositions(robot.getPos());
            }catch(Exception e){
                telemetry.addLine("No rings found...");
            }

            robot.drive.drive(gamepad1, 1.0, 1.0);

            robot.intake.setPower(0.0);
            robot.drive.write();
        }

        telemetry.addData("targets", toggle);

        for (int i = 0; i < targets.size(); i++){
            telemetry.addData("Ring Position: " + i, targets.get(i));
        }

        if(targets.size() == 0){
            telemetry.addLine("SIZE IS 0");
        }

        robot.intake.write();
        gamepadEx.loop();
    }

    public void sortTargets(){
        for(int i = 0; i < targets.size(); i++){
            int min = i;
            for(int j = i + 1; j < targets.size(); j++){
                if(targets.get(min).getY() > targets.get(j).getY()){
                    min = j;
                }
                Collections.swap(targets, min, i);
            }
        }

        if(targets.size() > 3){
            Log.i("HELLO WORLD", "HELLO WORLD");
            targets.subList(3, targets.size()).clear();
        }
    }
}