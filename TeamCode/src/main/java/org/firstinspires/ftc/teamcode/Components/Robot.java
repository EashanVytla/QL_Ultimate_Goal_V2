package org.firstinspires.ftc.teamcode.Components;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.Vision.RingDetectionPipelineV2;
import org.firstinspires.ftc.teamcode.Vision.RingLocalizerV2;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

public class Robot {
    public Mecanum_Drive drive;
    public Shooter shooter;
    public Intake intake;
    public WobbleGoal wobbleGoal;

    public ExpansionHubEx hub1;
    public ExpansionHubEx hub2;

    public S4T_Localizer localizer;
    private S4T_Encoder encoderLY;
    private S4T_Encoder encoderLX;
    private S4T_Encoder encoderRY;
    private S4T_Encoder encoderRX;
    private RevBulkData data;
    private RevBulkData data2;
    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    OpenCvCamera webcam;
    OpenCvPipeline detector;
    private static Pose2d startPos = new Pose2d(0, 0, 0);
    public static Vector2d ULTIMATE_GOAL_POS;
    public static Vector2d ULTIMATE_GOAL2_POS;
    public static Vector2d BLUE_GOAL_POS;
    private static boolean blue = false;
    public static Vector2d POWER_SHOT_R;
    public static Vector2d POWER_SHOT_M;
    public static Vector2d POWER_SHOT_L;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    public Robot(HardwareMap map, Telemetry telemetry){
        blue = false;
        ULTIMATE_GOAL_POS = new Vector2d(7, 136);
        ULTIMATE_GOAL2_POS = new Vector2d(5,-69);
        POWER_SHOT_R = new Vector2d(5 - 17.20 - 1.57, Robot.ULTIMATE_GOAL_POS.getY());
        POWER_SHOT_M = new Vector2d(5 - 25 - 2.47, Robot.ULTIMATE_GOAL_POS.getY());
        POWER_SHOT_L = new Vector2d(5 - 32.25 - 2.47, Robot.ULTIMATE_GOAL_POS.getY());
        this.hardwareMap = map;
        this.telemetry = telemetry;

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        hub1 = map.get(ExpansionHubEx.class, "Expansion Hub 173");
        hub2 = map.get(ExpansionHubEx.class, "Expansion Hub 2");

        encoderLY = new S4T_Encoder(map, "back_left");
        encoderLX = new S4T_Encoder(map, "front_left");
        encoderRY = new S4T_Encoder(map, "front_right");
        encoderRX = new S4T_Encoder(map, "back_right");

        drive = new Mecanum_Drive(map, telemetry);
        shooter = new Shooter(map, telemetry);
        intake = new Intake(map, telemetry);
        wobbleGoal = new WobbleGoal(map, telemetry);

        updateBulkData();

        localizer = new S4T_Localizer(hardwareMap, telemetry);
        localizer.setPacket(packet);
    }

    public void blue(){
        blue = true;
        ULTIMATE_GOAL_POS = new Vector2d(3, 136);
    }

    public void red(){
        blue = false;
    }

    public static boolean isBlue(){
        return blue;
    }

    public static double wrapHeading(double heading){
        if(heading > Math.PI){
            return heading - (2 * Math.PI);
        }

        return heading;
    }

    public void resetShooterPID(){
        shooter.resetPID();
    }

    public void operate(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {
        updateBulkData();
        updatePos();

        drive.driveCentric(gamepad1ex.gamepad, 1.0, 1.0, getPos().getHeading() + (blue ? -Math.toRadians(90) : Math.toRadians(90)));

        shooter.operate(gamepad1ex, gamepad2ex, getPos(), packet);
        intake.operate(gamepad1ex, gamepad2ex);
        wobbleGoal.operate(gamepad1ex, gamepad2ex);

        telemetry.addData("Robot Position:", getPos());
        telemetry.addData("Ultimate Goal Position", ULTIMATE_GOAL_POS);
        telemetry.addData("Ultimate Goal 2 Position", ULTIMATE_GOAL2_POS);

        dashboard.sendTelemetryPacket(packet);
        drive.write();
        intake.write();
        shooter.write();
        wobbleGoal.write();
    }

    public double getVelocityXMetersPerSecond(){
        double LX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_left"));
        double RX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_right"));
        double avg = (LX_velo + RX_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_STRAFE)/39.37;
    }

    public double getVelocityYMetersPerSecond(){
        double LY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_left"));
        double RY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_right"));
        double avg = (LY_velo + RY_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_VERT)/39.37;
    }

    public double getAngularVelocityOmega(){
        double LY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_left"));
        double RY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_right"));
        return (RY_velo - LY_velo)/S4T_Localizer.TRACK_WIDTH1;
    }

    public double getVelocityX(){
        double LX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_left"));
        double RX_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_right"));
        double avg = (LX_velo + RX_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_STRAFE);
    }

    public double getVelocityY(){
        double LY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "back_left"));
        double RY_velo = data.getMotorVelocity(hardwareMap.get(DcMotor.class, "front_right"));
        double avg = (LY_velo + RY_velo)/2;
        return (avg/localizer.TICKS_TO_INCHES_VERT);
    }

    public void setStartPose(Pose2d startPos){
        this.startPos = startPos;
    }

    public void initializeWebcam(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new RingDetectionPipelineV2();
        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public void initializeRingLocalizer(){
        detector = new RingLocalizerV2(telemetry);
        webcam.setPipeline(detector);
        //webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        //Log.i("HERE", "HERE");
    }

    public RevBulkData getData(){
        return data;
    }

    public RevBulkData getData2(){
        return data2;
    }

    public int getRingStackCase(){
        return ((RingDetectionPipelineV2)detector).getAnalysis();
    }

    public void updateBulkData(){
        data = hub1.getBulkInputData();
        data2 = hub2.getBulkInputData();
        shooter.updateData(data2);
    }

    public void updatePos(){
        encoderLX.update(data);
        encoderLY.update(data);
        encoderRX.update(data);
        encoderRY.update(data);
        localizer.update(getRawLeft_X_Dist(), getRawLeft_Y_Dist(), getRawRight_X_Dist(), getRawRight_Y_Dist(), getVelocityX(), getVelocityY(), data);
    }

    public double getLeft_X_Dist(){
        return encoderLX.getDist();
    }

    public double getRight_X_Dist(){
        return encoderRX.getDist();
    }

    public double getLeft_Y_Dist(){
        return encoderLY.getDist();
    }

    public double getRight_Y_Dist(){
        return encoderRY.getDist();
    }

    public double getRawLeft_X_Dist(){
        return encoderLX.distance;
    }

    public double getRawRight_X_Dist(){
        return encoderRX.distance;
    }

    public double getRawLeft_Y_Dist(){
        return encoderLY.distance;
    }

    public double getRawRight_Y_Dist(){
        return encoderRY.distance;
    }

    public Pose2d getPos(){
        if(blue){
            return new Pose2d(-localizer.getPose().getX() - startPos.getX(), localizer.getPose().getY() + startPos.getY(), (2 * Math.PI) - (localizer.getPose().getHeading() + startPos.getHeading()));
        }else{
            return new Pose2d(localizer.getPose().getX() + startPos.getX(), localizer.getPose().getY() + startPos.getY(), localizer.getPose().getHeading() + startPos.getHeading());
        }
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    public double angleWrap(double angle){
        return (angle + (2 * Math.PI)) % (2 * Math.PI);
    }

    public ArrayList<Vector2d> getRingPositions(){
        return ((RingLocalizerV2)detector).getRingPositions(getPos());
    }

    public void stopWebcam(){
        webcam.stopStreaming();
    }

    public void GoTo(Pose2d pose, Pose2d speedLimits){
        updateGoTo(pose, speedLimits);
    }

    public void GoTo(double x, double y, double heading, double maxspeed_x, double maxspeed_y, double maxspeed_z){
        updateGoTo(new Pose2d(x, y, heading), new Pose2d(maxspeed_x, maxspeed_y, maxspeed_z));
    }

    public void setAngle(double heading){
        localizer.setHeading(heading);
    }

    private void updateGoTo(Pose2d pose, Pose2d speedLimits){
        drive.goToPoint(pose, getPos(), speedLimits.getX(), speedLimits.getY(), speedLimits.getHeading());
        telemetry.addData("Position: ", getPos());
        telemetry.addData("Target Position: ", pose);
        drive.write();
        updatePos();
    }
}
