package org.firstinspires.ftc.teamcode.Components;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.Vision.RingDetectionPipelineV2;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

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
    RingDetectionPipelineV2 detector;
    Pose2d startPos = new Pose2d(0, 0, 0);
    private static boolean continuousMode = false;

    public Robot(HardwareMap map, Telemetry telemetry){
        this.hardwareMap = map;
        this.telemetry = telemetry;

        continuousMode = false;

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
    }

    public void start(){
        shooter.flicker.start();
    }

    public void operate(GamepadEx gamepad1, GamepadEx gamepad2){
        updateBulkData();

        if(gamepad1.isPress(GamepadEx.Control.dpad_left))
            continuousMode = !continuousMode;

        drive.driveCentric(gamepad1.gamepad, 1.0, 1.0, getPos().getHeading() + Math.toRadians(90));

        shooter.operate(gamepad1, getData2());
        intake.operate(gamepad1);
        wobbleGoal.operate(gamepad1);

        telemetry.addData("Robot Position:", getPos());

        telemetry.addLine("Shooting in " + (isContinuous() ? "continuous mode" : "flicker mode") + "...");

        updatePos();

        drive.write();
        intake.write();
        shooter.write();
        wobbleGoal.write();
    }

    public static boolean isContinuous(){
        return continuousMode;
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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public RevBulkData getData(){
        return data;
    }

    public RevBulkData getData2(){
        return data2;
    }

    public int getRingStackCase(){
        return detector.getAnalysis();
    }

    public void updateBulkData(){
        data = hub1.getBulkInputData();
        data2 = hub2.getBulkInputData();
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
        return new Pose2d(localizer.getPose().getX() + startPos.getX(), localizer.getPose().getY() + startPos.getY(), localizer.getPose().getHeading() + startPos.getHeading());
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    public double angleWrap(double angle){
        return (angle + (2 * Math.PI)) % (2 * Math.PI);
    }

    public Bitmap getWebcamImage(){
        return detector.getImage();
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
