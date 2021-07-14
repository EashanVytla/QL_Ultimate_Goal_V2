/*package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.google.gson.internal.$Gson$Preconditions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class T265_Localization_Tester extends LinearOpMode
{
    private static T265Camera slamra;
    public double move_power = 1.0;
    public double turn_power = 1.0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private Robot robot;
    private GamepadEx gamepadEx;
    private final double odoCovariance = 0.01;

    Transform2d OFFSET = new Transform2d(new Translation2d(-8.6247211/39.37, 0), Rotation2d.fromDegrees(90));

    Translation2d translation = new Translation2d(0, 0);
    Rotation2d rotation = new Rotation2d(0, 0);

    private final int robotRadius = 9; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx = new GamepadEx(gamepad1);

        if(slamra == null){
            try{
                slamra = new T265Camera(OFFSET, odoCovariance, hardwareMap.appContext);
            }catch (Exception e){
                slamra = null;
                telemetry.addData("ERROR","Couldn't find the camera... Trying again...");
            }
        }

        telemetry.addData("Is started?", slamra.isStarted());

        if(slamra != null){
            //slamra.setPose(new Pose2d(0, 0, new Rotation2d(0)));

            if(!slamra.isStarted()){
                slamra.start();
            }
        }

        robot = new Robot(hardwareMap, telemetry);

        while(!isStarted() && !isStopRequested()){
            if(slamra == null){
                try{
                    slamra = new T265Camera(OFFSET, odoCovariance, hardwareMap.appContext);
                }catch (Exception e){
                    slamra = null;
                    telemetry.addData("ERROR","Couldn't find the camera... Trying again...");
                }
            }

            telemetry.addData("Is started?", slamra.isStarted());

            if(slamra != null){
                if(!slamra.isStarted()){
                    slamra.setPose(new Pose2d(0, 0, new Rotation2d(0)));
                    slamra.start();
                }
            }
        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            if(gamepad1.share){
                robot.localizer.reset();
            }

            if(slamra == null){
                telemetry.addData("IT IS NULL", "IT IS NULL");
                try{
                    slamra = new T265Camera(OFFSET, odoCovariance, hardwareMap.appContext);
                    slamra.start();
                }catch (Exception e){
                    slamra = null;
                    telemetry.addData("ERROR","Couldn't find the camera... Trying again...");
                }
            }else{
                robot.updateBulkData();
                gamepadEx.loop();

                slamra.sendOdometry(-robot.getVelocityXMetersPerSecond(), -robot.getVelocityYMetersPerSecond());
                packet.put("X velocity", robot.getVelocityXMetersPerSecond());
                packet.put("Y velocity", robot.getVelocityYMetersPerSecond());

                T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
                if (up == null) return;

                if(gamepad1.a){
                    slamra.setPose(up.pose);
                }

                translation = new Translation2d(up.pose.getTranslation().getX() * 39.37, up.pose.getTranslation().getY() * 39.37);
                rotation = up.pose.getRotation();
            }

            packet.put("Pose", "(" + Math.round(translation.getX() * 100)/100.0 + ", " + Math.round(translation.getY() * 100)/100.0 + ", " + Math.round(rotation.getDegrees() * 100)/100.0 + ")");
            telemetry.addData("Pose", "(" + Math.round(translation.getX() * 100)/100.0 + ", " + Math.round(translation.getY() * 100)/100.0 + ", " + Math.round(rotation.getDegrees() * 100)/100.0 + ")");

            packet.put("start", gamepadEx.isPress(GamepadEx.Control.a));
            field.strokeCircle(translation.getX() + robot.localizer.DASHBOARD_OFFSET_FROM_CENTER.getY(), translation.getY() + robot.localizer.DASHBOARD_OFFSET_FROM_CENTER.getX(), robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
            double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
            field.strokeLine(x1 + robot.localizer.DASHBOARD_OFFSET_FROM_CENTER.getY(), y1 + robot.localizer.DASHBOARD_OFFSET_FROM_CENTER.getX(), x2 + robot.localizer.DASHBOARD_OFFSET_FROM_CENTER.getY(), y2 + robot.localizer.DASHBOARD_OFFSET_FROM_CENTER.getX());

            packet.put("Odo Position", robot.getPos());
            dashboard.sendTelemetryPacket(packet);

            robot.drive.driveCentric(gamepad1, move_power, turn_power, robot.getPos().getHeading() + (Math.PI/2));
            robot.drive.write();

            robot.updatePos();
            telemetry.addData("Odo Position", robot.getPos());
        }
    }
}
*/