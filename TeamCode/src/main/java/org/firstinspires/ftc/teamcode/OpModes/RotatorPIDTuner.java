package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.MPC.geometry.Pose2d;

@Autonomous
@Disabled
public class RotatorPIDTuner extends OpMode {
    Robot robot;
    FtcDashboard dashbaord;
    TelemetryPacket packet;

    @Override
    public void init() {
        dashbaord = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap, telemetry);
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        robot.updateBulkData();
        robot.updatePos();

        robot.shooter.setRotator(1, robot.getPos());

        telemetry.addData("Shooter Angle Radians", robot.shooter.getRotatorPos());
        telemetry.addData("Shooter Angle", Math.toDegrees(robot.shooter.getRotatorPos()));
        telemetry.addData("Current Position", robot.getPos());
        telemetry.addData("Error", Math.toDegrees(robot.shooter.getRotatorError2()));

        dashbaord.sendTelemetryPacket(packet);

        robot.shooter.write();
    }
}
