package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.MPC.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Autonomous
public class RotatorPIDTuner extends OpMode {
    Robot robot;
    FtcDashboard dashbaord;
    TelemetryPacket packet;
    GamepadEx gamepadEx;
    GamepadEx gamepad2Ex;
    int powerShotToggle = 1;

    @Override
    public void init() {
        dashbaord = FtcDashboard.getInstance();
        robot = new Robot(hardwareMap, telemetry);
        packet = new TelemetryPacket();
        gamepadEx = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        robot.updateBulkData();
        robot.updatePos();

        if(gamepadEx.isPress(GamepadEx.Control.a)){
            powerShotToggle++;
            powerShotToggle %= 4;
        }

        robot.shooter.setRotator(powerShotToggle, robot.getPos());

        telemetry.addData("Powershot Toggle", powerShotToggle);
        telemetry.addData("Shooter Angle Radians", robot.shooter.getRotatorPos());
        telemetry.addData("Shooter Angle", Math.toDegrees(robot.shooter.getRotatorPos()));
        telemetry.addData("Current Position", robot.getPos());

        gamepadEx.loop();
        robot.shooter.operate(gamepadEx, gamepad2Ex, robot.getPos(), robot.getData2(), robot.getData(), packet);
        robot.drive.driveCentric(gamepad1, 1.0, 1.0, robot.getPos().getHeading());
        robot.shooter.write();
        robot.drive.write();
        dashbaord.sendTelemetryPacket(packet);
    }
}
