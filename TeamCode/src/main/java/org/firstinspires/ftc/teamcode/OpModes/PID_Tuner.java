package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
public class PID_Tuner extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();
    }

    @Override
    public void loop() {
        robot.updateBulkData();

        robot.GoTo(new Pose2d(0, 0, Math.PI/2), new Pose2d(1, 1, 1));

        telemetry.update();
    }
}
