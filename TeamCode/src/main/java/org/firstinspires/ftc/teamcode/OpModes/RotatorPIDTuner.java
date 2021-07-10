package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Components.Shooter;

@Autonomous
public class RotatorPIDTuner extends OpMode {
    Shooter shooter;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        shooter.setRotator(0);

        telemetry.addData("Shooter Angle Radians", shooter.getRotatorPos());
        telemetry.addData("Shooter Angle", Math.toDegrees(shooter.getRotatorPos()));

        //shooter.write();
    }
}
