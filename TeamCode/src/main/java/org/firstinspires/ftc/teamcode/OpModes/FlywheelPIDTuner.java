package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.Components.Shooter.*;

@Autonomous
public class FlywheelPIDTuner extends OpMode {
    Motor motor;
    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Override
    public void init() {
        motor = new Motor(hardwareMap, "shooter", Motor.GoBILDA.BARE);
        motor.setRunMode(Motor.RunMode.VelocityControl);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        motor.setVeloCoefficients(kpF, kiF, kdF);

        motor.set(1750.0/2800.0);

        telemetry.addData("Velocity", motor.getCorrectedVelocity());
        packet.put("Velocity", motor.getCorrectedVelocity());

        dashboard.sendTelemetryPacket(packet);
    }
}
