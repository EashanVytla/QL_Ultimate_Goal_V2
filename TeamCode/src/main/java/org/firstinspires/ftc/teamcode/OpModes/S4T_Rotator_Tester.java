package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class S4T_Rotator_Tester extends OpMode {
    DcMotor rotator;

    @Override
    public void init() {
        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("Rotator Position", Math.toDegrees((rotator.getCurrentPosition()/1440.0) * (2 * Math.PI)));
    }
}
