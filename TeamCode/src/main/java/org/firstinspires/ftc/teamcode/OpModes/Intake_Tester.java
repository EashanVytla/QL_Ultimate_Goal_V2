package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (name = "Intake_Tester")
public class Intake_Tester extends LinearOpMode {
    final String name = "Intake";

    DcMotor motor;
    HardwareMap map;
    Telemetry telemetry;
    Intake_Tester intake;

    public Intake_Tester(HardwareMap map, Telemetry telemetry){
        this.map = map;
        this.telemetry = telemetry;
        motor = map.get(DcMotor.class, name);
    }
    @Override
    public void runOpMode(){
        intake = new Intake_Tester(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()){
            motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("Power", motor.getPower());
            telemetry.update();
        }
    }
}
