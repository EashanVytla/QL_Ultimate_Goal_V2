package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    public Caching_Motor flywheelMotor; // Flywheel Motor
    public Flicker flicker; // Servo to flick the rings into the flywheel
    private Telemetry telemetry; // Telemetry instance to output data onto the phone
    private boolean flywheelToggle = false; // Toggle the flywheel on and off
    public Caching_Servo flap; // Flap servo to change the height of the ring
    public Caching_Servo stopper; // Servo that stops the ring from falling out of the bucket

    public static double flywheelTargetVelo = 2000; // Target velocity for the flywheel

    // The physical servo constraints for the flap
    public static final double FLAP_MIN = 0.725;
    public static final double FLAP_MAX = 0.78;

    private final double ROTATOR_MIN = 0.28; //Left
    private final double ROTATOR_MAX = 0.69; //Right
    public static double ROTATOR_0 = 0.458;
    private final double DEGREES_TO_TICKS = (130/.825);
    private double flapTesterPos = 0.75;

    private final PIDFController flywheelPIDController; // PID controller for the Flywheel
    private final SimpleMotorFeedforward feedForward; // Flywheel feed-forward controller

    private Caching_Servo rotator; //Bucket rotator servo

    //PID constants for the flywheel (static for dashboard live tuning purposes)
    public static double kpF = 0.01;
    public static double kiF = 0;
    public static double kdF = 0.0001;

    //Feed-forward constants for the flywheel (static for dashboard live tuning purposes)
    public static double kS = 0.0;
    public static double kV = 1.075;

    public double stopperIn = 0.92;
    public double stopperOpen = 0.35;
    private boolean stopperToggle = false;

    public Shooter(HardwareMap map, Telemetry telemetry){
        flywheelMotor = new Caching_Motor(map, "shooter");
        flicker = new Flicker(map, telemetry);
        flap = new Caching_Servo(map, "flap");
        rotator = new Caching_Servo(map, "rotator");
        stopper = new Caching_Servo(map, "stopper");
        stopper.setPosition(stopperIn);

        rotator.setPosition(ROTATOR_0);
        flap.setPosition(0.2);
        write();

        flywheelPIDController = new PIDFController(new PIDCoefficients(kpF, kiF, kdF));
        feedForward = new SimpleMotorFeedforward(kS, kV);

        flywheelMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = telemetry;
    }

    public void start(){
        flicker.start();
    }

    // Writing to all the motors and servos
    public void write(){
        rotator.write();
        flicker.write();
        flywheelMotor.write();
        rotator.write();
        flap.write();
        stopper.write();
    }

    public double getFlapPos(double distance){
        double newDist = Range.clip(distance, 75, 120);

        double[] arr = new double[]{-6.69173811e-04, -2.41404719e-02,  7.05637256e-04,
                -8.65084574e-06,  5.00081670e-08, -1.12346487e-10};

        double a = 13.949544261905274;

        for(int i = 0; i < arr.length; i++){
            a += arr[i] * Math.pow(newDist, i + 1);
        }

        telemetry.addData("Regressions Position", a);

        return Range.clip(a, FLAP_MIN, FLAP_MAX);
    }

    public void resetPID(){
        flywheelPIDController.reset();
    }

    public void setRotator(Pose2d currentPos) {
        double targetangle = Math.atan2((Robot.ULTIMATE_GOAL_POS.getX() - currentPos.getX()), (Robot.ULTIMATE_GOAL_POS.getY() - currentPos.getY()));
        double heading = currentPos.getHeading();

        if(heading <  2 * Math.PI && heading >= Math.PI){
            heading -= 2 * Math.PI;
        }

        double tick_offset = Math.toDegrees(targetangle - heading) / DEGREES_TO_TICKS;

        double pos = Range.clip(ROTATOR_0 + tick_offset, ROTATOR_MIN, ROTATOR_MAX);

        if(currentPos.getY() >= 84){
            rotator.setPosition(ROTATOR_0);
        }else{
            rotator.setPosition(pos);
        }
    }

    public void setFlap(double pos){
        flap.setPosition(pos, 1e-6);
    }

    public void setRotator(double targetTick) {
        rotator.setPosition(Range.clip(targetTick, ROTATOR_MIN, ROTATOR_MAX), 1e-6);
    }

    public void setFlywheelVelocity(double targetVelo, double currentVelo){
        flywheelPIDController.setTargetPosition(targetVelo);

        setFlywheelPower(flywheelPIDController.update(currentVelo) + (feedForward.calculate(targetVelo)/2800));
    }

    public void setFlywheelPower(double value){
        flywheelMotor.setPower(-value);
    }

    public double getFlywheelVelcoity(RevBulkData data){
        if(data != null){
            return -data.getMotorVelocity(flywheelMotor.motor);
        }else{
            return 0;
        }
    }

    public double getRotatorPos(){
        return rotator.getPosition();
    }

    public void operate(GamepadEx gamepad1Ex, GamepadEx gamepad2Ex, Pose2d currentPos, RevBulkData data, TelemetryPacket packet){
        double flywheelVelo = getFlywheelVelcoity(data);

        telemetry.addData("Flywheel Velocity", flywheelVelo);
        packet.put("Flywheel Velocity", flywheelVelo);
        telemetry.addData("Dist to Ultimate Goal", currentPos.vec().distTo(Robot.ULTIMATE_GOAL_POS));

        if(gamepad2Ex.isPress(GamepadEx.Control.b)){
            stopperToggle = !stopperToggle;
            if(stopperToggle){
                stopper.setPosition(stopperOpen);
            }else{
                stopper.setPosition(stopperIn);
            }
        }


        if(gamepad1Ex.isPress(GamepadEx.Control.left_stick_button) || gamepad2Ex.isPress(GamepadEx.Control.a)) {
            flywheelToggle = !flywheelToggle;
        }

        if (flywheelToggle) {
            setFlywheelVelocity(flywheelTargetVelo, flywheelVelo);

            if (gamepad2Ex.isPress(GamepadEx.Control.left_trigger)) {
                Intake.pause = true;
                flicker.reset();
                flicker.flick = true;
            }

            //Only occurs one to reset the flicker timer and position
            if(gamepad1Ex.isPress(GamepadEx.Control.left_trigger) || gamepad2Ex.isPress(GamepadEx.Control.left_trigger)){
                flicker.reset();
            }

            //If you hold the left trigger then it flicks continuously
            if(gamepad1Ex.gamepad.left_trigger > 0.5 || gamepad2Ex.gamepad.left_trigger > 0.5){
                flicker.flick();
            }else{
                flicker.setPos(Flicker.outPos);
            }
        }else{
            Intake.pause = false;
            flywheelMotor.setPower(0.0);
        }

        //setFlap(getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())));
        setRotator(currentPos);

        telemetry.addData("Flap Regression Value", getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())));

        //Flap Regression Tuning
        //_________________________________________________________
        if(gamepad2Ex.gamepad.dpad_up){
            flapTesterPos += 0.0001;
        }

        if(gamepad2Ex.gamepad.dpad_down){
            flapTesterPos -= 0.0001;
        }

        flap.setPosition(flapTesterPos);
        //----------------------------------------------------------
        telemetry.addData("Flap Position", flapTesterPos);
    }
}