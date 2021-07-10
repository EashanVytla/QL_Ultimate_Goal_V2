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
    public static final double FLAP_MIN = 0.18;
    public static final double FLAP_MAX = 0.88;

    private final double DEGREES_TO_TICKS = (130/.825);
    private double flapTesterPos = 0.75;

    private final PIDFController flywheelPIDController; // PID controller for the Flywheel
    private final SimpleMotorFeedforward feedForward; // Flywheel feed-forward controller

    private final Caching_Motor rotator; //Bucket rotation motor
    private final Ma3_Encoder rotatorEncoder; //Bucket rotation encoder
    private final PIDFController rotatorPIDController; // PID controller for the Rotator
    private final double ROTATOR_MIN;
    private final double ROTATOR_0; //Rotator 0
    private final double Ma3_Offset; //Encoder Offset in Radians
    private final double ROTATOR_MAX; //Rotator Max Angle

    //PID constants for the flywheel (static for dashboard live tuning purposes)
    public static double kpF = 0.01;
    public static double kiF = 0;
    public static double kdF = 0.0001;

    //PID constants for the Bucket Rotator (static for dashboard live tuning purposes)
    public static double kpR = 6;
    public static double kiR = 0;
    public static double kdR = 0.1;

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
        rotator = new Caching_Motor(map, "rotator", 1e-6);
        stopper = new Caching_Servo(map, "stopper");
        rotatorEncoder = new Ma3_Encoder(map, "rotatorEncoder");
        stopper.setPosition(stopperIn);

        flap.setPosition(0.2);
        write();

        ROTATOR_MIN = Math.toRadians(125);
        ROTATOR_MAX = Math.toRadians(165);
        ROTATOR_0 = Math.toRadians(145);
        Ma3_Offset = 0.264 * (2 * Math.PI);

        flywheelPIDController = new PIDFController(new PIDCoefficients(kpF, kiF, kdF));
        feedForward = new SimpleMotorFeedforward(kS, kV);

        rotatorPIDController = new PIDFController(new PIDCoefficients(kpR, kiR, kdR));

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

        double[] arr = new double[]{2.52218538e-05,  9.56110472e-04, -2.80182843e-05,
                3.35131026e-07, -1.85107487e-09,  3.90995297e-12};

        double a = 0.3913533087607506;

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

        double offset = targetangle - heading;

        setRotator(-offset);
    }

    public void setRotator(Pose2d currentPos, RevBulkData data) {
        double targetangle = Math.atan2((Robot.ULTIMATE_GOAL_POS.getX() - currentPos.getX()), (Robot.ULTIMATE_GOAL_POS.getY() - currentPos.getY()));
        double heading = currentPos.getHeading();

        if(heading <  2 * Math.PI && heading >= Math.PI){
            heading -= 2 * Math.PI;
        }

        double offset = targetangle - heading;

        setRotator(-offset, data);
    }

    public void setFlap(double pos){
        flap.setPosition(pos, 1e-6);
    }

    //Target is 0-(2*PI) and 0-(-2*PI)
    //Negative values represent counter clockwise and positive values represent clockwise
    public void setRotator(double target) {
        if(target > Math.PI){
            target -= 2 * Math.PI;
        }

        target = Range.clip(target + ROTATOR_0, ROTATOR_MIN, ROTATOR_MAX);
        rotatorPIDController.setTargetPosition(target);

        telemetry.addData("target", Math.toDegrees(target));

        rotator.setPower(-rotatorPIDController.update(getRotatorPos()));
        telemetry.addData("Error", Math.toDegrees(rotatorPIDController.getLastError()));
    }

    public void setRotator(double target, RevBulkData data) {
        if(target > Math.PI){
            target -= 2 * Math.PI;
        }

        target = Range.clip(target + ROTATOR_0, ROTATOR_MIN, ROTATOR_MAX);
        rotatorPIDController.setTargetPosition(target);

        rotator.setPower(-rotatorPIDController.update(getRotatorPos(data)));
        telemetry.addData("Rotator POsition", getRotatorPos());
        telemetry.addData("Error", Math.toDegrees(rotatorPIDController.getLastError()));
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
        telemetry.addData("RAW", rotatorEncoder.getRawValue());
        return rotatorEncoder.getRadians(ROTATOR_0 - Ma3_Offset);
    }

    public double getRotatorPos(RevBulkData data){
        telemetry.addData("RAW", rotatorEncoder.getRawValue());
        return rotatorEncoder.getRadians(ROTATOR_0 - Ma3_Offset, data);
    }

    public void operate(GamepadEx gamepad1Ex, GamepadEx gamepad2Ex, Pose2d currentPos, RevBulkData eHubdata, RevBulkData cHubData, TelemetryPacket packet){
        double flywheelVelo = getFlywheelVelcoity(eHubdata);

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

            //Only occurs one to reset the flicker timer and position
            if(gamepad1Ex.isPress(GamepadEx.Control.left_trigger) || gamepad2Ex.isPress(GamepadEx.Control.left_trigger)){
                flicker.reset();
            }

            //If you hold the left trigger then it flicks continuously
            if(gamepad1Ex.gamepad.left_trigger > 0.5 || gamepad2Ex.gamepad.left_trigger > 0.5){
                flicker.flick();
            }else{
                flicker.setPos(Flicker.inPos);
            }
        }else{
            Intake.pause = false;
            flywheelMotor.setPower(0.0);
        }

        setFlap(getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())) - 0.001);
        setRotator(currentPos);

        telemetry.addData("Flap Regression Value", getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())));

        //Flap Regression Tuning
        //_________________________________________________________
        /*if(gamepad2Ex.gamepad.dpad_up){
            flapTesterPos += 0.0001;
        }

        if(gamepad2Ex.gamepad.dpad_down){
            flapTesterPos -= 0.0001;
        }

        setFlap(flapTesterPos);*/
        //----------------------------------------------------------
        telemetry.addData("Flap Position", flapTesterPos);
    }
}