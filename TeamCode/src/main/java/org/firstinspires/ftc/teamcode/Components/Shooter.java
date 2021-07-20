package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.ejml.dense.block.VectorOps_DDRB;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

import java.time.OffsetDateTime;

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
    public static final double FLAP_MAX = 0.93;

    private final double DEGREES_TO_TICKS = (130/.825);
    private double flapTesterPos = 0.75;

    private final PIDFController flywheelPIDController; // PID controller for the Flywheel
    private final SimpleMotorFeedforward feedForward; // Flywheel feed-forward controller

    private final Caching_Motor rotator; //Bucket rotation motor
    private final Ma3_Encoder rotatorEncoder; //Bucket rotation encoder
    private final PIDFController rotatorPIDController; // PID controller for the Rotator
    private final PIDFController rotatorPIDController2; // PID controller for the Rotator Larger Targets
    private final double ROTATOR_MIN;
    private final double ROTATOR_0; //Rotator 0
    private final double Ma3_Offset; //Encoder Offset in Radians
    private final double ROTATOR_MAX; //Rotator Max Angle

    //PID constants for the flywheel (static for dashboard live tuning purposes)
    public static double kpF = 0.01;
    public static double kiF = 0;
    public static double kdF = 0.0001;

    //PID constants for the Bucket Rotator (static for dashboard live tuning purposes)
    public static double kpR = 4.75;
    public static double kiR = 4.75;
    public static double kdR = 0.006;

    //PID constants for Larger Distances on the Bucket Rotator (static for dashboard live tuning purposes)
    public static double kpR_2 = 3.5;
    public static double kiR_2 = 0;
    public static double kdR_2 = 0.05;

    //Feed-forward constants for the flywheel (static for dashboard live tuning purposes)
    public static double kS = 0.0;
    public static double kV = 1.075;

    public double stopperIn = 0.5;
    public double stopperOpen = 0.5;
    private boolean stopperToggle = false;

    private int powerShotToggle = 0;

    public Shooter(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;

        flywheelMotor = new Caching_Motor(map, "shooter");
        flicker = new Flicker(map, telemetry);
        flap = new Caching_Servo(map, "flap");
        rotator = new Caching_Motor(map, "rotator", 1e-6);
        stopper = new Caching_Servo(map, "stopper");
        rotatorEncoder = new Ma3_Encoder(map, "rotatorEncoder");
        stopper.setPosition(stopperIn);

        flap.setPosition(0.2);
        write();

        ROTATOR_MIN = Math.toRadians(120);
        ROTATOR_MAX = Math.toRadians(173);
        ROTATOR_0 = Math.toRadians(145);
        Ma3_Offset = Math.toRadians(196.6);

        flywheelPIDController = new PIDFController(new PIDCoefficients(kpF, kiF, kdF));
        feedForward = new SimpleMotorFeedforward(kS, kV);

        rotatorPIDController = new PIDFController(new PIDCoefficients(kpR, kiR, kdR));
        rotatorPIDController2 = new PIDFController(new PIDCoefficients(kpR_2, kiR_2, kdR_2));

        flywheelMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void start(){
        flicker.start();
    }

    // Writing to all the motors and servos
    public void write(){
        flicker.write();
        flywheelMotor.write();
        rotator.write();
        flap.write();
        stopper.write();
    }

    public double getFlapPos(double distance){
        double newDist = Range.clip(distance, 75, 120);

        double a = 9.9683e-9 * Math.pow(newDist, 6);
        a += -0.00000374408 * Math.pow(newDist, 5);
        a += 0.000531852 * Math.pow(newDist, 4);
        a += -0.0340679 * Math.pow(newDist, 3);
        a += 0.844729 * Math.pow(newDist, 2);

        a /= 100;

        /*double[] arr = new double[] {0, 1.14185, -0.223794, -0.0523778,
                0.0147242, -0.000758333};

        double a = 0;

        for(int i = 0; i < arr.length; i++){
            a += arr[i] * Math.pow(newDist, i + 1);
        }*/

        return Range.clip(a - 0.017, FLAP_MIN, FLAP_MAX);
    }

    public double getFlapPosPowerShot(double distance){
        double newDist = Range.clip(distance, 84, 103);
        double a = 9.9683e-9 * Math.pow(newDist, 6);
        a += -0.00000374408 * Math.pow(newDist, 5);
        a += 0.000531852 * Math.pow(newDist, 4);
        a += -0.0340679 * Math.pow(newDist, 3);
        a += 0.844729 * Math.pow(newDist, 2);

        a /= 100;

        a -= 0.03;

        /*double[] arr = new double[] {-3.98641555e-15,  6.16910146e-07,  3.83054854e-05, -8.13602663e-07,
                4.57405101e-09};

        double a = 0.8481613278756606;

        for(int i = 0; i < arr.length; i++){
            a += arr[i] * Math.pow(newDist, i + 1);
        }*/

        telemetry.addData("Power Shot Regressions Position", a);

        return Range.clip(a, FLAP_MIN, FLAP_MAX);
    }

    public void resetPID(){
        flywheelPIDController.reset();
    }

    public void setRotator(Pose2d currentPos) {
        double targetangle = (Robot.isBlue() ? -1 : 1) * Math.atan2((Robot.ULTIMATE_GOAL_POS.getX() - currentPos.getX()), (Robot.ULTIMATE_GOAL_POS.getY() - currentPos.getY()));
        double heading = currentPos.getHeading();

        if(heading <  2 * Math.PI && heading >= Math.PI){
            heading -= 2 * Math.PI;
        }

        double offset = targetangle - heading;

        double target = Range.clip(-offset + ROTATOR_0, ROTATOR_MIN, ROTATOR_MAX);

        telemetry.addData("OFFSET", Math.abs(target - getRotatorPos()));

        if(Math.abs(target - getRotatorPos()) > Math.toRadians(3)){
            telemetry.addLine("Im in the big PID");
            setRotator(-offset, true);
        }else{
            telemetry.addLine("Im in the small PID");
            setRotator(-offset, false);
        }

        telemetry.addData("OFFSET NEW", Math.toDegrees(-offset));
        telemetry.addData("ULTIMATE GOAL POS NEW", Robot.ULTIMATE_GOAL_POS);
        telemetry.addData("Target Position NEW", Math.toDegrees(targetangle));
        telemetry.addData("Current Pos NEW", currentPos);
    }

    public void setRotator(int powerShot, Pose2d currentPos) {
        double targetangle;

        switch(powerShot){
            case 1:
                targetangle = (Robot.isBlue() ? -1 : 1) * Math.atan2((Robot.POWER_SHOT_R.getX() - currentPos.getX()), (Robot.POWER_SHOT_R.getY() - currentPos.getY()));
                break;
            case 2:
                targetangle = (Robot.isBlue() ? -1 : 1) * Math.atan2((Robot.POWER_SHOT_M.getX() - currentPos.getX()), (Robot.POWER_SHOT_M.getY() - currentPos.getY()));
                break;
            case 3:
                targetangle = (Robot.isBlue() ? -1 : 1) * Math.atan2((Robot.POWER_SHOT_L.getX() - currentPos.getX()), (Robot.POWER_SHOT_L.getY() - currentPos.getY()));
                break;
            default:
                 targetangle = (Robot.isBlue() ? -1 : 1) * Math.atan2((Robot.ULTIMATE_GOAL_POS.getX() - currentPos.getX()), (Robot.ULTIMATE_GOAL_POS.getY() - currentPos.getY()));
                 break;
        }

        telemetry.addData("Target Position NEW", Math.toDegrees(targetangle));
        telemetry.addData("Current Pos NEW", currentPos);

        double heading = currentPos.getHeading();

        if(heading <  2 * Math.PI && heading >= Math.PI){
            heading -= 2 * Math.PI;
        }

        double offset = targetangle - heading;

        setRotator(-offset, true);
    }
    
    private void setRotatorPower(double power){
        rotator.setPower(power);
    }

    public void setFlap(double pos){
        flap.setPosition(pos, 1e-6);
    }

    //Target is 0-(2*PI) and 0-(-2*PI)
    //Negative values represent counter clockwise and positive values represent clockwise
    public void setRotator(double target, boolean largeTarget) {
        if(largeTarget){
            if(target > Math.PI){
                target -= 2 * Math.PI;
            }

            target = Range.clip(target + ROTATOR_0, ROTATOR_MIN, ROTATOR_MAX);
            rotatorPIDController2.setTargetPosition(target);

            telemetry.addData("Target", target);

            setRotatorPower(-rotatorPIDController2.update(getRotatorPos()));
            telemetry.addData("Error", Math.toDegrees(rotatorPIDController2.getLastError()));

            /*if(Math.abs(rotatorPIDController2.getLastError()) < Math.toRadians(0.25)){
                telemetry.addLine("IN RANGE!");
                rotatorPIDController.reset();
            }else{
                telemetry.addLine("REACHING TARGET...");
            }*/
        }else{
            if(target > Math.PI){
                target -= 2 * Math.PI;
            }

            telemetry.addData("Target", target + ROTATOR_0);

            target = Range.clip(target + ROTATOR_0, ROTATOR_MIN, ROTATOR_MAX);
            rotatorPIDController.setTargetPosition(target);

            double power = -rotatorPIDController.update(getRotatorPos());

            telemetry.addData("Power", power);

            setRotatorPower(power);
            telemetry.addData("Error", Math.toDegrees(rotatorPIDController.getLastError()));

            if(Math.abs(rotatorPIDController.getLastError()) < Math.toRadians(0.25)){
                telemetry.addLine("IN RANGE!");
                rotatorPIDController.reset();
            }else{
                telemetry.addLine("REACHING TARGET...");
            }
        }
    }

    public void setRotator(double target, TelemetryPacket packet) {
        setRotator(target, false);
        packet.put("Error", Math.toDegrees(rotatorPIDController.getLastError()));
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
        double val = rotatorEncoder.getRadians(ROTATOR_0 - Ma3_Offset);
        return val;
    }

    public double getRotatorPos(RevBulkData data){
        return rotatorEncoder.getRadians(ROTATOR_0 - Ma3_Offset, data);
    }

    public void operate(GamepadEx gamepad1Ex, GamepadEx gamepad2Ex, Pose2d currentPos, RevBulkData eHubdata, RevBulkData cHubData, TelemetryPacket packet){
        double flywheelVelo = getFlywheelVelcoity(eHubdata);

        telemetry.addData("Flywheel Velocity", flywheelVelo);
        packet.put("Flywheel Velocity", flywheelVelo);

        telemetry.addData("Dist to Ultimate Goal", currentPos.vec().distTo(Robot.ULTIMATE_GOAL_POS));

        if(gamepad1Ex.isPress(GamepadEx.Control.a)){
            powerShotToggle++;
            powerShotToggle %= 4;
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.b)){
            stopperToggle = !stopperToggle;
            if(stopperToggle){
                stopper.setPosition(stopperOpen);
            }else{
                stopper.setPosition(stopperIn);
            }
        }

        if(gamepad1Ex.isPress(GamepadEx.Control.right_trigger) || gamepad2Ex.isPress(GamepadEx.Control.a)) {
            flywheelToggle = !flywheelToggle;
        }

        if (flywheelToggle) {
            if(powerShotToggle == 0) {
                setFlywheelVelocity(flywheelTargetVelo, flywheelVelo);
            }else{
                setFlywheelVelocity(1750, flywheelVelo);
            }

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

        if(powerShotToggle == 0){
            setFlap(getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())));
            telemetry.addData("Dist to Ultimate Goal", currentPos.vec().distTo(Robot.ULTIMATE_GOAL_POS));
        }else if(powerShotToggle == 1){
            setFlap(getFlapPosPowerShot(Robot.POWER_SHOT_R.distTo(currentPos.vec())));
            telemetry.addData("Dist to Right Power Shot", currentPos.vec().distTo(Robot.POWER_SHOT_R));
        }else if(powerShotToggle == 2){
            setFlap(getFlapPosPowerShot(Robot.POWER_SHOT_M.distTo(currentPos.vec())));
            telemetry.addData("Dist to Middle Power Shot", currentPos.vec().distTo(Robot.POWER_SHOT_M));
        }else if(powerShotToggle == 3){
            setFlap(getFlapPosPowerShot(Robot.POWER_SHOT_L.distTo(currentPos.vec())));
            telemetry.addData("Dist to Left Power Shot", currentPos.vec().distTo(Robot.POWER_SHOT_L));
        }

        if(powerShotToggle == 0){
            setRotator(currentPos);
        }else{
            setRotator(powerShotToggle, currentPos);
        }

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
        telemetry.addData("Powershot Toggle", powerShotToggle);
        telemetry.addData("Flap Position", flapTesterPos);
    }
}