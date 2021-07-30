package org.firstinspires.ftc.teamcode.Components;


import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;

import java.util.Arrays;

@Config
public class Mecanum_Drive{
    Caching_Motor[] motors = new Caching_Motor[4];

    Telemetry telemetry;

    PIDFController PID_X;
    PIDFController PID_Y;
    PIDFController PID_Z;

    PIDFController PID_X_MPC;
    PIDFController PID_Y_MPC;
    PIDFController PID_Z_MPC;

    public static double kp = 0.095;
    public static double ki = 0.0;
    public static double kd = 0.015;

    public static double kps = 0.3575;
    public static double kis = 0.0;
    public static double kds = 0.035;

    public static double kpr = 2.75;
    public static double kir = 0.0;
    public static double kdr = 0.15;

    public double kpMPC = 0.065;
    public double kiMPC = 0;
    public double kdMPC = 0.005;

    public double kprMPC = 3.5;
    public double kirMPC = 0;
    public double kdrMPC = 0.0005;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    int counter;

    public Mecanum_Drive(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        motors[0] = new Caching_Motor(map, "front_left");
        motors[1] = new Caching_Motor(map, "front_right");
        motors[2] = new Caching_Motor(map, "back_left");
        motors[3] = new Caching_Motor(map, "back_right");

        motors[0].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PID_X = new PIDFController(new PIDCoefficients(kps, kis, kds));
        PID_Y = new PIDFController(new PIDCoefficients(kp, ki, kd));
        PID_Z = new PIDFController(new PIDCoefficients(kpr, kir, kdr));

        PID_X_MPC = new PIDFController(new PIDCoefficients(kpMPC, kiMPC, kdMPC));
        PID_Y_MPC = new PIDFController(new PIDCoefficients(kpMPC, kiMPC, kdMPC));
        PID_Z_MPC = new PIDFController(new PIDCoefficients(kprMPC, kirMPC, kdrMPC));
        counter = 0;

        motors[1].motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].motor.setDirection(DcMotorSimple.Direction.REVERSE );
    }

    public void resetPID(){ }

    public void write(){
        motors[counter].write();
        counter = (counter + 1) % 4;
    }

    public void setPower(double x, double y, double rot){
        //Calculating the 4 motor powers
        double frontLeftMotorPower = y - x - rot;
        double frontRightMotorPower = y + x + rot;
        double backLeftMotorPower = y + x - rot;
        double backRightMotorPower = y - x + rot;

        //Creating an array of all motor powers
        double motorPowers[] = {Math.abs(frontLeftMotorPower),
                Math.abs(backRightMotorPower),
                Math.abs(backLeftMotorPower),
                Math.abs(frontRightMotorPower)};
        //Sorting the motor powers to find the highest one
        Arrays.sort(motorPowers);

        //Verifying highest motor power != 0 to avoid a divide by 0 error
        if(Math.abs(motorPowers[3]) > 1){
            //Normalizing the values to be within 0-1
            frontLeftMotorPower /= Math.abs(motorPowers[3]);
            frontRightMotorPower /= Math.abs(motorPowers[3]);
            backRightMotorPower /= Math.abs(motorPowers[3]);
            backLeftMotorPower /= Math.abs(motorPowers[3]);
        }

        //Setting the powers
        motors[0].setPower(frontLeftMotorPower);
        motors[1].setPower(frontRightMotorPower);
        motors[2].setPower(backLeftMotorPower);
        motors[3].setPower(backRightMotorPower);
    }

    public void setPower(double UpLeft, double BackLeft, double UpRight, double BackRight){
        motors[0].setPower(-UpLeft);
        motors[1].setPower(-UpRight);
        motors[2].setPower(-BackLeft);
        motors[3].setPower(-BackRight);
    }

    public void setPower(Vector2 vec, double rot){
        setPower(vec.x, vec.y, rot);
    }

    public void setPowerCentic(double x, double y, double rot, double heading){
        setPower(new Vector2(x, y).rotated(-heading), rot);
    }

    public void drive(Gamepad gamepad, double maxMove, double maxTurn){
        setPower(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(gamepad.left_stick_y, -maxMove, maxMove), Range.clip(gamepad.right_stick_x, -maxTurn, maxTurn));
    }

    public void driveCentric(Gamepad gamepad, double maxMove, double maxTurn, double heading){
        setPowerCentic(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(gamepad.left_stick_y, -maxMove, maxMove), Range.clip(gamepad.right_stick_x, -maxTurn, maxTurn), heading);
    }

    public void goToPoint(Pose2d targetPos, Pose2d currentPos, double xspeed, double yspeed, double zspeed){
        PID_X.setOutputBounds(-xspeed, xspeed);
        PID_Y.setOutputBounds(-yspeed, yspeed);
        PID_Z.setOutputBounds(-zspeed, zspeed);

        double heading = 0;
        double target_heading = targetPos.getHeading();

        if(currentPos.getHeading() <= Math.PI){
            heading = currentPos.getHeading();
        }else{
            heading = -((2 * Math.PI ) - currentPos.getHeading());
        }

        if(Math.abs(targetPos.getHeading() - heading) >= Math.toRadians(180.0)){
            target_heading = -((2 * Math.PI) - targetPos.getHeading());
        }

        telemetry.addData("Target Pos: ", targetPos);
        telemetry.addData("Current Pos: ", currentPos);
        telemetry.addData("Translational Error", targetPos.vec().distTo(currentPos.vec()));
        telemetry.addData("Rotational Error", Math.toDegrees(Math.abs(target_heading - heading)));

        PID_X.setTargetPosition(targetPos.getX());
        PID_Y.setTargetPosition(targetPos.getY());
        PID_Z.setTargetPosition(target_heading);

        setPowerCentic(Robot.isBlue() ? -PID_X.update(currentPos.getX()) : PID_X.update(currentPos.getX()), -PID_Y.update(currentPos.getY()), Robot.isBlue() ? -PID_Z.update(heading) : PID_Z.update(heading), (Robot.isBlue() ? -currentPos.getHeading() : currentPos.getHeading()) + (Robot.isBlue() ? (2 * Math.PI) : 0));
    }

    public void goToPointFF(Pose2d targetPos, Pose2d currentPos, double maxPow, double FLff, double BLff, double FRff, double BRff) {
        /*PID_X_MPC.setOutputBounds(-maxmovespeed, maxmovespeed);
        PID_Y_MPC.setOutputBounds(-maxmovespeed, maxmovespeed);
        PID_Z_MPC.setOutputBounds(-maxturnspeed, maxturnspeed);*/

        double heading = 0;
        double target_heading = targetPos.getHeading();

        if(currentPos.getHeading() <= Math.PI){
            heading = currentPos.getHeading();
        }else{
            heading = -((2 * Math.PI ) - currentPos.getHeading());
        }

        if(Math.abs(targetPos.getHeading() - heading) >= Math.toRadians(180.0)){
            target_heading = -((2 * Math.PI) - targetPos.getHeading());
        }

        telemetry.addData("Target Pos: ", targetPos);
        telemetry.addData("Current Pos: ", currentPos);
        telemetry.addData("Translational Error", targetPos.vec().distTo(currentPos.vec()));
        telemetry.addData("Rotational Error", Math.toDegrees(Math.abs(target_heading - heading)));

        packet.put("x error", Math.abs(currentPos.getX() - targetPos.getX()));
        packet.put("y error", Math.abs(currentPos.getY() - targetPos.getY()));
        packet.put("theta error", Math.abs(currentPos.getHeading() - targetPos.getHeading()));

        packet.put("x target", targetPos.getX());
        packet.put("y target", targetPos.getY());
        packet.put("theta target", targetPos.getHeading());
        packet.put("x current", currentPos.getX());
        packet.put("y current", currentPos.getY());
        packet.put("theta current", currentPos.getHeading());

        dashboard.sendTelemetryPacket(packet);

        PID_X_MPC.setTargetPosition(targetPos.getX());
        PID_Y_MPC.setTargetPosition(targetPos.getY());
        PID_Z_MPC.setTargetPosition(target_heading);

        double x = Range.clip(PID_X_MPC.update(currentPos.getX()), -1.0, 1.0);
        double y = Range.clip(PID_Y_MPC.update(currentPos.getY()), -1.0, 1.0);
        double z = Range.clip(PID_Z_MPC.update(heading), -1.0, 1.0);

        setPowerFeedforwardCentric(x, y, z, currentPos.getHeading(), maxPow, FLff, BLff, FRff, BRff);
    }

    public void setPowerFeedforwardCentric(double x, double y, double rot, double heading, double maxPow, double FLff, double BLff, double FRff, double BRff) {
        Vector2d powers = new Vector2d(x, y).rotated(heading);

        double FrontLeftVal = powers.getY() + powers.getX() + rot;
        double FrontRightVal = powers.getY() - powers.getX() - rot;
        double BackLeftVal = powers.getY() - powers.getX() + rot;
        double BackRightVal = powers.getY() + powers.getX() - rot;

        setPower(Range.clip(FrontLeftVal + FLff, -maxPow, maxPow), Range.clip(BackLeftVal + BLff, -maxPow, maxPow), Range.clip(FrontRightVal + FRff, -maxPow, maxPow), Range.clip(BackRightVal + BRff, -maxPow, maxPow));
    }
}
