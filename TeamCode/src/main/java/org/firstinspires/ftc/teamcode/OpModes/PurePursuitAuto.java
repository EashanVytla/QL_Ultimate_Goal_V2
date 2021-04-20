package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.MPC.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUnits;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class PurePursuitAuto extends LinearOpMode {
    int state = 0;
    ElapsedTime elapsedTime;
    boolean gtp = false;
    Robot robot;

    @Override
    public void runOpMode() {
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        robot.wobbleGoal.servo_grab.setPosition(robot.wobbleGoal.clamp_pos);
        robot.wobbleGoal.servo_lift.setPosition(0.3475);
        robot.wobbleGoal.servo_lift.write();
        robot.wobbleGoal.servo_grab.write();

        waitForStart();

        elapsedTime.startTime();

        while(opModeIsActive()){
            robot.updateBulkData();
            ArrayList<CurvePoint> points = new ArrayList<>();

            double flywheelVelo = robot.shooter.getFlywheelVelcoity(robot.getData2());

            telemetry.addData("Flywheel Velocity", flywheelVelo);

            switch(state){
                case 0:
                    if(elapsedTime.time() >= 0.5){
                        robot.localizer.reset();
                        robot.wobbleGoal.autoLift();
                        elapsedTime.reset();
                        RobotMovement.resetIndex();
                        state++;
                    }

                    break;
                case 1:
                    points.add(new CurvePoint(new Pose2d(0, 0, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-12d, 27d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-18.5d, 56.5d, Math.toRadians(0)), 0.5d, 0.5d, 25));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 3.0){
                            robot.shooter.stopFlywheel();
                            elapsedTime.reset();
                            robot.shooter.reset();
                            state++;
                        }else{
                            robot.shooter.setFlap(0.405);
                            if(elapsedTime.time() > 2.0){
                                robot.shooter.setRotator(0.455);
                            }else if(elapsedTime.time() > 1.0){
                                robot.shooter.setRotator(0.4765);
                            }else{
                                robot.shooter.setRotator(0.50899);
                            }

                            robot.shooter.setFlywheelVelocity(1750, flywheelVelo);
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    break;
                case 2:
                    points.add(new CurvePoint(new Pose2d(-21d, 56d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(19, 109.5, Math.toRadians(325)), 1d, 1d, 25));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 1.0){
                            elapsedTime.reset();
                            state++;
                        }else{
                            if(elapsedTime.time() > 0.5){
                                robot.wobbleGoal.release();
                            }
                            robot.wobbleGoal.down();
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    break;
                case 3:
                    points.add(new CurvePoint(new Pose2d(19, 109.5, Math.toRadians(325)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-15, 54, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-16.5, 18.7, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(1.69, 18.2, Math.toRadians(0)), 1d, 1d, 25));

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 1.0){
                            elapsedTime.reset();
                            state++;
                        }else{
                            if(elapsedTime.time() > 0.5){
                                robot.wobbleGoal.autoLift();
                            }
                            robot.wobbleGoal.down();
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    break;
                case 4:
                    points.add(new CurvePoint(new Pose2d(1.69, 18.2, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(3.8, 20.5, Math.toRadians(0)), 1.0, 1d, 25));
                    if(robot.getPos().getY() < 56){
                        points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(0)), 1d, 1d, 25));
                    }else{
                        points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(325)), 1d, 1d, 25));
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 1.0){
                            elapsedTime.reset();
                            state++;
                        }else{
                            if(elapsedTime.time() > 0.5){
                                robot.wobbleGoal.release();
                            }
                            robot.wobbleGoal.down();
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    break;
                case 5:
                    gtp = true;
                    points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(325)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-6.6, 75.3, Math.toRadians(0)), 1d, 1d, 25));

                    robot.wobbleGoal.lift();
                    robot.wobbleGoal.clamp();

                    break;
            }

            if(points.size() != 0){
                if(!gtp){
                    RobotMovement.followCurve(points, robot, telemetry);
                }else{
                    robot.GoTo(points.get(points.size() - 1).toPose(),new Pose2d(1, 1, 1));
                }
            }else{
                robot.drive.setPower(0, 0, 0);
                robot.updatePos();
            }

            robot.wobbleGoal.write();
            robot.shooter.write();

            telemetry.addData("Size", points.size());

            telemetry.addData("Position", robot.getPos());
            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
