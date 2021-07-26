package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Flicker;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class CornerAuto extends LinearOpMode {
    int state = 0;
    ElapsedTime elapsedTime;
    boolean gtp = false;
    Robot robot;
    private Pose2d ZONE_1_2_POS = new Pose2d(11, 72, Math.toRadians(0));
    private Pose2d ZONE_2_2_POS = new Pose2d(-8.5, 95.5, Math.toRadians(0));
    private Pose2d ZONE_4_2_POS = new Pose2d(11, 119.5, Math.toRadians(325));

    private Pose2d PARK = new Pose2d(4.6, 75.3, Math.toRadians(0));

    private int ringCase = 0;

    @Override
    public void runOpMode() {
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        robot.intake.barUp();

        robot.intake.write();

        robot.setStartPose(new Pose2d(24, 0));

        //Blue Side
        /*robot.blue();
        robot.inverse();
         */

        robot.initializeWebcam();

        while(!isStarted() && !isStopRequested()){;
            ringCase = robot.getRingStackCase();
            ringCase = 1;
            //telemetry.addData("Gyro", Math.toDegrees(robot.localizer.gyro.getAngleCorrected()));
            telemetry.addData("Ring Case", ringCase);

            if(gamepad1.a){
                robot.blue();
                robot.wobbleGoal.init();
            }

            if(gamepad1.y){
                robot.red();
                robot.wobbleGoal.init();
            }

            if(Robot.isBlue()){
                telemetry.addData("BLUE SIDE AUTO...", "TEEHEE");
            }else{
                telemetry.addData("RED SIDE AUTO...", "TEEHEE");
            }

            telemetry.update();
            elapsedTime.reset();
        }

        //robot.stopWebcam();

        waitForStart();

        elapsedTime.startTime();

        while(opModeIsActive()){
            robot.updateBulkData();
            ArrayList<CurvePoint> points = new ArrayList<>();

            double flywheelVelo = robot.shooter.getFlywheelVelcoity(robot.getData2());

            telemetry.addData("Flywheel Velocity", flywheelVelo);

            switch(state){
                case 0:
                    if(elapsedTime.time() >= 0.1){
                        robot.shooter.stopper.setPosition(robot.shooter.stopperOpen);
                        robot.wobbleGoal.lift();
                        robot.intake.barDown();
                        elapsedTime.reset();
                        RobotMovement.resetIndex();
                        state++;
                    }

                    break;
                case 1:
                    robot.shooter.setRotator(robot.getPos());

                    points.add(new CurvePoint(new Pose2d(24, 0, 0), 1d, 1d, 25));

                    points.add(new CurvePoint(new Pose2d(-5, 0, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(0, 20.5, Math.toRadians(0)), 1.0, 1d, 25));

                    if(robot.getPos().getY() > 45){
                        robot.intake.setPower(0.0);
                        robot.shooter.flywheelMotor.setPower(0.0);

                        //WOBBLE GOAL #2
                        switch (ringCase){
                            case 0:
                                points.add(new CurvePoint(ZONE_1_2_POS, 1d, 1d, 25));
                                break;
                            case 1:
                                points.add(new CurvePoint(ZONE_2_2_POS, 1d, robot.getPos().vec().distTo(ZONE_2_2_POS.vec()) < 15 ? 0.2 : 1d, 25));
                                break;
                            case 4:
                                points.add(new CurvePoint(ZONE_4_2_POS, 1d, robot.getPos().vec().distTo(ZONE_4_2_POS.vec()) < 15 ? 0.2 : 1d, 25));
                                break;
                        }

                    }else if(robot.getPos().getY() > 15){
                        robot.shooter.flap.setPosition(robot.shooter.getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(robot.getPos().vec())));
                        robot.shooter.flicker.flick();
                        robot.shooter.setFlywheelVelocity(2000, flywheelVelo);
                        robot.intake.setPower(1.0);

                        //WOBBLE GOAL #2
                        switch (ringCase){
                            case 0:
                                points.add(new CurvePoint(new Pose2d(ZONE_1_2_POS.getX(), ZONE_1_2_POS.getY(), 0), 0.15d, 1d, 25));
                                break;
                            case 1:
                                points.add(new CurvePoint(new Pose2d(ZONE_2_2_POS.getX(), ZONE_2_2_POS.getY(), 0), 0.15d, 1d, 25));
                                break;
                            case 4:
                                points.add(new CurvePoint(new Pose2d(ZONE_4_2_POS.getX(), ZONE_4_2_POS.getY(), 0), 0.15d, 1d, 25));
                                break;
                        }
                    }else{
                        robot.shooter.flap.setPosition(robot.shooter.getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(robot.getPos().vec())));
                        //robot.shooter.startFlywheel();
                        robot.shooter.setFlywheelVelocity(2000, flywheelVelo);
                        robot.intake.setPower(1.0);

                        //WOBBLE GOAL #2
                        switch (ringCase){
                            case 0:
                                points.add(new CurvePoint(new Pose2d(ZONE_1_2_POS.getX(), ZONE_1_2_POS.getY(), 0), 1d, 1d, 25));
                                break;
                            case 1:
                                points.add(new CurvePoint(new Pose2d(ZONE_2_2_POS.getX(), ZONE_2_2_POS.getY(), 0), 1d, 1d, 25));
                                break;
                            case 4:
                                points.add(new CurvePoint(new Pose2d(ZONE_4_2_POS.getX(), ZONE_4_2_POS.getY(), 0), 1d, 1d, 25));
                                break;
                        }
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 1.5){
                            elapsedTime.reset();
                            robot.drive.resetPID();
                            state++;
                        }else{
                            if(elapsedTime.time() > 0.5){
                                robot.wobbleGoal.release();
                            }

                            if(elapsedTime.time() > 1.0){
                                robot.wobbleGoal.lift();
                            }else{
                                robot.wobbleGoal.down();
                            }
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    break;
                case 2:
                    robot.intake.barUp();
                    gtp = true;
                    points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(325)), 0.5d, 0.5d, 25));
                    points.add(new CurvePoint(new Pose2d(-6.6, 75.3, Math.toRadians(0)), 0.5d, 0.5d, 25));

                    if(PARK.vec().distTo(robot.getPos().vec()) < 2.5){
                        robot.wobbleGoal.down();
                        robot.wobbleGoal.clamp();
                    }

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
            robot.intake.write();

            telemetry.addData("Size", points.size());

            telemetry.addData("Position", robot.getPos());
            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
