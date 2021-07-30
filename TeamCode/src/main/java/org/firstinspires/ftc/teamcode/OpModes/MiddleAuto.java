package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Flicker;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous
public class MiddleAuto extends LinearOpMode {
    int state = 0;
    ElapsedTime elapsedTime;
    boolean gtp = false;
    Robot robot;
    private boolean powershotBool = false;

    private Pose2d POWER_SHOT_POS = new Pose2d(-20d, 52d, Math.toRadians(0));
    private Pose2d ZONE_1_POS = new Pose2d(14, 79, Math.toRadians(0));
    private Pose2d ZONE_2_POS = new Pose2d(-0.5, 103.5, Math.toRadians(0));
    private Pose2d ZONE_4_POS = new Pose2d(15, 105, Math.toRadians(325));
    private Pose2d PARK = new Pose2d(-16.6, 75.3, Math.toRadians(0));

    private int ringCase = 0;
    ArrayList<Vector2d> bounceBackPoints = new ArrayList<>();
    private boolean ringsFound = false;

    @Override
    public void runOpMode() {
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        robot.intake.initAuto();

        robot.intake.write();

        robot.shooter.resetRotator();

        boolean bigPID = false;

        //Blue Side
        /*robot.blue();
        robot.inverse();
         */

        boolean firstBB = true;

        int powershot = 0;
        boolean readyForPowershot = false;

        robot.initializeWebcam();

        while(!isStarted() && !isStopRequested()){
            telemetry.addData("Rotator", robot.shooter.rotator.motor.getMode());
            ringCase = robot.getRingStackCase();

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

        robot.setStartPose(new Pose2d(Robot.isBlue() ? 8.5 : -13.5, 0));

        waitForStart();

        //robot.stopWebcam();

        elapsedTime.startTime();

        robot.shooter.resetRotator();

        while(opModeIsActive()){
            robot.updateBulkData();
            ArrayList<CurvePoint> points = new ArrayList<>();

            double flywheelVelo = robot.shooter.getFlywheelVelcoity(robot.getData2());

            telemetry.addData("Flywheel Velocity", flywheelVelo);
            telemetry.addData("BIG PID", bigPID);
            telemetry.addData("Elapsed Time", elapsedTime.time());

            switch(state){
                case 0:
                    if(elapsedTime.time() >= 0.1){
                        robot.wobbleGoal.lift();
                        elapsedTime.reset();
                        RobotMovement.resetIndex();
                        state++;
                    }

                    robot.intake.release();

                    break;
                case 1:
                    points.add(new CurvePoint(new Pose2d(0, 0, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-16d, 27d, Math.toRadians(0)), 0.75d, 1d, 25));
                    points.add(new CurvePoint(POWER_SHOT_POS, 0.5d, 1.0d, 25));
                    telemetry.addData("Rotator Error 2", Math.toDegrees(robot.shooter.getRotatorError2()));
                    telemetry.addData("Rotator Error", Math.toDegrees(robot.shooter.getRotatorError()));

                    robot.shooter.setFlywheelVelocity(1460, flywheelVelo);

                    if(powershotBool){
                        switch(powershot){
                            case 0:
                                //FIRST POWERSHOT
                                robot.shooter.setRotator(1, robot.getPos(), false);
                                robot.shooter.setFlap(robot.shooter.getFlapPosPowerShot(Robot.POWER_SHOT_R.distTo(robot.getPos().vec())));

                                if(readyForPowershot){
                                    robot.shooter.stopper.setPosition(robot.shooter.stopperOpen);

                                    if(elapsedTime.time() > 0.5){
                                        robot.shooter.flicker.setPos(Flicker.inPos);
                                    }else if(elapsedTime.time() > 0.25){
                                        robot.shooter.flicker.setPos(Flicker.outPos);
                                    }

                                    if(elapsedTime.time() > 1.0){
                                        readyForPowershot = false;
                                        firstBB = true;
                                        robot.shooter.resetRotatorPID();
                                        elapsedTime.reset();
                                        powershot++;
                                    }
                                }else{
                                    if(robot.shooter.getFlywheelVelcoity(robot.getData2()) >= 1440 && Math.abs(robot.shooter.getRotatorError()) < Math.toRadians(0.75)){
                                        if(elapsedTime.time() > 0.5){
                                            readyForPowershot = true;
                                            elapsedTime.reset();
                                        }
                                    }else{
                                        elapsedTime.reset();
                                    }
                                }
                                break;
                            case 1:
                                //SECOND POWERSHOT
                                robot.shooter.setRotator(2, robot.getPos());
                                robot.shooter.setFlap(robot.shooter.getFlapPosPowerShot(Robot.POWER_SHOT_M.distTo(robot.getPos().vec())));

                                if(readyForPowershot){
                                    if(elapsedTime.time() > 0.25){
                                        robot.shooter.flicker.setPos(Flicker.inPos);
                                    }else{
                                        robot.shooter.flicker.setPos(Flicker.outPos);
                                    }

                                    if(elapsedTime.time() > 1.0){
                                        readyForPowershot = false;
                                        robot.shooter.resetRotatorPID();
                                        elapsedTime.reset();
                                        powershot++;
                                    }
                                }else{
                                    if(robot.shooter.getFlywheelVelcoity(robot.getData2()) >= 1440 && Math.abs(robot.shooter.getRotatorError2()) < Math.toRadians(0.75)){
                                        if(elapsedTime.time() > 0.5){
                                            readyForPowershot = true;
                                            elapsedTime.reset();
                                        }
                                    }else{
                                        elapsedTime.reset();
                                    }
                                }
                                break;
                            case 2:
                                //THIRD POWERSHOT
                                robot.shooter.setRotator(3, robot.getPos());
                                robot.shooter.setFlap(robot.shooter.getFlapPosPowerShot(Robot.POWER_SHOT_L.distTo(robot.getPos().vec())));

                                if(readyForPowershot){
                                    if(elapsedTime.time() > 0.25){
                                        robot.shooter.flicker.setPos(Flicker.inPos);
                                    }else{
                                        robot.shooter.flicker.setPos(Flicker.outPos);
                                    }

                                    if(elapsedTime.time() > 1.0){
                                        readyForPowershot = false;
                                        firstBB = true;
                                        robot.shooter.setFlywheelPower(0.0);
                                        elapsedTime.reset();
                                        robot.shooter.resetRotatorPID();
                                        state++;
                                    }

                                }else{
                                    robot.shooter.flicker.setPos(Flicker.inPos);
                                    if(robot.shooter.getFlywheelVelcoity(robot.getData2()) >= 1440 && Math.abs(robot.shooter.getRotatorError2()) < Math.toRadians(0.75)){
                                        if(elapsedTime.time() > 1.0){
                                            readyForPowershot = true;
                                            elapsedTime.reset();
                                        }
                                    }else{
                                        elapsedTime.reset();
                                    }
                                }
                                break;
                        }
                    }else{
                        elapsedTime.reset();
                        if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && Math.abs(Robot.wrapHeading(robot.getPos().getHeading())) < Math.toRadians(1.0)){
                            powershotBool = true;
                            robot.initializeRingLocalizer();
                            robot.shooter.resetRotatorPID();
                        }

                        robot.shooter.setRotator(1, robot.getPos());
                    }

                    telemetry.addData("POWERSHOT", powershot);
                    telemetry.addData("CHECK THIS", readyForPowershot);
                    break;
                case 2:
                    if (elapsedTime.time() > 3.5){
                        if(elapsedTime.time() > 8){
                            elapsedTime.reset();
                            state++;
                            ringsFound = true;
                        }

                        if(firstBB){
                            robot.stopWebcam();
                            sortTargets();
                            firstBB = false;
                        }
                        points.add(new CurvePoint(POWER_SHOT_POS.getX(), POWER_SHOT_POS.getY(), 1.0, 1.0, 5.0, 0));
                        for(int i = 0; i < bounceBackPoints.size(); i++){
                            points.add(new CurvePoint(bounceBackPoints.get(i).getX(), bounceBackPoints.get(i).getY() - 10, 1.0, 1.0, 5.0, 0));
                        }

                        robot.intake.setPower(1.0);

                        if(points.size() > 1){
                            if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                                elapsedTime.reset();
                                state++;
                                ringsFound = true;
                            }
                        }else{
                            state++;
                            ringsFound = false;
                            elapsedTime.reset();
                        }
                    }else{
                        bounceBackPoints = robot.getRingPositions();
                        telemetry.addData("BounceBack Size", String.valueOf(bounceBackPoints.size()));
                    }
                    break;
                case 3:
                    robot.shooter.setFlywheelPower(0.0);
                    points.add(new CurvePoint(POWER_SHOT_POS.getX() - 5, POWER_SHOT_POS.getY() + 50, 1.0, 1.0, 25.0, 0));

                    switch (ringCase){
                        case 0:
                            points.add(new CurvePoint(ZONE_1_POS, 1d, 1d, 25));
                            break;
                        case 1:
                            points.add(new CurvePoint(ZONE_2_POS, 1d, 1d, 25));
                            break;
                        case 4:
                            points.add(new CurvePoint(ZONE_4_POS, 1d, 1d, 25));
                            break;
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2){
                        robot.intake.setPower(0.0);
                        if(elapsedTime.time() > 1.5){
                            elapsedTime.reset();
                            if(ringsFound){
                                state++;
                            }else{
                                state += 2;
                            }
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
                case 4:
                    points.add(new CurvePoint(POWER_SHOT_POS.getX() - 5, POWER_SHOT_POS.getY() + 50, 1.0, 1.0, 25.0, 0));
                    points.add(new CurvePoint(POWER_SHOT_POS.getX(), POWER_SHOT_POS.getY(), 1.0, 1.0, 25.0, 0));
                    robot.shooter.setFlap(robot.shooter.getFlapPos(robot.getPos().vec().distTo(Robot.ULTIMATE_GOAL_POS)));

                    if(robot.getPos().vec().distTo(POWER_SHOT_POS.vec()) < 1.0){
                        if(elapsedTime.time() > 3.0){
                            robot.shooter.setFlywheelPower(0.0);
                            state++;
                        }else{
                            robot.shooter.setRotator(robot.getPos());

                            if(Math.abs(robot.shooter.getRotatorError()) < Math.toRadians(0.25)){
                                robot.shooter.resetRotatorPID();
                            }

                            if(robot.shooter.getFlywheelVelcoity(robot.getData2()) >= 1980 && Math.abs(robot.shooter.getRotatorError()) < Math.toRadians(0.75)){
                                if(elapsedTime.time() > 0.25){
                                    robot.shooter.flicker.flick();
                                }else{
                                    robot.shooter.flicker.resetTime();
                                }

                                robot.shooter.stopper.setPosition(robot.shooter.stopperOpen);
                            }
                        }
                    }else{
                        robot.shooter.resetRotatorPID();
                        robot.shooter.setRotator(0, robot.getPos());
                        elapsedTime.reset();
                    }

                    if(robot.getPos().vec().distTo(POWER_SHOT_POS.vec()) < 15){
                        robot.shooter.setFlywheelVelocity(2000, robot.shooter.getFlywheelVelcoity(robot.getData2()));
                    }
                    break;
                case 5:
                    robot.shooter.setFlywheelPower(0.0);
                    points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(325)), 1.0d, 1.0d, 25));
                    if(!ringsFound) {
                        points.add(new CurvePoint(POWER_SHOT_POS.getX() - 5, POWER_SHOT_POS.getY() + 50, 1.0, 1.0, 25.0, 0));
                    }
                    points.add(new CurvePoint(PARK, 0.5d, 0.5d, 25));

                    if(PARK.vec().distTo(robot.getPos().vec()) < 2.5){
                        robot.wobbleGoal.lift();
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

            telemetry.addData("Position", robot.getPos());
            //robot.updateBulkData();
            //robot.updatePos();
            telemetry.addData("State", state);
            telemetry.update();
        }
    }

    public void sortTargets(){
        for(int i = 0; i < bounceBackPoints.size(); i++){
            int min = i;
            for(int j = i + 1; j < bounceBackPoints.size(); j++){
                if(bounceBackPoints.get(min).getY() > bounceBackPoints.get(j).getY()){
                    min = j;
                }
                Collections.swap(bounceBackPoints, min, i);
            }
        }

        if(bounceBackPoints.size() > 3){
            bounceBackPoints.subList(3, bounceBackPoints.size()).clear();
        }
    }
}
