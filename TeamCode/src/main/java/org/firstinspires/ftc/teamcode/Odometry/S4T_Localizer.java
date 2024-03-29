package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Config
public class S4T_Localizer {
    public static double TRACK_WIDTH1 = 2758.5928398760488810481200495949;//2765.7946010509571449916620398876;//2776.0495316690330858458660896672;//2766.5903757664166216705064587044;//2762.221712973584;

    public static double TRACK_WIDTH2 = 2430.0002062977823005120106478814;//2435.0706293059986372639215795995;//2437.4977421881500411343970569909;//2437.2590097735121981307437313458;//2434.8826022248963;

    private final double EPSILON = 1e-6;
    private static Pose2d myPose = new Pose2d(0, 0, 0);
    double prevHeading = 0;

    double prevx = 0;
    double prevy = 0;

    double prevely = 0;
    double prevery = 0;
    double prevelx = 0;
    double preverx = 0;

    double prevelyRaw = 0;
    double preveryRaw = 0;
    double prevelxRaw = 0;
    double preverxRaw = 0;

    public static double heading = 0;
    Telemetry telemetry;
    public static double k_strafe = 1.0;
    public static double k_vert = 1.0;
    public double TICKS_TO_INCHES_VERT = 203.766403;//201.67339734597755609;
    public double TICKS_TO_INCHES_STRAFE = 338.491591;//335.381388888888888;

    public static double clipping_strafe = 0.075;
    public static double clipping_vert = 0.075;
    public final Vector2d DASHBOARD_OFFSET_FROM_CENTER = new Vector2d(-48, -55);

    private HardwareMap hardwareMap;
    private TelemetryPacket packet;

    public S4T_Localizer(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = map;

        /*if(slamra == null){
            try{
                slamra = new T265Camera(OFFSET, odoCovariance, map.appContext);
            }catch (Exception e){
                slamra = null;
                telemetry.addData("ERROR","Couldn't find the camera... Trying again...");
            }
        }

        telemetry.addData("Is started?", slamra.isStarted());

        if(slamra != null){
            //slamra.setPose(new Pose2d(0, 0, new Rotation2d(0)));

            if(!slamra.isStarted()){
                slamra.start();
            }
        }*/
    }

    public void setPacket(TelemetryPacket packet){
        this.packet = packet;
    }

    private void addPacket(String caption, Object value){
        if(packet != null){
            packet.put(caption, value);
        }
    }

    public double wf = 1;
    public double ws = 1;
    double dtheta = 0;
    public Pose2d dashboardPos = new Pose2d(0, 0, 0);
    private double absoluteHeading = 0;

    public void update(double elxRaw, double elyRaw, double erxRaw, double eryRaw, double xVelo, double yVelo, RevBulkData data){
        double y = ((elyRaw + eryRaw)/2) / TICKS_TO_INCHES_VERT;
        double x = ((elxRaw + erxRaw)/2) / TICKS_TO_INCHES_STRAFE;
        double dy = y - prevy;
        double dx = x - prevx;

        prevx = x;
        prevy = y;

        double dElyRaw = elyRaw - prevelyRaw;
        double dEryRaw = eryRaw - preveryRaw;
        double dElxRaw = elxRaw - prevelxRaw;
        double dErxRaw = erxRaw - preverxRaw;

        prevelx = elxRaw;
        prevely = elyRaw;
        preverx = erxRaw;
        prevery = eryRaw;

        prevelxRaw = elxRaw;
        prevelyRaw = elyRaw;
        preverxRaw = erxRaw;
        preveryRaw = eryRaw;

        double dthetastrafe = (dErxRaw - dElxRaw) / TRACK_WIDTH2;
        double dthetavert = (dEryRaw - dElyRaw) / TRACK_WIDTH1;

        dtheta = weightedTheta(dx, dy, dthetavert, dthetastrafe);
        absoluteHeading += dtheta;
        heading %= 2 * Math.PI;

        heading += dtheta;
        heading %= 2 * Math.PI;

        telemetry.addData("Vertical Heading", Math.toDegrees(((eryRaw - elyRaw) / TRACK_WIDTH1) % (2 * Math.PI)));
        telemetry.addData("Strafe Heading", Math.toDegrees(((erxRaw - elxRaw) / TRACK_WIDTH2) % (2 * Math.PI)));

        Vector2 myVec = ConstantVelo(dy, dx, prevHeading, dtheta);
        prevHeading = heading;

        myPose = myPose.plus(new Pose2d(myVec.x, myVec.y, dtheta));
        myPose = new Pose2d(myPose.getX(), myPose.getY(), (Math.toRadians(360) - heading) % Math.toRadians(360));

        dashboardPos = new Pose2d(myPose.getY() + DASHBOARD_OFFSET_FROM_CENTER.getY(), -myPose.getX() + DASHBOARD_OFFSET_FROM_CENTER.getX(), (2 * Math.PI) - myPose.getHeading());
    }

    public double getAbsoluteHeading(){
        return absoluteHeading;
    }

    public void reset(){
        myPose = new Pose2d(0, 0, 0);
        heading = 0;
    }

    public double angleWrap(double angle){
        return ((2 * Math.PI) + angle) % (2 * Math.PI);
    }

    public double weightedTheta(double dx, double dy, double dthetavert, double dthetastrafe){
        determineWeights(dx, dy);

        if(Math.abs(wf) <= clipping_vert){
            wf = 0;
        }

        if(Math.abs(ws) <= clipping_strafe){
            ws = 0;
        }


        double value = 0;
        double total = wf + ws;
        if(total != 0){
            value = ((wf * dthetavert) + (ws * -dthetastrafe))/total;
        }else{
            value = (dthetavert - dthetastrafe)/2;
            //value = dthetavert;
        }

        addPacket("wf", wf);
        addPacket("ws", ws);

        return value;
    }

    private double prevdx = 0;
    private double prevdy = 0;

    public void setHeading(double heading){
        this.heading = heading;
    }

    public static double normalizationFactor = 8;

    public void determineWeights(double dx, double dy){
        double total = dx + dy;

        double mydx = (dx + prevdx)/2;
        double mydy = (dy + prevdy)/2;

        if(total != 0){
            mydx /= total;
            mydy /= total;
            mydx *= normalizationFactor;
            mydy *= normalizationFactor;
        }

        //If dx is higher, wf is lower and vice versa
        if(mydx != 0) {
            wf = Math.pow(Math.E, -k_strafe * Math.abs(mydx));
        }

        //If dy is high, ws is lower and vice versa
        if(mydy != 0) {
            ws = Math.pow(Math.E, -k_vert * Math.abs(mydy));
        }

        prevdx = dx;
        prevdy = dy;
    }

    public Pose2d getPose(){
        return myPose;
    }

    private Vector2 circleUpdate(double dr, double dx, double dy, double dtheta){
        if(dtheta <= EPSILON){
            double sineTerm = 1.0 - dtheta * dtheta / 6.0;
            double cosTerm = dtheta / 2.0;

            return new Vector2(
                    sineTerm * dx - cosTerm * dy,
                    cosTerm * dx + sineTerm * dy
            );
        }else{
            double radius = (TRACK_WIDTH1/2) + (dr/dtheta);
            double strafe_radius = dy/dtheta;
            return new Vector2(
                    (radius * (1 - Math.cos(dtheta))) + (strafe_radius * Math.sin(dtheta)),
                    (Math.sin(dtheta) * radius) + (strafe_radius * (1 - Math.cos(dtheta)))
            );
        }
    }

    public Vector2 ConstantVelo(double delta_y, double delta_x, double prev_heading, double delta_theta){
        Pose2d RawRobotDelta = new Pose2d(delta_x, delta_y, delta_theta);

        double sinterm = 0;
        double costerm = 0;

        if(delta_theta <= EPSILON){
            sinterm = 1.0 - delta_theta * delta_theta / 6.0;
            costerm = delta_theta / 2.0;
        }else{
            sinterm = Math.sin(delta_theta) / delta_theta;
            costerm = (1 - Math.cos(delta_theta)) / delta_theta;
        }

        Vector2 FeildCentricDelta = new Vector2((sinterm * RawRobotDelta.getX()) - (costerm * RawRobotDelta.getY()), (costerm * RawRobotDelta.getX()) + (sinterm * RawRobotDelta.getY()));
        FeildCentricDelta.rotate(prev_heading);

        return FeildCentricDelta;
    }
}
