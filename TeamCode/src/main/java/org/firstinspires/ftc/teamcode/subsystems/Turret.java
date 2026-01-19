package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.Paths.PathsBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class Turret {
    static PIDController controller;
    static PIDController controller2;

    public static double p = 0.035, i = 0, d = 0.001;
    public static double defult_f = -0.2;
    public static double f = defult_f;

    public static double p2 = 0.015, i2 = 0, d2 = 0.001;
    public static double defult_f2 = -0.2;
    public static double f2 = defult_f2;


    static Limelight3A limelight;
    public static int TurretState = 1;
    static DcMotor turretMotor;
    public static double goalOffset;

    public static double goalPosY = 136;
    public static double goalPosX = 140;
    public static double encododersPerDegree = 60/27 /*gear ratio*/ * 537.7/*Encoders per rotation*/ / 360;

    static Follower follower;
    public static final Pose startPose = new Pose(124, 121, Math.toRadians(90));

    public static double maxDegreeRange = 95;
    static Pose roboPose;

    public static boolean usingCamera = false;
    public static double acceptableDegrees = 1;
    public static double cameraBufferTime = .1;

    static Timer cameraTimer;

    public static void init(HardwareMap hardwareMap, Telemetry telemetry){
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        turretMotor = hardwareMap.dcMotor.get("m4");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        controller.setPID(p, i, d);
        controller2.setPID(p2, i2, d2);

        cameraTimer = new Timer();
    }

    public static void runTurret(Telemetry telemetry){
        follower.update();
        roboPose = follower.getPose();

        if (TurretState == 1){
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                telemetry.addData("Result Status", "Is Valid");
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                usingCamera = true;
                cameraTimer.resetTimer();
                for (LLResultTypes.FiducialResult tag : tags) {
                    double angleX = tag.getTargetXDegrees();
                    telemetry.addData("Tag " + tag.getFiducialId() + " X°", angleX);

                    goalOffset = angleX;

                    if(turret_angle_to_robot() - get_goal_angle() < 0){
                        f2 = -defult_f2;
                    } else if (turret_angle_to_robot() - get_goal_angle() > 0) {
                        f2 = defult_f2;
                    }
                    if (Math.abs(turret_angle_to_robot() - get_goal_angle()) < acceptableDegrees) {
                        f2 = 0;
                    }


                    if (Math.abs(get_goal_angle()) < maxDegreeRange) {
                        double pid = controller.calculate(0, goalOffset);
                        double power = pid + f2;
                        turretMotor.setPower(power);
                    } else {
                        turretMotor.setPower(0);
                    }

                }


            } else if (cameraTimer.getElapsedTimeSeconds() > cameraBufferTime){
                usingCamera = false;
                if(turret_angle_to_robot() - get_goal_angle() < 0){
                    f = -defult_f;
                } else if (turret_angle_to_robot() - get_goal_angle() > 0) {
                    f = defult_f;
                }
                if (Math.abs(turret_angle_to_robot() - get_goal_angle()) < acceptableDegrees) {
                    f = 0;
                }


                if (Math.abs(get_goal_angle()) < maxDegreeRange) {
                    double pid = controller.calculate(turret_angle_to_robot(), get_goal_angle());
                    double power = pid + f;
                    turretMotor.setPower(power);
                } else {
                    turretMotor.setPower(0);
                }
            } else {
                turretMotor.setPower(0);
            }


        } else if (TurretState == 2) {
            double pid = controller.calculate(turret_angle_to_robot(), 0);
            double power = pid + f;
            turretMotor.setPower(power);


        } else{
            turretMotor.setPower(0);
        }
        telemetry.addData("Motor Power:", turretMotor.getPower());
        telemetry.addData("feedF", f);
        telemetry.addData("Motor Position", turretMotor.getCurrentPosition());
        telemetry.addData("Distance to Goal", get_goal_distance());
        telemetry.addData("Angle to Goal", get_goal_angle());
        telemetry.addData("Heading", roboPose.getHeading());
        telemetry.addData("x", roboPose.getX());
        telemetry.addData("y", roboPose.getY());
        telemetry.addData("turret angle", turret_angle_to_robot());
        telemetry.addData("unmod turret angle", turretMotor.getCurrentPosition());
        telemetry.addData("turret degrees offset", get_goal_angle() - turret_angle_to_robot());
        telemetry.addData("Using camera", usingCamera);
    }
    public static double get_goal_distance() {


        return  Math.abs(Math.sqrt(Math.pow(roboPose.getX() - goalPosX, 2) + Math.pow(roboPose.getY() - goalPosY, 2)));
    }

    public static double get_goal_angle() {
        roboPose = follower.getPose();
        return Math.toDegrees(roboPose.getHeading() - Math.toRadians(90) + Math.atan((roboPose.getX() - goalPosX)/(roboPose.getY() - goalPosY)));
    }

    public static double turret_angle_to_robot() {
        return turretMotor.getCurrentPosition() / encododersPerDegree;
    }
}
