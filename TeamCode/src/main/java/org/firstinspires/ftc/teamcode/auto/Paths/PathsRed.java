package org.firstinspires.ftc.teamcode.auto.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathsRed {
    public static final Pose startPose = new Pose(124, 121, Math.toRadians(90));
    final Pose shootingPose = new Pose(106, 106, Math.toRadians(43));

    final Pose PPG = new Pose(94, 85, Math.toRadians(0));
    final Pose PGP = new Pose(94, 61, Math.toRadians(0));
    final Pose GPP = new Pose(94, 38, Math.toRadians(0));

    final Pose IntakePPG = new Pose(120, 85, Math.toRadians(0));
    final Pose IntakePGP = new Pose(126, 61, Math.toRadians(0));
    final Pose IntakeGPP = new Pose(126, 38, Math.toRadians(0));

    public PathChain startToShoot;
    public PathChain moveToPPG, moveToIntakePPG, shootPPG;
    public PathChain moveToPGP, moveToIntakePGP, shootPGP;
    public PathChain moveToGPP, moveToIntakeGPP, shootGPP;

    public PathsRed(Follower follower)
    {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        moveToPPG = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, PPG))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), PPG.getHeading())
                .build();

        moveToIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPG, IntakePPG))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootPPG = follower.pathBuilder()
                .addPath(new BezierLine(IntakePPG, shootingPose))
                .setLinearHeadingInterpolation(IntakePPG.getHeading(), shootingPose.getHeading())
                .build();

        moveToPGP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, PGP))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), PGP.getHeading())
                .build();

        moveToIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGP, IntakePGP))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootPGP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakePGP, GPP, shootingPose))
                .setLinearHeadingInterpolation(IntakePGP.getHeading(), shootingPose.getHeading())
                .build();

        moveToGPP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, GPP))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), GPP.getHeading())
                .build();

        moveToIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP, IntakeGPP))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootGPP = follower.pathBuilder()
                .addPath(new BezierLine(IntakeGPP, shootingPose))
                .setLinearHeadingInterpolation(IntakeGPP.getHeading(), shootingPose.getHeading())
                .build();
    }
}
