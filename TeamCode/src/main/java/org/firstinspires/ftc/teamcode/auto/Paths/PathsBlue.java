package org.firstinspires.ftc.teamcode.auto.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathsBlue {
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

    public PathsBlue(Follower follower)
    {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose.mirror(), shootingPose.mirror()))
                .setLinearHeadingInterpolation(startPose.mirror().getHeading(), shootingPose.mirror().getHeading())
                .build();

        moveToPPG = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose.mirror(), PPG.mirror()))
                .setLinearHeadingInterpolation(shootingPose.mirror().getHeading(), PPG.mirror().getHeading())
                .build();

        moveToIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPG.mirror(), IntakePPG.mirror()))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootPPG = follower.pathBuilder()
                .addPath(new BezierLine(IntakePPG.mirror(), shootingPose.mirror()))
                .setLinearHeadingInterpolation(IntakePPG.mirror().getHeading(), shootingPose.mirror().getHeading())
                .build();

        moveToPGP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose.mirror(), PGP.mirror()))
                .setLinearHeadingInterpolation(shootingPose.mirror().getHeading(), PGP.mirror().getHeading())
                .build();

        moveToIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGP.mirror(), IntakePGP.mirror()))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootPGP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakePGP.mirror(), GPP.mirror(), shootingPose.mirror()))
                .setLinearHeadingInterpolation(IntakePGP.mirror().getHeading(), shootingPose.mirror().getHeading())
                .build();

        moveToGPP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose.mirror(), GPP.mirror()))
                .setLinearHeadingInterpolation(shootingPose.mirror().getHeading(), GPP.mirror().getHeading())
                .build();

        moveToIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP.mirror(), IntakeGPP.mirror()))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootGPP = follower.pathBuilder()
                .addPath(new BezierLine(IntakeGPP.mirror(), shootingPose.mirror()))
                .setLinearHeadingInterpolation(IntakeGPP.mirror().getHeading(), shootingPose.mirror().getHeading())
                .build();
    }
}
