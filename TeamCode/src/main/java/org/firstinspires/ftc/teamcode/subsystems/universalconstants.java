package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

public class universalconstants {

    public static double intakePowerActive = 1, intakePowerStop = 0, intakePowerReverse = -.6;
    public static double shooterVeloReady = 1070, shooterVeloIdle = 900;
    public static double shooterServoOpen = 0.40, shooterServoClose = 0.2;
    public static double lastPressTime = .2;
    public static double goalPosY = 136, goalPosX = 140;
    public static double encodersPerDegree =  60 /27 /*gear ratio*/ * 537.7/*Encoders per rotation*/ / 360;
    public static double acceptableDegrees = 1, maxDegreeRange = 95, cameraBufferTime = .2;
    public static final Pose startPose = new Pose(124, 121, Math.toRadians(90));
}
