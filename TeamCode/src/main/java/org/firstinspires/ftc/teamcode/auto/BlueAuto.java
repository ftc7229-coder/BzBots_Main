package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.Paths.PathsBlue;
import org.firstinspires.ftc.teamcode.auto.Paths.PathsRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BlueAuto", group = "Auto")
public class BlueAuto extends OpMode {
    List<Integer> idSamples = new ArrayList<>();
    Follower follower;
    PathsBlue paths;
    private Timer purpleTimer2, shooterTimerLeft, shooterTimerRight, sorterTimer, shooterTimer, greenTimer, purpleTimer1;                                                                                                                                                                                        //Idk
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    DcMotor turretMotor;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Launcher.runLauncher();
                follower.followPath(paths.startToShoot);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Intake.runIntake();
                    Launcher.openLauncher();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    Intake.reverseIntake();

                    follower.followPath(paths.moveToPPG);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    Launcher.closeLauncher();
                    Intake.runIntake();
                    follower.setMaxPower(.65);
                    follower.followPath(paths.moveToIntakePPG);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.shootPPG);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    Launcher.openLauncher();
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    Intake.reverseIntake();
                    follower.followPath(paths.moveToPGP);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    Intake.runIntake();
                    Launcher.closeLauncher();
                    follower.setMaxPower(.65);
                    follower.followPath(paths.moveToIntakePGP);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.shootPGP);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()){
                    Launcher.openLauncher();
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 1.5){
                    Intake.reverseIntake();
                    follower.followPath(paths.moveToGPP);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    Launcher.closeLauncher();
                    Intake.runIntake();
                    follower.setMaxPower(.65);
                    follower.followPath(paths.moveToIntakeGPP);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(paths.shootGPP);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()){
                    Launcher.openLauncher();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds()>1.5){
                    follower.followPath(paths.moveToPGP);
                    Launcher.closeLauncher();
                    setPathState(15);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        Launcher.init(hardwareMap);
        Intake.init(hardwareMap);
        purpleTimer1 = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PathsRed.startPose.mirror());
        paths = new PathsBlue(follower);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        super.start();
    }

    @Override
    public void loop() {
        telemetry.addData("Path State:", pathState);
        follower.update();
        autonomousPathUpdate();
        Launcher.setShooterVelo();
    }

    @Override
    public void stop() {
        super.stop();
    }
}