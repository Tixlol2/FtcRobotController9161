package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.armHoldPosition;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawCloseCommand;
import org.firstinspires.ftc.teamcode.subsystems.clawOpenCommand;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.grabSpecimen;

@Autonomous(name = "Blue Scoring Tower")
public class blueScore extends LinearOpMode {

    public PathChain path1;
    public PathChain path2;
    public PathChain path3;
    public PathChain path4;
    public PathChain path5;
    public PathChain path6;
    public PathChain path7;
    public Follower follower;
    public int pathState = 0;

    armSubsystem armSubsystem;
    clawSubsystem clawSubsystem;

    public grabSpecimen grabSpecimen;
    public armHoldPosition armHoldPosition;

    CommandScheduler commandScheduler = CommandScheduler.getInstance();

    public clawCloseCommand clawClose;
    public clawOpenCommand clawOpen;

    @Override
    public void runOpMode() throws InterruptedException {
        //One time
        autonPoints autonPoints = new autonPoints();
        armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
        clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver");
        follower = new Follower(hardwareMap);
        follower.resetIMU();
        follower.setStartingPose(autonPoints.startBlueScore);

        armSubsystem.setArmTarget(-100);
        armSubsystem.setExtendTarget(-10);
        armHoldPosition = new armHoldPosition(armSubsystem);

        clawOpen = new clawOpenCommand(clawSubsystem);
        clawClose = new clawCloseCommand(clawSubsystem);

        grabSpecimen = new grabSpecimen(armSubsystem, clawSubsystem);

        armSubsystem.setDefaultCommand(armHoldPosition);

        //Will go to blueScore
        path1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.startBlueScore), new Point(autonPoints.blueScore))))
                .setConstantHeadingInterpolation(autonPoints.startBlueScore.getHeading())
                .build();
        //Turn to have back facing the blue scoring tower
        path2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(autonPoints.startBlueScore.getHeading(), autonPoints.blueScore.getHeading())
                .build();
        //Turn to face middle block
        path3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(autonPoints.blueScore.getHeading(), 22.5)
                .build();
        //Turn to have back facing the blue scoring tower
        path4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(22.5, autonPoints.blueScore.getHeading())
                .build();
        //Turn to face left most block
        path5 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(autonPoints.blueScore.getHeading(), 45)
                .build();
        //Turn to have back facing the blue scoring tower
        path6 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(45, autonPoints.blueScore.getHeading())
                .build();
        //Go park
        path7 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoints.blueScore), new Point(autonPoints.bluePark), new Point(new Pose(-60, -36)))))
                .setLinearHeadingInterpolation(autonPoints.blueScore.getHeading(), 0)
                .build();





        while(!isStarted()){
            //Looping during init
            commandScheduler.run();
        }



        while(!isStopRequested()){
            //Main Function
            autonomousPathUpdate();
            follower.update();
            telemetry.addData("Path State: ", pathState);
            follower.telemetryDebug(telemetry);
        }







    }

    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                follower.followPath(path1);
                follower.update();
                setPathState(1);
                break;
            case 1:
                follower.followPath(path2);
                follower.update();
                setPathState(2);
                break;
            case 2:
                follower.followPath(path3);
                follower.update();
                setPathState(3);
                break;
            case 3:
                follower.followPath(path4);
                follower.update();
                setPathState(4);
                break;
            case 4:
                follower.followPath(path5);
                follower.update();
                setPathState(5);
                break;
            case 5:
                follower.followPath(path6);
                follower.update();
                setPathState(6);
                break;
            case 6:
                follower.followPath(path7);
                follower.update();
                setPathState(-1);
                break;
            case -1:
                telemetry.addLine("Finished");
                break;

        }
    }

    public void setPathState(int state){
        pathState = state;
        autonomousPathUpdate();
    }

}
