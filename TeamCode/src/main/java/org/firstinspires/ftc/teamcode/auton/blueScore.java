package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;


public class blueScore extends LinearOpMode {

    public PathChain path1;
    public PathChain path2;
    public PathChain path3;
    public PathChain path4;
    public PathChain path5;
    public PathChain path6;
    public PathChain path7;



    @Override
    public void runOpMode() throws InterruptedException {
        //One time
        autonPoints autonPoints = new autonPoints();
        armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
        clawSubsystem clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver");
        Follower follower = new Follower(hardwareMap);
        follower.resetIMU();
        follower.setStartingPose(autonPoints.startBlueScore);

        //Will go to blueScore
        path1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.startBlueScore), new Point(autonPoints.blueScore))))
                .setConstantHeadingInterpolation(0)
                .build();
        //Turn to have back facing the blue scoring tower
        path2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(0, -45)
                .build();
        //Turn to face middle block
        path3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(-45, 22.5)
                .build();
        //Turn to have back facing the blue scoring tower
        path4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(22.5, -45)
                .build();
        //Turn to face left most block
        path5 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(-45, 45)
                .build();
        //Turn to have back facing the blue scoring tower
        path6 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoints.blueScore), new Point(autonPoints.blueScore))))
                .setLinearHeadingInterpolation(45, -45)
                .build();
        //Go park
        path7 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoints.blueScore), new Point(autonPoints.bluePark), new Point(new Pose(50, 72.5)))))
                .setLinearHeadingInterpolation(-45, 0)
                .build();





        while(!isStarted()){
            //Looping during init




        }








    }
}
