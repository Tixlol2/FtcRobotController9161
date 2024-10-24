package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;

@Autonomous
public class pedroPathingTest extends LinearOpMode {

    Follower follower;




    Pose startPose = new Pose(10, 108, 0);
    Pose middlePose = new Pose(60, 108, 0);
    Pose endPose = new Pose(11, 110, 180);


    Point middle = new Point(60, 108, Point.CARTESIAN);
    Point end = new Point(endPose);

    Path path = new Path(new BezierLine(middle, end));

    armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        PathChain newPath = follower.pathBuilder()
                .addPath(path)
                .setLinearHeadingInterpolation(0, Math.toRadians(180))
                .build();
        follower.followPath(newPath);
        armSubsystem.setArmAngle(-30);

        while(opModeIsActive()){

            follower.update();





        }

    }





}
