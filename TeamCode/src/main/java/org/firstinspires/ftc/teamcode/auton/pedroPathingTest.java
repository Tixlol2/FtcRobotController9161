package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class pedroPathingTest extends LinearOpMode {

    Follower follower;




    Pose startPose = new Pose(10, 108, 0);
    Pose middlePose = new Pose(60, 108, 0);
    Pose endPose = new Pose(11, 110, 180);


    Point middle = new Point(60, 108, Point.CARTESIAN);
    Point end = new Point(endPose);

    Path path = new Path(new BezierCurve(middle, end));

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();



        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        PathChain newPath = follower.pathBuilder()
                .addPath(path)
                .build();

        while(opModeIsActive()){

            follower.followPath(newPath);



        }

    }





}
