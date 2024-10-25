package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.subsystems.armPIDFCommand;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;

@Autonomous
public class pedroPathingTest extends LinearOpMode {

    Follower follower;

    CommandScheduler commandScheduler;


    Pose startPose = new Pose(10, 10, 0);
    Pose middlePose = new Pose(60, 10, 0);
    Pose endPose = new Pose(20,     10, 0);


    Point middle = new Point(60, 108, Point.CARTESIAN);
    Point end = new Point(endPose);

    Path path = new Path(new BezierLine(new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN), end));




    @Override
    public void runOpMode() throws InterruptedException {
        commandScheduler = CommandScheduler.getInstance();

        armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
        armPIDFCommand armPIDFCommand = new armPIDFCommand(armSubsystem, 0, 0);
        armSubsystem.setDefaultCommand(armPIDFCommand);


        waitForStart();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        armSubsystem.setArmTarget(-20);

        PathChain newPath = follower.pathBuilder()
                .addPath(path)
                .setConstantHeadingInterpolation(0)
                .build();
        follower.followPath(newPath);
        armSubsystem.setArmAngle(-60);

        while(opModeIsActive()){

            follower.update();
            commandScheduler.run();

            follower.telemetryDebug(telemetry);
            telemetry.update();






        }

    }





}
