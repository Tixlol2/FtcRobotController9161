package org.firstinspires.ftc.teamcode.auton;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.armHoldPosition;
import org.firstinspires.ftc.teamcode.subsystems.armPIDFCommand;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawCloseCommand;
import org.firstinspires.ftc.teamcode.subsystems.clawOpenCommand;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.grabSpecimen;


public class redScore extends LinearOpMode {


    public int pathState = 0;

    public PathChain getToMiddle;

    Pose start = new Pose(36, -60, 0);
    Pose middleSpec = new Pose(60, -40, 0);


    Follower follower;

    armSubsystem armSubsystem;
    clawSubsystem clawSubsystem;

    public grabSpecimen grabSpecimen;
    public armHoldPosition armHoldPosition;

    CommandScheduler commandScheduler = CommandScheduler.getInstance();

    public clawCloseCommand clawClose;
    public clawOpenCommand clawOpen;
    @Override
    public void runOpMode() throws InterruptedException {
        armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
        clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver");
        armSubsystem.setArmTarget(-100);
        armSubsystem.setExtendTarget(-10);
        armHoldPosition = new armHoldPosition(armSubsystem);

        clawOpen = new clawOpenCommand(clawSubsystem);
        clawClose = new clawCloseCommand(clawSubsystem);

        grabSpecimen = new grabSpecimen(armSubsystem, clawSubsystem);

        armSubsystem.setDefaultCommand(armHoldPosition);

        //During Init
        follower = new Follower(hardwareMap);
        follower.resetIMU();
        follower.setStartingPose(start);

        getToMiddle = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(middleSpec), new Point(middleSpec.getX() + .0001, middleSpec.getY() + .0001, 1), new Point(middleSpec))))
                .setConstantHeadingInterpolation(middleSpec.getHeading())
                .build();
        while(!isStarted()){commandScheduler.run();}

        while(!isStopRequested()){
            //Main Function

            follower.update();
            while(!follower.isBusy()) {
                autonomousPathUpdate();
            }
            commandScheduler.run();
            follower.telemetryDebug(telemetry);




        }


    }

    public void autonomousPathUpdate() {

        switch(pathState) {
            case 0:
                follower.followPath(getToMiddle);
                setPathState(1);
                break;
            case 1:
                commandScheduler.schedule(grabSpecimen);
                setPathState(2);
                break;

        }


    }

    public void setPathState(int state){
        pathState = state;
        autonomousPathUpdate();
    }

}
