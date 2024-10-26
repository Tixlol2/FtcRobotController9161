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

@Autonomous(name = "Blue Observation Zone")
public class bluePark extends LinearOpMode {

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
        follower.setStartingPose(autonPoints.startBluePark);

        armSubsystem.setArmTarget(-100);
        armSubsystem.setExtendTarget(-10);
        armHoldPosition = new armHoldPosition(armSubsystem);

        clawOpen = new clawOpenCommand(clawSubsystem);
        clawClose = new clawCloseCommand(clawSubsystem);

        grabSpecimen = new grabSpecimen(armSubsystem, clawSubsystem);

        armSubsystem.setDefaultCommand(armHoldPosition);

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(autonPoints.startBluePark), new Point(autonPoints.bluePark)))
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
