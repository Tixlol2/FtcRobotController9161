package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.armPIDFCommand;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;

@Config
@TeleOp
public class armSubclassTesting extends LinearOpMode {

    private PIDController controller;


    public static int target = 0;

    private final double ticks_in_degree = (751.8 * 4) / 360;


    CommandScheduler scheduler;


    @Override
    public void runOpMode() throws InterruptedException {


        scheduler = scheduler.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();


        armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
        armPIDFCommand armPIDFCommand = new armPIDFCommand(armSubsystem);

        armSubsystem.setDefaultCommand(armPIDFCommand);


        while(!isStopRequested()){

            armSubsystem.setArmTarget(target);

            scheduler.run();



            telemetry.addData("Current Pos: ", armSubsystem.getAnglePos());
            telemetry.addData("Current Set Target: ", target);
            telemetry.addData("Current REAL Target", armSubsystem.getAngleTargetTK());

            telemetry.update();



        }



    }
}
