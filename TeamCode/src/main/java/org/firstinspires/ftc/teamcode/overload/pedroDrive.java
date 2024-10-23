package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.armPIDFCommand;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;


@TeleOp(name="Pedro Drive", group = "Drive")
public class pedroDrive extends LinearOpMode {

    //Class def


    double deflator;

    int angleTarget = 0;
    int extendTarget = 0;



    //Limelight3A ll3a;
    Follower follower;
    boolean driveCentric;
    CommandScheduler commandScheduler;



    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart();
        //During Initialization:
        //ll3a = hardwareMap.get(Limelight3A.class, "LL3a");



        commandScheduler = CommandScheduler.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //hMap, name of servo used for claw
        clawSubsystem clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver");
        //hMap, name of motor used to change the EXTENSION HEIGHT of the arm/slides
        armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
        armPIDFCommand armPIDFCommand = new armPIDFCommand(armSubsystem);





        armSubsystem.setDefaultCommand(armPIDFCommand);

        follower = new Follower(hardwareMap);



        follower.startTeleopDrive();



        while (opModeIsActive()) {
            //Drive
            // ----------------------------
            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;
            if (gamepad1.b) {
                driveCentric = false;
            } else if (gamepad1.a) {
                driveCentric = true;
            }


            // ----------------------------
            // Other Funcs Used in TeleOp
            // ----------------------------

            //Testing clawSubsystem
            if (gamepad2.b) {
                clawSubsystem.close();
            } else if (gamepad2.a) {
                clawSubsystem.open();
            }
            if (gamepad2.x) {
                clawSubsystem.setAnglePosition(1);
            } else if (gamepad2.y) {
                clawSubsystem.setAnglePosition(0);
            }

            //Testing armSubsystem
            angleTarget += (int) (Math.pow(gamepad2.left_stick_y, 3) * 4);
            extendTarget += (int) (Math.pow(gamepad2.right_stick_y, 3) * 120);



            // ----------------------------
            // Updaters
            // ----------------------------

            telemetry.addData("Current TK Pos of Angle: ", armSubsystem.getAnglePos());
            telemetry.addData("Current Target of Angle: ", armSubsystem.getAngleTargetTK());


            telemetry.addData("Current TK Pos of Extension: ", armSubsystem.getExtenderPos());
            telemetry.addData("Current Target of Extension: ", armSubsystem.getExtTargetTK());


            telemetry.addData("Motor Power Ang", armSubsystem.angleMotor.getPower());
            telemetry.addData("Motor Power Ext", armSubsystem.extenderMotor.getPower());

            telemetry.addData("PID Power Ang", armSubsystem.anglePower);
            telemetry.addData("PID Power Ext", armSubsystem.powerExtension);



            telemetry.addLine("Don't Crash!");
            telemetry.addData("Driver Centric?", driveCentric);





            //Controls ArmAngle
            armSubsystem.setArmAngle(angleTarget);
            //Controls Extension of Arm
            armSubsystem.setExtendTarget(extendTarget);


            commandScheduler.run();


            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, driveCentric);
            follower.update();

            //Call telemetry at the end because the smart guy on the FTC discord server said to
            telemetry.update();
            // ----------------------------
        }




    }
}
