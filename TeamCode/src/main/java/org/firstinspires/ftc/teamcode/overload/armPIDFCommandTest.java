package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.armSubsystem;

@Config
@TeleOp
public class armPIDFCommandTest extends LinearOpMode {



    private PIDController pidFController= new PIDController(pAngle, iAngle, dAngle);

    public static double pAngle = .00, iAngle = 0.08, dAngle = 0.000, fAngle = -0.08;

    public static int angleTarget = 0;



    private int armAngle;
    private double anglePower;
    private double anglePIDFpower;
    private double anglefeedForward;

    private final double ticks_in_degree = (751.8 * 4) / 360;

    private final double ticks_in_inch = (537.7 / 112) / 25.4;

    private PIDController pidController= new PIDController(pExtend, iExtend, dExtend);

    public static double pExtend = 0.008, iExtend = 0, dExtend = 0, fExtend = 0;

    public static int target_in_ticksExtend = 0;



    private int extendPos;
    private double powerExtension;
    private double PIDFpowerExtend;

    @Override
    public void runOpMode() throws InterruptedException {
        armSubsystem m_armSubsystem = new armSubsystem(hardwareMap, "armExt", "armAng");
        waitForStart();
        while (!isStopRequested()) {
            angleTarget = m_armSubsystem.getAngleTargetTK();

        if(angleTarget >= -10){
            angleTarget = -10;
        } else if (angleTarget <= -500){angleTarget = -500;}
        //Angle motor
        //pidFController.setPID(pAngle, iAngle, dAngle);
        armAngle = m_armSubsystem.angleMotor.getCurrentPosition();
        anglePIDFpower = pidFController.calculate(armAngle, angleTarget);
        anglefeedForward = Math.cos(Math.toRadians(armAngle / ticks_in_degree)) * fAngle;
        anglePower = anglePIDFpower + anglefeedForward;
        if(anglePower > .8 ){
            anglePower = .8;
        } else if (anglePower < -.8){anglePower = -.8;}
        m_armSubsystem.angleMotor.setPower(anglePower);

        target_in_ticksExtend = m_armSubsystem.getExtTargetTK();
        //Extension motor
        if(target_in_ticksExtend >= -100){
            target_in_ticksExtend = -100;
        } else if (target_in_ticksExtend <= -3550){angleTarget = -3550;}
        //pidController.setPID(pExtend, iExtend, dExtend);
        extendPos = m_armSubsystem.extenderMotor.getCurrentPosition();
        PIDFpowerExtend = pidController.calculate(extendPos, target_in_ticksExtend);

        powerExtension = PIDFpowerExtend;
        if(powerExtension > .8 ){
            powerExtension = .8;
        } else if (powerExtension < -.8){powerExtension = -.8;}
        m_armSubsystem.extenderMotor.setPower(powerExtension);
        }


    }



}
