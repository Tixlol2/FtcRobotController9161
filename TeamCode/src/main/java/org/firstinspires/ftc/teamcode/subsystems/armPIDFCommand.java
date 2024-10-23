package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class armPIDFCommand extends CommandBase {

    private final armSubsystem m_armSubsystem;

    private PIDController pidFController;

    public  double pAngle = .0025, iAngle = .08, dAngle = 0.0005, fAngle = -0.12;

    public  int angleTarget = 0;



    private int armAngle;
    private double anglePower;
    private double anglePIDFpower;
    private double anglefeedForward;

    private final double ticks_in_degree = (751.8 * 4) / 360;

    private final double ticks_in_inch = (537.7 / 112) / 25.4;

    private PIDController pidController;

    public  double pExtend = 0.008, iExtend = 0, dExtend = 0, fExtend = 0;

    public  int target_in_ticksExtend = 0;



    private int extendPos;
    private double powerExtension;
    private double PIDFpowerExtend;

    public armPIDFCommand(armSubsystem subsystem) {
        m_armSubsystem = subsystem;
        addRequirements(m_armSubsystem);
        pidFController = new PIDController(pAngle, iAngle, dAngle);
        pidController = new PIDController(pExtend, iExtend, dExtend);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        angleTarget = m_armSubsystem.getAngleTargetTK();
        if(angleTarget >= -10){
            angleTarget = -10;
        } else if (angleTarget <= -500){angleTarget = -500;}
        //Angle motor
        pidFController.setPID(pAngle, iAngle, dAngle);
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
        pidController.setPID(pExtend, iExtend, dExtend);
        extendPos = m_armSubsystem.extenderMotor.getCurrentPosition();
        PIDFpowerExtend = pidController.calculate(extendPos, target_in_ticksExtend);

        powerExtension = PIDFpowerExtend;
        if(powerExtension > .8 ){
            powerExtension = .8;
        } else if (powerExtension < -.8){powerExtension = -.8;}
        m_armSubsystem.extenderMotor.setPower(powerExtension);


    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
