package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class armPIDFCommand extends CommandBase {

    private final armSubsystem m_armSubsystem;

    private PIDController pidFController;

    public static double pAngle = 0.0035, iAngle = 0.05, dAngle = 0, fAngle = 0;

    public static int angleTarget = 0;



    private int armAngle;
    private double anglePower;
    private double anglePIDFpower;
    private double anglefeedForward;

    private final double ticks_in_degree = (751.8 / 4) / 360;

    private final double ticks_in_inch = (537.7 / 112) / 25.4;

    private PIDController pidController;

    public static double pExtend = 0, iExtend = 0, dExtend = 0, fExtend = 0;

    public static int target_in_ticksExtend = 0;



    private int extendPos;
    private double powerExtension;
    private double PIDFpowerExtend;

    public armPIDFCommand(armSubsystem subsystem) {
        m_armSubsystem = subsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        pidFController = new PIDController(pAngle, iAngle, dAngle);
        pidController = new PIDController(pExtend, iExtend, dExtend);

    }

    @Override
    public void execute() {
        angleTarget = m_armSubsystem.getAngleTargetTK();
        //Angle motor
        pidFController.setPID(pAngle, iAngle, dAngle);
        armAngle = m_armSubsystem.angleMotor.getCurrentPosition();
        anglePIDFpower = pidFController.calculate(armAngle, angleTarget);
        anglefeedForward = Math.cos(Math.toRadians(angleTarget / ticks_in_degree)) * fAngle;
        anglePower = anglePIDFpower + anglefeedForward;
        if(anglePower > .8 ){
            anglePower = .8;
        }
        m_armSubsystem.angleMotor.setPower(anglePower);

        target_in_ticksExtend = m_armSubsystem.getExtTargetTK();
        //Extension motor
        pidController.setPID(pExtend, iExtend, dExtend);
        extendPos = m_armSubsystem.extenderMotor.getCurrentPosition();
        PIDFpowerExtend = pidController.calculate(extendPos, target_in_ticksExtend);

        powerExtension = PIDFpowerExtend;
        if(powerExtension > .6 ){
            powerExtension = .6;}
        m_armSubsystem.extenderMotor.setPower(powerExtension);


    }

    @Override
    public boolean isFinished() {
        if (((armAngle >= angleTarget - 10) && (armAngle <= angleTarget + 10)) && ((extendPos >= target_in_ticksExtend - 10) && (extendPos <= target_in_ticksExtend + 10)));   {
            return true;
        }

    }
}
