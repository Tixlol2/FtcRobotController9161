package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class armAnglePIDFCommand extends CommandBase {

    private final armSubsystem m_armSubsystem;

    private PIDController pidfController;

    public static double p = 0.0035, i = 0.05, d = 0, f = 0;

    public static int target_in_ticks = 0;

    private int target_in_degrees;

    private int arm_pos;
    private double power;
    private double PIDFpower;
    private double feedForward;

    private final double ticks_in_degree = (751.8 / 3) / 360;

    private final double ticks_in_inch = ticks_in_degree / 4.409;

    public armAnglePIDFCommand(armSubsystem subsystem) {
        m_armSubsystem = subsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        pidfController = new PIDController(p, i, d);


    }

    @Override
    public void execute() {
        target_in_ticks = m_armSubsystem.getAngleTargetTK();
        //Angle motor
        pidfController.setPID(p, i, d);
        arm_pos = m_armSubsystem.angleMotor.getCurrentPosition();
        PIDFpower = pidfController.calculate(arm_pos, target_in_ticks);
        feedForward = Math.cos(Math.toRadians(target_in_ticks / ticks_in_degree)) * f;
        power = PIDFpower + feedForward;
        if(power > .8 ){power = .8;}
        m_armSubsystem.angleMotor.setPower(power);
    }

    @Override
    public boolean isFinished() {
        if ((arm_pos >= target_in_ticks - 10) && (arm_pos <= target_in_ticks + 10)) {
            return true;
        } return false;

    }
}
