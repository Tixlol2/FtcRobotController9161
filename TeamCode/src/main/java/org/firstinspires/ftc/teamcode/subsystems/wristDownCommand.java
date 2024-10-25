package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;

public class wristDownCommand extends CommandBase {

    private final clawSubsystem m_clawSubsystem;

    public wristDownCommand(clawSubsystem subsystem){

        m_clawSubsystem = subsystem;

    }

    @Override
    public void initialize(){
        m_clawSubsystem.setAnglePosition(1);
    }

    @Override
    public boolean isFinished() {
        if (m_clawSubsystem.driverOfClaw.getPosition() == 1) {
            return true;
        } else {
            return false;
        }

    }

}
