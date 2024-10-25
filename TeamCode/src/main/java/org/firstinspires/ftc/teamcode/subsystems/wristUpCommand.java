package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;

public class wristUpCommand extends CommandBase {

    private final clawSubsystem m_clawSubsystem;

    public wristUpCommand(clawSubsystem subsystem){

        m_clawSubsystem = subsystem;

    }

    @Override
    public void initialize(){
        m_clawSubsystem.setAnglePosition(0);
    }

    @Override
    public boolean isFinished() {
        if (m_clawSubsystem.driverOfClaw.getPosition() == 0) {
            return true;
        } else {
            return false;
        }

    }

}
