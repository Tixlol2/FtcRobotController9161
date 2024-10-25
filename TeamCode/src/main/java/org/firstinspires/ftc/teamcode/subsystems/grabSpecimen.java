package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class grabSpecimen extends SequentialCommandGroup {

    public grabSpecimen(armSubsystem aSubsystem, clawSubsystem cSubsystem){

        addCommands(
                new armPIDFCommand(aSubsystem, -100, -150),
                new wristDownCommand(cSubsystem),
                new clawCloseCommand(cSubsystem),
                new wristUpCommand(cSubsystem),
                new armPIDFCommand(aSubsystem, -100, 0)
        );


    }




}
