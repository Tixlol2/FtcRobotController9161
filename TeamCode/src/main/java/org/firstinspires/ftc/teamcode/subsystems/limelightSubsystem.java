package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class limelightSubsystem extends SubsystemBase {


    Limelight3A ll3a;


    public limelightSubsystem(HardwareMap hMap, final String name, final int pipeline){

        ll3a = hMap.get(Limelight3A.class, name);

        ll3a.pipelineSwitch(pipeline);

    }


}
