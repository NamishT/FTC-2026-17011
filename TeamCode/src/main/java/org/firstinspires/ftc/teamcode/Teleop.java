package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Teleop extends OpMode {
    Drivebase drivebase = new Drivebase();

    Shooter shooter = new Shooter();

    @Override
    public void init(){

    }

    @Override
    public void loop(){
        drivebase.mecanumDrive();
        shooter.shooting_machine();

    }

}
