package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;

@TeleOp(name = "Blue TeleOp", group = "Competition")
public class BlueTeleOp extends AbstractTeleOp {
    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }
}
