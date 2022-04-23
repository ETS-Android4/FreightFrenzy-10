package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;

@TeleOp(name = "Testing Blue TeleOp Solo", group = "Competition")
public class BlueTeleOpSolo extends AbstractTeleOpSolo {
    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }
}
