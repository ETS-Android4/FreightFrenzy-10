package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.util.Alliance.BLUE;

@TeleOp(name = "Blue TeleOp Solo", group = "Competition")
public class BlueTeleOpSolo extends AbstractTeleOpSolo {
    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }
}
