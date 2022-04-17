package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.util.Alliance.RED;

@TeleOp(name = "Red TeleOp Solo", group = "Competition")
public class RedTeleOpSolo extends AbstractTeleOpSolo {
    @Override
    public void setAlliance() {
        this.alliance = RED;
    }
}
