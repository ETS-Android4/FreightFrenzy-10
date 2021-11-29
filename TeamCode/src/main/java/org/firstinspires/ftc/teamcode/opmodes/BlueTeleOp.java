package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.oldutil.Alliance.BLUE;

@TeleOp(name = "Blue TeleOp", group = "Competition")
public class BlueTeleOp extends AbstractTeleOp {
    @Override
    public void setAlliance() {
        this.alliance = BLUE;
    }
}
