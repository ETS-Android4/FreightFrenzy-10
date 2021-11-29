package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.oldutil.Alliance.RED;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class RedTeleOp extends AbstractTeleOp {
    @Override
    public void setAlliance() {
        this.alliance = RED;
    }
}
