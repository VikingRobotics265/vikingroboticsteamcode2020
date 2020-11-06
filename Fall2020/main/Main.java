package org.firstinspires.ftc.teamcode.vikingroboticsteamcode2020.Fall2020.main;
import org.firstinspires.ftc.teamcode.vikingroboticsteamcode2020.Fall2020.lib.*;

class Main extends Init {
    @Override
    public void runOpMode() {
        super.runOpMode();
        driveFor(1.0, 2500);
        sleep(500);
        driveFor(0.5, 1000);
    }
}
