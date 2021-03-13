package frc.robot.commands.ramseteAuto;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.config.Config;

public class TranslationScaled extends Translation2d {
    public TranslationScaled() {
        super();
    }
    
    public TranslationScaled(double x, double y) {
        super(x * Config.scaleField, y * Config.scaleField);
    }
}
