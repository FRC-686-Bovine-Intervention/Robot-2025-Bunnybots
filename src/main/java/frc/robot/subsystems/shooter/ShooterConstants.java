package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.RobotConstants;
import frc.util.Environment;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public final class ShooterConstants {
    public static final GearRatio motorToMechRatio = new GearRatio()
        .sprocket(1)
        .sprocket(1)
    ;
        
    public static final LinearRelation flywheel = LinearRelation.wheelDiameter(Inches.of(6));

    public static final DoubleSupplier shooterSpeedEnvCoef = Environment.switchVar(
        () -> 1,
        () -> (new LoggedTunableNumber("Demo Constraints/Shooter Demo Speed", 0.5)).getAsDouble()
    );
    
    public static final LinearVelocity exitVelocity = MetersPerSecond.of(5);

    /* M, M/S */
    public static final InterpolatingDoubleTreeMap targetShooterSpeed = new InterpolatingDoubleTreeMap();
    static {
        targetShooterSpeed.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        targetShooterSpeed.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0);
        targetShooterSpeed.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0);
        targetShooterSpeed.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5);
        targetShooterSpeed.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5);
    }

    /* M, M/S */
    public static final InterpolatingDoubleTreeMap minimumShooterSpeed = new InterpolatingDoubleTreeMap();
    static {
        minimumShooterSpeed.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5-2);
        minimumShooterSpeed.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0-2);
        minimumShooterSpeed.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0-2);
        minimumShooterSpeed.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5-2);
        minimumShooterSpeed.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5-2);
    }

}
