package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import java.util.function.DoubleSupplier;

public class AutoAimCommand extends Command {
    private final SwerveSubsystem drive;
    private final LauncherSubsystem launcher;
    private final DoubleSupplier vX, vY, vRot;

    public AutoAimCommand(SwerveSubsystem drive, LauncherSubsystem launcher, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRot) {
        this.drive = drive; this.launcher = launcher; this.vX = vX; this.vY = vY; this.vRot = vRot;
        addRequirements(drive, launcher);
    }

    @Override
    public void execute() {
        // 1. Get distance and set shooter/hood using your existing launcher methods
        double dist = drive.getAimDistance();
        launcher.setShooterVelocity(ShooterCurves.getShooterVelocity(dist));
        launcher.setHoodPos(ShooterCurves.getHoodPosition(dist));

        // 2. Read driver joysticks for movement
        double x = MathUtil.applyDeadband(vX.getAsDouble(), 0.1) * drive.maximumSpeed;
        double y = MathUtil.applyDeadband(vY.getAsDouble(), 0.1) * drive.maximumSpeed;
        double rot = MathUtil.applyDeadband(vRot.getAsDouble(), 0.1);

        // 3. Aim & Move (Manual override right stick, otherwise auto-aim curve)
        double spin = (Math.abs(rot) > 0) ? rot * drive.maximumSpeed : ShooterCurves.getRotationSpeed(drive.getAimError());
        drive.drive(new Translation2d(x, y), spin, true);
    }

    @Override
    public void end(boolean interrupted) { 
        launcher.setShooterVelocity(0); 
        drive.drive(new Translation2d(0, 0), 0, true); 
    }
}