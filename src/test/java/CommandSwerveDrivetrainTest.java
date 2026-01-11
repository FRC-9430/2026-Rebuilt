import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.util.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.junit.jupiter.api.*;

/**
 * Tests for CommandSwerveDrivetrain and TunerConstants configuration.
 */
class CommandSwerveDrivetrainTest {

    private CommandSwerveDrivetrain drivetrain;

    @BeforeAll
    static void initializeHAL() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        drivetrain = TunerConstants.createDrivetrain();
        CommandScheduler.getInstance().cancelAll();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        if (drivetrain != null) drivetrain.close();
    }

    // ==================== TUNER CONSTANTS VALIDATION ====================

    @Test
    @DisplayName("Module positions should follow WPILib coordinate convention")
    void modulePositions_shouldFollowWPILibConvention() {
        // WPILib: +X is forward, +Y is left
        // Front modules: +X, Back modules: -X
        // Left modules: +Y, Right modules: -Y
        
        assertTrue(TunerConstants.FrontLeft.LocationX > 0, "FrontLeft should have +X");
        assertTrue(TunerConstants.FrontLeft.LocationY > 0, "FrontLeft should have +Y");
        
        assertTrue(TunerConstants.FrontRight.LocationX > 0, "FrontRight should have +X");
        assertTrue(TunerConstants.FrontRight.LocationY < 0, "FrontRight should have -Y");
        
        assertTrue(TunerConstants.BackLeft.LocationX < 0, "BackLeft should have -X");
        assertTrue(TunerConstants.BackLeft.LocationY > 0, "BackLeft should have +Y");
        
        assertTrue(TunerConstants.BackRight.LocationX < 0, "BackRight should have -X");
        assertTrue(TunerConstants.BackRight.LocationY < 0, "BackRight should have -Y");
    }

    // ==================== COMMAND INTEGRATION TESTS ====================

    @Test
    @DisplayName("applyRequest should create command requiring drivetrain")
    void applyRequest_shouldRequireDrivetrain() {
        Command cmd = drivetrain.applyRequest(() -> new SwerveRequest.Idle());
        
        assertTrue(cmd.getRequirements().contains(drivetrain),
            "Command should require the drivetrain subsystem");
    }

    @Test
    @DisplayName("Multiple applyRequest commands should conflict")
    void multipleApplyRequests_shouldConflict() {
        Command cmd1 = drivetrain.applyRequest(() -> new SwerveRequest.Idle());
        Command cmd2 = drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
        
        // Commands requiring the same subsystem should share requirements
        assertEquals(cmd1.getRequirements(), cmd2.getRequirements(),
            "Both commands should require the same subsystem");
    }

    // ==================== POSE RESET TEST ====================

    @Test
    @DisplayName("resetPose should update reported pose")
    void resetPose_shouldUpdatePose() {
        Pose2d newPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));
        
        drivetrain.resetPose(newPose);
        
        Pose2d reported = drivetrain.getState().Pose;
        assertEquals(5.0, reported.getX(), 0.01);
        assertEquals(3.0, reported.getY(), 0.01);
        assertEquals(45.0, reported.getRotation().getDegrees(), 0.1);
    }

    // ==================== HARDWARE CONFIGURATION TEST ====================

    @Test
    @DisplayName("Drivetrain should have four modules with correct device types")
    void drivetrain_shouldHaveCorrectHardware() {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = drivetrain.getModules();
        
        assertEquals(4, modules.length, "Should have exactly 4 modules");
        
        for (int i = 0; i < 4; i++) {
            assertNotNull(modules[i].getDriveMotor(), "Module " + i + " should have drive motor");
            assertNotNull(modules[i].getSteerMotor(), "Module " + i + " should have steer motor");
            assertNotNull(modules[i].getEncoder(), "Module " + i + " should have encoder");
        }
    }
}