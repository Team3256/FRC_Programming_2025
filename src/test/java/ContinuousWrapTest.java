import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ContinuousWrapTest {



    @Test
    public void testWrapPositive() {
        assertEquals(1.1, Arm.continuousWrapAtHome(.1, .9, 0));
    }
    @Test
    public void testWrapZero() {
        assertEquals(0.2, Arm.continuousWrapAtHome(0.2, 0.0, 0));
    }
    @Test
    public void testWrapNegative() {
        assertEquals(-1.1, Arm.continuousWrapAtHome(-.1, -1.8, 0));
    }

    @Test
    public void testWrapLongPathPositive() {
        assertEquals(1.1, Arm.continuousWrapAtHome(.1, 1.2, 1));
    }
    @Test
    public void testWrapLongPathNegative() {
        assertEquals(-.9, Arm.continuousWrapAtHome(.1, -.3, -1));
    }

    @Test
    public void testSourceRightToSourceLeft() {
        assertEquals(.84, Arm.continuousWrapAtHome(.84, .66, 1));
    }
    @Test
    public void testSourceLeftToSourceRight() {
        assertEquals(.66, Arm.continuousWrapAtHome(.66, .84, -1));
    }

}
