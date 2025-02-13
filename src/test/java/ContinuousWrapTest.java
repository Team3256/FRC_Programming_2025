import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ContinuousWrapTest {



    @Test
    public void testWrapPositive() {
        assertEquals(1.1, Arm.continuousWrapAtHome(.1, .9));
    }
    @Test
    public void testWrapNegative() {
        assertEquals(-2.1, Arm.continuousWrapAtHome(-.1, -1.8));
    }

}
