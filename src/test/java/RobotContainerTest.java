import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.warriorb.lib.drivers.Mapper;

public class RobotContainerTest {

  RobotContainer frcRobot;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void test() {
    frcRobot = new RobotContainer();
    Mapper mapper = new Mapper();
    mapper.registerControllerMap(frcRobot.m_driverController);
    mapper.registerControllerMap(frcRobot.m_operatorController);
    mapper.dumpControllerMap();
    assertEquals(1, 1);
  }
}
