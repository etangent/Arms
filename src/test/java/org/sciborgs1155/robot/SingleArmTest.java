package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.SingleArm.SingleArm;

public class SingleArmTest {
  private SingleArm singleArm;
  private final double DELTA = 0.05;

  @BeforeEach
  void setup() {
    setupTests();
    singleArm = SingleArm.create();
  }

  @AfterEach
  void destroy() throws Exception {
    reset(singleArm);
  }

  @ParameterizedTest
  @ValueSource(doubles = {0, Math.PI, Math.PI / 2, Math.PI / 4, 3 * Math.PI / 4, 2})
  void attainsPosition(double position) {
    run(singleArm.goTo(() -> position));
    fastForward();

    assertEquals(position, singleArm.position(), DELTA);
  }
}
