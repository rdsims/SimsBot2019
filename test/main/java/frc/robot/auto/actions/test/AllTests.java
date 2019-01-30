package frc.robot.auto.actions.test;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import org.junit.runners.Suite.SuiteClasses;

@RunWith(Suite.class)
@SuiteClasses(
{ TestPathFollowerWithVisionAction.class, KinematicsTest.class, PathTest.class, TestRigidTransform2d.class})
public class AllTests
{

}
