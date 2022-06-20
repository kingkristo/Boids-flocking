#include <gtest/gtest.h>
#include "Flock.h"
#include "ngl/NGLInit.h"
#include <ngl/Vec3.h>
//Given that other code is heavily adapted from other sources and already tested only the flocking class is tested here
//in many cases it is argued that only public methods etc should be tested,
//here it did not make sense for the majority of flocking methods to be made public, nor did it make sense to add an arbitrary public interface just for testing purposes
//despite this it would be unwise to only test the public functionality of the flock, so friend functions are used as a solution.

//with that being said the testing strategy used here is fairly lightweight with tests mostly focused on spotting segmentation faults / compilation faulure.
//Due to the nature of boids the most effective way to test was visually, and explicit testing of individual attributes was unessacarry with gtest acting as a good fallback to see if any components had broken with new additions.

TEST(Flock,ctor)
{
  auto flock = Flock();
  //if flcoksize is set to default we can assume other attributes have been as well
  ASSERT_EQ(flock.m_flockSize, 300);
}

TEST(Flock,userCtor)
{
  ngl::NGLInit::initialize();
  auto flock = Flock(200);
  ASSERT_FLOAT_EQ(flock.m_flockSize, 200);
}

//test update
TEST(Flock,update)
{
  ngl::NGLInit::initialize();
  auto flock = Flock(100);
  ngl::Vec3 oldPos0 = flock.m_agents[0].pos;
  ngl::Vec3 oldVel0 = flock.m_agents[0].vel;
  ngl::Vec3 oldPos99 = flock.m_agents[99].pos;
  ngl::Vec3 oldVel99 = flock.m_agents[99].vel;
  float delta = 0.03;
  flock.update(delta);
  bool pos0Changed = (oldPos0 != flock.m_agents[0].pos);
  bool pos99Changed = (oldPos99 != flock.m_agents[99].pos);
  bool vel0Changed = (oldVel0 != flock.m_agents[0].vel);
  bool vel99Changed = (oldVel99 != flock.m_agents[99].vel);
  ASSERT_EQ(pos0Changed, true);
  ASSERT_EQ(pos99Changed, true);
  ASSERT_EQ(vel0Changed, true);
  ASSERT_EQ(vel99Changed, true);

  //test with very high delta for seg faults due to drifting
  auto flock2 = Flock(100);
  oldPos0 = flock2.m_agents[0].pos;
  oldVel0 = flock2.m_agents[0].vel;
  oldPos99 = flock2.m_agents[99].pos;
  oldVel99 = flock2.m_agents[99].vel;
  delta = 1000;
  flock.update(delta);
  pos0Changed = (oldPos0 != flock2.m_agents[0].pos);
  pos99Changed = (oldPos99 != flock2.m_agents[99].pos);
  vel0Changed = (oldVel0 != flock2.m_agents[0].vel);
  vel99Changed = (oldVel99 != flock2.m_agents[99].vel);
  ASSERT_EQ(pos0Changed, true);
  ASSERT_EQ(pos99Changed, true);
  ASSERT_EQ(vel0Changed, true);
  ASSERT_EQ(vel99Changed, true);
}

//testQT setters
TEST(Flock,setters)
{
  ngl::NGLInit::initialize();
  auto flock = Flock(100);
  flock.setMaxSpeed(200);
  ASSERT_FLOAT_EQ(flock.m_maxSpeed, 200);
  flock.setMaxSteering(200);
  ASSERT_FLOAT_EQ(flock.m_maxSteering, 200);
  flock.setAlignmentStrength(1);
  ASSERT_FLOAT_EQ(flock.m_alignmentStrength, 1);
  flock.setCohesionStrength(0.5);
  ASSERT_FLOAT_EQ(flock.m_cohesionStrength, 0.5);
  flock.setSeperationStrength(0.3);
  ASSERT_FLOAT_EQ(flock.m_seperationStrength, 0.3);
  flock.setDesiredSeperation(2);
  ASSERT_FLOAT_EQ(flock.m_desiredSeperation, 2);
  flock.setNeighborhoodRadius(0.5);
  ASSERT_FLOAT_EQ(flock.m_neighborhoodRadius, 0.5);
  flock.setFlockSize(20);
  ASSERT_EQ(flock.m_flockSize, 20);
  flock.toggleEdgeBarrier();
  ASSERT_EQ(flock.m_edgeBarrier, false);
  flock.setAgentSize(5);
  ASSERT_FLOAT_EQ(flock.m_agentScale, 5);
  flock.toggleObsticles();
  ASSERT_EQ(flock.m_useObsticles, true);
}
