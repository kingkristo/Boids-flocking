#ifndef FLOCK_H_
#define FLOCK_H_
#include <ngl/Vec3.h>
#include <ngl/Mat4.h>
#include <ngl/MultiBufferVAO.h>
#include <gtest/gtest.h>

class Flock
{
public:
  Flock() = default;
  ~Flock() = default;
  Flock(size_t _flockSize);
  void update(float _delta);
  void render(ngl::Mat4 MVP) const;
  //qt setters///////////////////////
  void setMaxSpeed(float _maxSpeed);
  void setMaxSteering(float _maxSteering);
  void setAlignmentStrength(float _alignment);
  void setCohesionStrength(float _cohesion);
  void setSeperationStrength(float _seperation);
  void setDesiredSeperation(float _distance);
  void setNeighborhoodRadius(float _radius);
  void setFlockSize(int _size);
  void toggleEdgeBarrier();
  void setAgentSize(float _scale);
  void toggleObsticles();
private:
  //It's customary to pass all primitive types by value because the operations necessary to copy them are typically just a single assembly instruction. Passing size_t s by value is therefore preferable over passing size_t s by reference
  //same true for float
  //struct added here to pos and vel can both be passed by pointer avoding more expensive indexing into previous m_pos and m_vel arrays
  struct Agent
  {
    ngl::Vec3 pos;
    ngl::Vec3 vel;
  };
  ngl::Vec3 startFlocking(Agent &current, std::vector<Agent *> &partition, float _delta);
  ngl::Vec3 seperate(Agent &current, std::vector<Agent *> &partition);
  ngl::Vec3 cohesion(Agent &current, std::vector<Agent *> &partition);
  ngl::Vec3 align(Agent &current, std::vector<Agent *> &partition);
  ngl::Vec3 seek(ngl::Vec3 &target, Agent &current);
  bool raySphere(ngl::Vec3 &_rayStart, ngl::Vec3 &_rayDir, ngl::Vec3 &_pos, GLfloat _radius);
  ngl::Vec3 collisionAvoidance(Agent &current, float _delta);
  std::vector<Agent *> m_binDivisions[20][20][20];
  int m_maxBinIndex = 19;
  //edge of scene cube
  float m_screenEdge = 4.5;
  void addAgents(int offset);
  std::vector<Agent> m_agents;
  //std::vector<ngl::Vec3> m_vel;
  //REMOVE THIS?
  std::vector<ngl::Vec3> m_colour;
  std::unique_ptr<ngl::MultiBufferVAO> m_vao;
  //QT params with default vals
  float m_deltaScale = 35;
  float m_maxSpeed = 0.1;
  float m_maxSteering = 0.1;
  float m_alignmentStrength = 0.4;
  float m_cohesionStrength = 0.2;
  float m_seperationStrength = 0.4;
  float m_desiredSeperation = 0.6;
  float m_neighborhoodRadius = 1.5f;
  bool m_edgeBarrier = true;
  bool m_useObsticles = false;
  float m_agentScale = 1.5;
  int m_flockSize = 300;
  ngl::Vec3 m_obsCenter[4];
  float m_obsRadius[4];
  //Vectors here to prevent lots of new Vec3 objects being created
  std::vector<Agent *> m_surroundingNeighbors;
  ngl::Vec3 m_acceleration;
  ngl::Vec3 m_seperationForce;
  ngl::Vec3 m_cohesionForce;
  ngl::Vec3 m_alignmentForce;
  ngl::Vec3 m_avoidForce;
  ngl::Vec3 m_finalVel;
  ngl::Vec3 m_avoidance;
  ngl::Vec3 m_ahead;
  ngl::Vec3 m_normalVel;
  ngl::Vec3 m_sum;
  ngl::Vec3 m_steer;
  ngl::Vec3 m_desired;
  ngl::Vec3 m_posVecDiff;
  ngl::Vec3 m_p;
  //tests
  FRIEND_TEST(Flock,ctor);
  FRIEND_TEST(Flock,userCtor);
  FRIEND_TEST(Flock,update);
  //FRIEND_TEST(Flock,render);
  FRIEND_TEST(Flock,setters);
};

#endif