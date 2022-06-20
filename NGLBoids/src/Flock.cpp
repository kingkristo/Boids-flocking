#include "Flock.h"
#include "Random.h"
#include <ngl/Vec3.h>
#include <ngl/Mat4.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOFactory.h>
#include <algorithm>

Flock::Flock(size_t _flockSize)
{
  //add obsticles
  m_obsCenter[0] = ngl::Vec3(3.0f, 3.0f, -3.0f);
  m_obsRadius[0] = 2.0f;
  m_obsCenter[1] = ngl::Vec3(-2.0f, -2.0f, -2.0f);
  m_obsRadius[1] = 2.0f;
  m_obsCenter[2] = ngl::Vec3(-1.0f, 1.0f, 3.0f);
  m_obsRadius[2] = 2.0f;
  m_obsCenter[3] = ngl::Vec3(3.0f, -3.0f, 3.0f);
  m_obsRadius[3] = 2.0f;
  //add agents
  m_flockSize = _flockSize;

  addAgents(0);
  //vao object buffer
  m_vao = ngl::vaoFactoryCast<ngl::MultiBufferVAO>(ngl::VAOFactory::createVAO(ngl::multiBufferVAO, GL_TRIANGLE_STRIP));
  m_vao->bind();
  std::vector<ngl::Vec3> verts =
      {
          ngl::Vec3(0.0f, 0.1f, 0.1f),
          ngl::Vec3(0.0f, 0.0f, -0.1f),
          ngl::Vec3(-0.05f, 0.0f, 0.1f),
          ngl::Vec3(0.0f, 0.1f, 0.1f),
          ngl::Vec3(0.0f, 0.0f, -0.1f),
          ngl::Vec3(0.05f, 0.0f, 0.1f),
          ngl::Vec3(0.0f, 0.1f, 0.1f),
          ngl::Vec3(0.0f, 0.0f, 0.15f),
          ngl::Vec3(-0.05f, 0.0f, 0.1f),
          ngl::Vec3(0.0f, 0.1f, 0.1f),
          ngl::Vec3(0.0f, 0.0f, 0.15f),
          ngl::Vec3(0.05f, 0.0f, 0.1f)};

  //simple "fake" shading to give boid bodies contrast without the need for any lighting calculations
  m_colour.resize(verts.size());
  for (int i = 0; i < verts.size(); i++)
  {
    if (i < verts.size() / 2 - 2)
    {
      //"lit" side
      m_colour[i] = ngl::Vec3(0, 1, 0);
    }
    else
    {
      //"shadow" side
      m_colour[i] = ngl::Vec3(0, 0.5, 0);
    }
  }

  m_vao->setData(ngl::MultiBufferVAO::VertexData(0, 0)); //points
  m_vao->setData(ngl::MultiBufferVAO::VertexData(0, 0)); //colours
  //only assign to this buffer once and modify the MVP for scale, rotation and translation
  m_vao->setData(0, ngl::MultiBufferVAO::VertexData(verts.size() * sizeof(ngl::Vec3), verts[0].m_x));
  // now we set the attribute pointer to be 0 (as this matches vertIn in our shader)
  m_vao->setVertexAttributePointer(0, 3, GL_FLOAT, 0, 0);
  //Colour buffer
  m_vao->setData(1, ngl::MultiBufferVAO::VertexData(verts.size() * sizeof(ngl::Vec3), m_colour[0].m_x));
  m_vao->setVertexAttributePointer(1, 3, GL_FLOAT, 0, 0);
  m_vao->setNumIndices(verts.size());
  m_vao->unbind();
}

void Flock::addAgents(int offset)
{
  //resize arrays
  m_surroundingNeighbors.resize(m_flockSize);
  m_agents.resize(m_flockSize);
  //emit new boids from offset position
  ngl::Vec3 emitDir = ngl::Vec3(0, 10, 0);
  float spread = 0.05f;
  for (size_t i = offset; i < m_flockSize; i++)
  {
    m_agents[i].pos = (0, 0, 0);
    m_agents[i].vel = emitDir * Random::randomPositiveFloat() + Random::randomVectorOnSphere() * spread;
  }
  //calculate spacial partition memberships
  setNeighborhoodRadius(m_neighborhoodRadius);
}

void Flock::update(float _delta)
{
  for (size_t i = 0; i < m_flockSize; i++)
  {
    //find spacial partition before position change
    int offset = (int)(1 / m_neighborhoodRadius * m_screenEdge);
    int prevBinX = int(m_agents[i].pos.m_x * 1 / m_neighborhoodRadius) + offset;
    int prevBinY = int(m_agents[i].pos.m_y * 1 / m_neighborhoodRadius) + offset;
    int prevBinZ = int(m_agents[i].pos.m_z * 1 / m_neighborhoodRadius) + offset;
    m_surroundingNeighbors.clear();
    //for the current spacial partition and all immediatly surrounding ones find neighboring agents
    for (int i = -1; i < 2; i++)
    {
      for (int j = -1; j < 2; j++)
      {
        for (int k = -1; k < 2; k++)
        {
          //prevent adding to bins outside of range (in edge case where boids go outside of the scene)
          if (!(prevBinX + i < 0 || prevBinX + i > m_maxBinIndex || prevBinY + j < 0 ||
                prevBinY + j > m_maxBinIndex || prevBinZ + k < 0 || prevBinZ + k > m_maxBinIndex))
          {
            m_surroundingNeighbors.insert(m_surroundingNeighbors.end(), m_binDivisions[prevBinX + i][prevBinY + j][prevBinZ + k].begin(),
                                          m_binDivisions[prevBinX + i][prevBinY + j][prevBinZ + k].end());
          }
        }
      }
    }
    //flock
    m_acceleration = startFlocking(m_agents[i], m_surroundingNeighbors, _delta);
    //a zero m_acceleration will cause crash upon normalization
    if (!m_acceleration.m_x + m_acceleration.m_y + m_acceleration.m_z == 0)
    {
      //normalize necessary
      m_acceleration.normalize();
    }
    //scale speed by max steering and add to current velocity
    m_acceleration = m_acceleration * m_maxSteering;
    //scale delta for better behaviour
    m_agents[i].vel += m_acceleration * _delta * m_deltaScale;
    //limit velocity by max speed
    m_agents[i].vel.normalize();
    m_agents[i].vel = m_agents[i].vel * m_maxSpeed;
    //new position
    m_agents[i].pos += m_agents[i].vel * _delta * m_deltaScale;
    //collisions with scene cube walls
    //bounce off of walls
    if (m_edgeBarrier)
    {
      //keep agents from moving beyond boundery in case this happens during slowdown
      if (m_agents[i].pos.m_z >= m_screenEdge)
      {
        m_agents[i].pos.m_z = m_screenEdge;
        m_agents[i].vel.m_z += -3;
      }
      else if (m_agents[i].pos.m_z <= -m_screenEdge)
      {
        m_agents[i].pos.m_z = -m_screenEdge;
        m_agents[i].vel.m_z += 3;
      }
      if (m_agents[i].pos.m_y >= m_screenEdge)
      {
        m_agents[i].pos.m_y = m_screenEdge;
        m_agents[i].vel.m_y += -3;
      }
      else if (m_agents[i].pos.m_y <= -m_screenEdge)
      {
        m_agents[i].pos.m_y = -m_screenEdge;
        m_agents[i].vel.m_y += 3;
      }
      if (m_agents[i].pos.m_x >= m_screenEdge)
      {
        m_agents[i].pos.m_x = m_screenEdge;
        m_agents[i].vel.m_x += -3;
      }
      else if (m_agents[i].pos.m_x <= -m_screenEdge)
      {
        m_agents[i].pos.m_x = -m_screenEdge;
        m_agents[i].vel.m_x += 3;
      }
    }
    //teleport to the other side on wall hit
    else
    {
      if (m_agents[i].pos.m_z >= m_screenEdge)
      {
        m_agents[i].pos.m_z = m_screenEdge;
        m_agents[i].pos.m_z = -m_agents[i].pos.m_z;
      }
      else if (m_agents[i].pos.m_z <= -m_screenEdge)
      {
        m_agents[i].pos.m_z = -m_screenEdge;
        m_agents[i].pos.m_z = -m_agents[i].pos.m_z;
      }
      if (m_agents[i].pos.m_y >= m_screenEdge)
      {
        m_agents[i].pos.m_y = m_screenEdge;
        m_agents[i].pos.m_y = -m_agents[i].pos.m_y;
      }
      else if (m_agents[i].pos.m_y <= -m_screenEdge)
      {
        m_agents[i].pos.m_y = -m_screenEdge;
        m_agents[i].pos.m_y = -m_agents[i].pos.m_y;
      }
      if (m_agents[i].pos.m_x >= m_screenEdge)
      {
        m_agents[i].pos.m_x = m_screenEdge;
        m_agents[i].pos.m_x = -m_agents[i].pos.m_x;
      }
      else if (m_agents[i].pos.m_x <= -m_screenEdge)
      {
        m_agents[i].pos.m_x = -m_screenEdge;
        m_agents[i].pos.m_x = -m_agents[i].pos.m_x;
      }
    }

    //calculate new spacial partition
    int binX = int(m_agents[i].pos.m_x * 1 / m_neighborhoodRadius) + offset;
    int binY = int(m_agents[i].pos.m_y * 1 / m_neighborhoodRadius) + offset;
    int binZ = int(m_agents[i].pos.m_z * 1 / m_neighborhoodRadius) + offset;
    //update partitions if needed (validate to avoid seg fault)
    if ((binX != prevBinX || binY != prevBinY || binZ != prevBinZ) &&
        binX >= 0 && binX <= m_maxBinIndex && binY >= 0 && binY <= m_maxBinIndex && binZ >= 0 && binZ <= m_maxBinIndex)
    {
      //remove from old bin
      m_binDivisions[prevBinX][prevBinY][prevBinZ].erase(std::remove(m_binDivisions[prevBinX][prevBinY][prevBinZ].begin(),
                                                                     m_binDivisions[prevBinX][prevBinY][prevBinZ].end(), &m_agents[i]),
                                                         m_binDivisions[prevBinX][prevBinY][prevBinZ].end());
      //add to new bin
      m_binDivisions[binX][binY][binZ].push_back(&m_agents[i]);
    }
  }
}

void Flock::render(ngl::Mat4 MVP) const
{
  for (size_t i = 0; i < m_flockSize; i++)
  {
    ngl::Mat4 rotX;
    ngl::Mat4 rotY;
    ngl::Mat4 scale;
    ngl::Mat4 translate;
    //spherical rotation based on Jons thesis (available at https://nccastaff.bournemouth.ac.uk/jmacey/MastersProject/MyMSc/Thesis.pdf)
    float yrot = atan2(m_agents[i].vel.m_x, m_agents[i].vel.m_z) + 3.18;
    float r = m_agents[i].vel.length();
    float xrot = asin(m_agents[i].vel.m_y / r);
    ; //1.5708;
    //convert to deg
    xrot = xrot * 57.2958;
    yrot = yrot * 57.2958;
    rotX.rotateX(xrot);
    rotY.rotateY(yrot);
    scale.scale(m_agentScale, m_agentScale, m_agentScale);
    translate.translate(m_agents[i].pos.m_x, m_agents[i].pos.m_y, m_agents[i].pos.m_z);
    //rotate x -> rotate y -> scale -> translate -> camera
    ngl::ShaderLib::setUniform("MVP", MVP * translate * scale * rotY * rotX);
    m_vao->bind();
    m_vao->draw();
    m_vao->unbind();
  }
}

//QT attribute setters
void Flock::setMaxSpeed(float _maxSpeed)
{
  m_maxSpeed = _maxSpeed;
}
void Flock::setMaxSteering(float _maxSteering)
{
  m_maxSteering = _maxSteering;
}
void Flock::setAlignmentStrength(float _alignment)
{
  m_alignmentStrength = _alignment;
}
void Flock::setCohesionStrength(float _cohesion)
{
  m_cohesionStrength = _cohesion;
}
void Flock::setSeperationStrength(float _seperation)
{
  m_seperationStrength = _seperation;
}
void Flock::setDesiredSeperation(float _distance)
{
  m_desiredSeperation = _distance;
}

void Flock::setNeighborhoodRadius(float _radius)
{
  //previously had a set size of box for the partition, then when radius increases append more and more boxes (SLOW)
  //now only the box of the agent and its immediate neighbors are conactinated, but upon changing the radius these box allocations are recalculated
  //this means some slowdown will occur upon changing radius, but sim will be faster after recalculation
  m_neighborhoodRadius = _radius;
  int offset = (int)(1 / _radius * 4.5);
  //clear old bins
  for (size_t i = 0; i < 20; i++)
  {
    for (size_t j = 0; j < 20; j++)
    {
      for (size_t k = 0; k < 20; k++)
      {
        m_binDivisions[i][j][k].clear();
      }
    }
  }
  //add to new bins
  for (size_t i = 0; i < m_flockSize; i++)
  {
    int binX = int(m_agents[i].pos.m_x * 1 / _radius) + offset;
    int binY = int(m_agents[i].pos.m_y * 1 / _radius) + offset;
    int binZ = int(m_agents[i].pos.m_z * 1 / _radius) + offset;
    if (binX <= m_maxBinIndex && binY <= m_maxBinIndex && binZ <= m_maxBinIndex)
    {
      m_binDivisions[binX][binY][binZ].push_back(&m_agents[i]);
    }
  }
}
void Flock::setFlockSize(int _size)
{
  //Resize Bins (preallocating here will be faster than constantly pushing to back and changing size each time)
  for (int i = 0; i < 20; i++)
  {
    for (int j = 0; j < 20; j++)
    {
      for (int k = 0; k < 20; k++)
      {
        m_binDivisions[i][j][k].resize(m_flockSize);
      }
    }
  }
  //decreasing flock
  if (_size < m_flockSize)
  {
    m_flockSize = _size;
    m_agents.resize(m_flockSize);
    //recalculate bins
    setNeighborhoodRadius(m_neighborhoodRadius);
  }
  else if (_size > m_flockSize)
  {
    int offset = m_flockSize;
    m_flockSize = _size;
    addAgents(offset);
  }
}
//teleport or bounce from walls
void Flock::toggleEdgeBarrier()
{
  m_edgeBarrier = !m_edgeBarrier;
}
void Flock::setAgentSize(float _scale)
{
  m_agentScale = _scale;
}
void Flock::toggleObsticles()
{
  m_useObsticles = !m_useObsticles;
}

ngl::Vec3 Flock::startFlocking(Agent &current, std::vector<Agent *> &partition, float _delta)
{
  m_seperationForce = seperate(current, partition);
  m_cohesionForce = cohesion(current, partition);
  m_alignmentForce = align(current, partition);
  m_avoidForce = collisionAvoidance(current, _delta);
  //scale and combine rules
  m_seperationForce = m_seperationForce * m_seperationStrength;
  m_cohesionForce = m_cohesionForce * m_cohesionStrength;
  m_alignmentForce = m_alignmentForce * m_alignmentStrength;
  m_finalVel = m_seperationForce + m_cohesionForce + m_alignmentForce + m_avoidForce;
  return (m_finalVel);
}

//raySphere method taken from NGL demo available at https://github.com/NCCA/Collisions
bool Flock::raySphere(ngl::Vec3 &_rayStart, ngl::Vec3 &_rayDir, ngl::Vec3 &_pos, GLfloat _radius)
{
  // variables for the Quadratic roots and discriminator
  GLfloat A, B, C, discrim;
  // normalize the ray
  _rayDir.normalize();
  // cal the A value as the dotproduct a.a
  A = _rayDir.dot(_rayDir);
  //b= 2*d.(Po-Pc)
  m_p = _rayStart - _pos;
  B = _rayDir.dot(m_p) * 2;
  // C = (Po-Pc).(Po-Pc)-r^2
  C = m_p.dot(m_p) - _radius * _radius;
  // finally get the descrim
  // b^2-4(ac)
  discrim = B * B - 4 * (A * C);
  // if the discrim <= 0.0 it's not a hit
  if (discrim <= 0.0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

ngl::Vec3 Flock::collisionAvoidance(Agent &current, float _delta)
{
  if (!m_useObsticles)
  {
    //no obsticles
    return ((0, 0, 0));
  }
  m_avoidance = (0, 0, 0);
  //3 gives the best balence between pre avoidence and not avoiding too soon
  float MAX_SEE_AHEAD = 3;
  m_normalVel = current.vel;
  m_normalVel.normalize();
  m_ahead = current.pos + (m_normalVel * MAX_SEE_AHEAD * _delta * m_deltaScale);
  float shortestDist = 100;
  //for all obsticles
  for (int i = 0; i < 4; i++)
  {
    bool intersect = raySphere(current.pos, current.vel, m_obsCenter[i], m_obsRadius[i]);
    //sqrt and pow removed for speed
    float posDist = (current.pos.m_x - m_obsCenter[i].m_x) * (current.pos.m_x - m_obsCenter[i].m_x) + (current.pos.m_y - m_obsCenter[i].m_y) * (current.pos.m_y - m_obsCenter[i].m_y) + (current.pos.m_z - m_obsCenter[i].m_z) * (current.pos.m_z - m_obsCenter[i].m_z);
    //if inside sphere or ray intersect within range and the closest obsticle evaluated
    //important to check if agent is inside sphere as although rare this can happen
    if ((posDist <= m_obsRadius[i] * m_obsRadius[i] || (posDist <= MAX_SEE_AHEAD * MAX_SEE_AHEAD && intersect)) && posDist < shortestDist)
    {
      shortestDist = posDist;
      m_avoidance = m_ahead - m_obsCenter[i];
    }
  }
  return m_avoidance;
}

ngl::Vec3 Flock::seperate(Agent &current, std::vector<Agent *> &partition)
{
  m_sum = (0, 0, 0);
  int count = 0;
  for (Agent *neighbor : partition)
  {
    //exact distance needed here so sqrt necessary
    float neighborDist = std::sqrt((current.pos.m_x - neighbor->pos.m_x) * (current.pos.m_x - neighbor->pos.m_x) + (current.pos.m_y - neighbor->pos.m_y) * (current.pos.m_y - neighbor->pos.m_y) + (current.pos.m_z - neighbor->pos.m_z) * (current.pos.m_z - neighbor->pos.m_z));
    //if neighbor is too close
    if ((neighborDist > 0) && (neighborDist < m_desiredSeperation))
    {
      m_posVecDiff = current.pos - neighbor->pos;
      //normalize needed
      m_posVecDiff.normalize();
      m_posVecDiff = m_posVecDiff / neighborDist;
      m_sum = m_sum + m_posVecDiff;
      count++;
    }
  }
  if (count > 0)
  {
    m_sum = m_sum / count;
    m_steer = m_sum - current.vel;
    return m_steer;
  }
  else
  {
    return m_sum;
  }
}

ngl::Vec3 Flock::cohesion(Agent &current, std::vector<Agent *> &partition)
{
  m_sum = (0, 0, 0);
  int count = 0;
  for (Agent *neighbor : partition)
  {
    float neighborDistSquared = (current.pos.m_x - neighbor->pos.m_x) * (current.pos.m_x - neighbor->pos.m_x) + (current.pos.m_y - neighbor->pos.m_y) * (current.pos.m_y - neighbor->pos.m_y) + (current.pos.m_z - neighbor->pos.m_z) * (current.pos.m_z - neighbor->pos.m_z);
    //sqrt removed for speed
    if ((neighborDistSquared > 0) && (neighborDistSquared < m_neighborhoodRadius * m_neighborhoodRadius))
    {
      m_sum = m_sum + neighbor->pos;
      count++;
    }
  }
  if (count > 0)
  {
    m_sum = m_sum / count;
    //seek the average agent pos
    return (seek(m_sum, current));
  }
  else
  {
    return m_sum;
  }
}

ngl::Vec3 Flock::align(Agent &current, std::vector<Agent *> &partition)
{
  m_sum = (0, 0, 0);
  int count = 0;
  for (Agent *neighbor : partition)
  {
    //sqrt removed for speed
    float neighborDistSquared = (current.pos.m_x - neighbor->pos.m_x) * (current.pos.m_x - neighbor->pos.m_x) + (current.pos.m_y - neighbor->pos.m_y) * (current.pos.m_y - neighbor->pos.m_y) + (current.pos.m_z - neighbor->pos.m_z) * (current.pos.m_z - neighbor->pos.m_z);
    if ((neighborDistSquared > 0) && (neighborDistSquared < m_neighborhoodRadius))
    {
      m_sum += neighbor->vel;
      count++;
    }
  }
  if (count > 0)
  {
    //find average velocity of neighbors
    m_sum = m_sum / count;
    //normalize necessary
    m_sum.normalize();
    m_steer = m_sum - current.vel;
    return m_steer;
  }
  else
  {
    return m_sum;
  }
}

ngl::Vec3 Flock::seek(ngl::Vec3 &target, Agent &current)
{
  m_desired = target - current.pos;
  m_steer = m_desired - current.vel;
  return m_steer;
}
