//taken from particle lab

#include "Random.h"
#include <cmath>
#include <ngl/Vec3.h>

std::mt19937 Random::m_generator;

auto randomFloatDist = std::uniform_real_distribution<float>(-1.0f, 1.0f);
auto randomPositiveFloatDist = std::uniform_real_distribution<float>(0.0f, 1.0f);

ngl::Vec3 Random::randomVec3(float _mult)
{
  return ngl::Vec3(randomFloatDist(m_generator) * _mult, randomFloatDist(m_generator) * _mult, randomFloatDist(m_generator) * _mult);
}

ngl::Vec3 Random::randomPositiveVec3(float _mult)
{
  return ngl::Vec3(randomPositiveFloatDist(m_generator) * _mult, randomPositiveFloatDist(m_generator) * _mult, randomPositiveFloatDist(m_generator) * _mult);
}

float Random::randomFloat(float _mult)
{
  return randomFloatDist(m_generator) * _mult;
}
float Random::randomPositiveFloat(float _mult)
{
  return randomPositiveFloatDist(m_generator) * _mult;
}

ngl::Vec3 Random::randomVectorOnSphere(float _radius)
{
  float phi = randomPositiveFloat(static_cast<float>(M_PI * 2.0f));
  float costheta = randomFloat();
  float u = randomPositiveFloat();
  float theta = acos(costheta);
  float r = _radius * std::cbrt(u);
  return ngl::Vec3(r * sin(theta) * cos(phi),
                   r * sin(theta) * sin(phi),
                   r * cos(theta));
}
