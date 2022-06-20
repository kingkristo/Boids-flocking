//adapted from QT NGL demo https://github.com/NCCA/QtNGL and particle system lab
#include <QMouseEvent>
#include <QGuiApplication>
#include <ngl/ShaderLib.h>
#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Util.h>
#include <iostream>
#include <memory>
#include "Flock.h"

NGLScene::NGLScene(QWidget *_parent) : QOpenGLWidget(_parent)
{

  // set this widget to have the initial keyboard focus
  setFocus();
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  this->resize(_parent->size());
}

NGLScene::~NGLScene()
{
  std::cout << "Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL(int _w, int _h)
{
  m_win.width = static_cast<int>(_w * devicePixelRatio());
  m_win.height = static_cast<int>(_h * devicePixelRatio());
  //project worked out here to account for resizing
  m_project = ngl::perspective(45.0f, static_cast<float>(_w) / _h, 0.1, 300);
}

//make this string a constant to avoid mistyping
//no overhead to this
const auto ColourShader = "ColourShader";

void NGLScene::initializeGL()
{
  // we must call that first before any other GL commands to load and link the
  // gl commands from the lib, if that is not done program will crash
  ngl::NGLInit::initialize();
  glClearColor(0.2f, 0.2f, 0.2f, 1.0f); // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);

  ngl::ShaderLib::loadShader(ColourShader, "shaders/ColourVertex.glsl",
                             "shaders/ColourFragment.glsl");
  ngl::ShaderLib::use(ColourShader);
  //create flock
  m_flock = std::make_unique<Flock>(300);
  //initialize previous time to prevent error
  m_previousTime = std::chrono::high_resolution_clock::now();
  //start timer for timer event
  startTimer(30);
  m_view = ngl::lookAt({10, 10, 10}, {0, 0, 0}, {0, 1, 0});
  //scene cube
  m_bbox = std::make_unique<ngl::BBox>(ngl::Vec3(0.0f, 0.0f, 0.0f), 10.0f, 10.0f, 10.0f);
  //simple black box obsticles
  m_obsticle[0] = std::make_unique<ngl::BBox>(ngl::Vec3(3.0f, 3.0f, -3.0f), 2.0f, 2.0f, 2.0f);
  m_obsticle[0]->setDrawMode(GL_TRIANGLE_STRIP);
  m_obsticle[1] = std::make_unique<ngl::BBox>(ngl::Vec3(-2.0f, -2.0f, -2.0f), 2.0f, 2.0f, 2.0f);
  m_obsticle[1]->setDrawMode(GL_TRIANGLE_STRIP);
  m_obsticle[2] = std::make_unique<ngl::BBox>(ngl::Vec3(-1.0f, 1.0f, 3.0f), 2.0f, 2.0f, 2.0f);
  m_obsticle[2]->setDrawMode(GL_TRIANGLE_STRIP);
  m_obsticle[3] = std::make_unique<ngl::BBox>(ngl::Vec3(3.0f, -3.0f, 3.0f), 2.0f, 2.0f, 2.0f);
  m_obsticle[3]->setDrawMode(GL_TRIANGLE_STRIP);
}

//updating each time step
void NGLScene::timerEvent(QTimerEvent *_timer)
{
  auto currentTime = std::chrono::high_resolution_clock::now();
  auto delta = std::chrono::duration<float,std::chrono::seconds::period>
  (currentTime-m_previousTime).count();
  m_flock->update(delta);
  update();
  m_previousTime=currentTime;
}


void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, m_win.width, m_win.height);
  //rotate using mouse
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  rotX.rotateX(m_win.spinXFace);
  rotY.rotateY(m_win.spinYFace);
  //mouse rotate
  m_mouseGlobalTX = rotX * rotY;
  //translate using right click and zoom w scroll wheel
  m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
  m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;
  ngl::Mat4 MVP = m_project * m_view * m_mouseGlobalTX;
  //set view matrix
  ngl::ShaderLib::use(ColourShader);
  //pass MVP to emmitter to be modified with boid position,orientation and scale
  m_flock->render(MVP);
  //use unmodified MVP for other objects
  ngl::ShaderLib::setUniform("MVP", MVP);
  m_bbox->draw();
  if (m_obsticles)
  {
    m_obsticle[0]->draw();
    m_obsticle[1]->draw();
    m_obsticle[2]->draw();
    m_obsticle[3]->draw();
  }
}
//QT setters, multiply as needed to account for QT slider resolution
void NGLScene::setMaxSpeed(int _maxSpeed)
{
  m_flock->setMaxSpeed(_maxSpeed * 0.01f);
}
void NGLScene::setMaxSteering(int _maxSteering)
{
  m_flock->setMaxSteering(_maxSteering * 0.01);
}
void NGLScene::setAlignmentStrength(int _alignment)
{
  m_flock->setAlignmentStrength(_alignment * 0.1);
}
void NGLScene::setCohesionStrength(int _cohesion)
{
  m_flock->setCohesionStrength(_cohesion * 0.1);
}
void NGLScene::setSeperationStrength(int _seperation)
{
  m_flock->setSeperationStrength(_seperation * 0.1);
}
void NGLScene::setDesiredSeperation(int _distance)
{
  m_flock->setDesiredSeperation(_distance * 0.1);
}
void NGLScene::setNeighborhoodRadius(int _radius)
{
  m_flock->setNeighborhoodRadius(_radius * 0.1);
}
void NGLScene::setFlockSize(int _size)
{
  m_flock->setFlockSize(_size);
}
void NGLScene::toggleEdgeBarrier()
{
  m_flock->toggleEdgeBarrier();
}
void NGLScene::setAgentSize(int _scale)
{

  m_flock->setAgentSize(_scale * 0.1);
}
void NGLScene::toggleObsticles()
{
  m_obsticles = !m_obsticles;
  m_flock->toggleObsticles();
}
