#include "MainWindow.h"
#include "ui_MainWindow.h"
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), m_ui(new Ui::MainWindow)
{
  m_ui->setupUi(this);
  m_gl = new NGLScene(this);
  m_ui->s_mainWindowGridLayout->addWidget(m_gl, 0, 0, 2, 1);
  //set defaults
  m_ui->_maxSpeed->setValue(10);
  m_ui->_maxSteering->setValue(10);
  m_ui->_alignment->setValue(4);
  m_ui->_cohesion->setValue(2);
  m_ui->_seperation->setValue(4);
  m_ui->_radius->setValue(15);
  m_ui->_distance->setValue(6);
  m_ui->_size->setValue(300);
  m_ui->_agentSize->setValue(15);
  //connect signals and slots
  connect(m_ui->_maxSpeed, SIGNAL(valueChanged(int)), m_gl, SLOT(setMaxSpeed(int)));
  connect(m_ui->_maxSteering, SIGNAL(valueChanged(int)), m_gl, SLOT(setMaxSteering(int)));
  connect(m_ui->_alignment, SIGNAL(valueChanged(int)), m_gl, SLOT(setAlignmentStrength(int)));
  connect(m_ui->_cohesion, SIGNAL(valueChanged(int)), m_gl, SLOT(setCohesionStrength(int)));
  connect(m_ui->_seperation, SIGNAL(valueChanged(int)), m_gl, SLOT(setSeperationStrength(int)));
  connect(m_ui->_distance, SIGNAL(valueChanged(int)), m_gl, SLOT(setDesiredSeperation(int)));
  connect(m_ui->_radius, SIGNAL(valueChanged(int)), m_gl, SLOT(setNeighborhoodRadius(int)));
  connect(m_ui->_size, SIGNAL(valueChanged(int)), m_gl, SLOT(setFlockSize(int)));
  connect(m_ui->_edges, SIGNAL(stateChanged(int)), m_gl, SLOT(toggleEdgeBarrier()));
  connect(m_ui->_agentSize, SIGNAL(valueChanged(int)), m_gl, SLOT(setAgentSize(int)));
  connect(m_ui->_obsticles, SIGNAL(stateChanged(int)), m_gl, SLOT(toggleObsticles()));
}

MainWindow::~MainWindow()
{
  delete m_ui;
}
