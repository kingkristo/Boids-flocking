//adapted from QT NGL demo https://github.com/NCCA/QtNGL
#ifndef NGLSCENE_H_
#define NGLSCENE_H_
#include <ngl/Transformation.h>
#include <ngl/Vec3.h>
#include <ngl/Mat4.h>
#include <ngl/Text.h>
#include "WindowParams.h"
#include <QEvent>
#include <QResizeEvent>
#include <QOpenGLWidget>
#include <memory>
#include "Flock.h"
#include <ngl/BBox.h>

class NGLScene : public QOpenGLWidget
{
  Q_OBJECT // must include this if you use Qt signals/slots
      public :
      /// @brief Constructor for GLWindow
      //----------------------------------------------------------------------------------------------------------------------
      /// @brief Constructor for GLWindow
      /// @param [in] _parent the parent window to create the GL context in
      //----------------------------------------------------------------------------------------------------------------------
      NGLScene(QWidget *_parent);

  /// @brief dtor
  ~NGLScene() override;
public slots:
  void setMaxSpeed(int _maxSpeed);
  void setMaxSteering(int _maxSteering);
  void setAlignmentStrength(int _alignment);
  void setCohesionStrength(int _cohesion);
  void setSeperationStrength(int _seperation);
  void setDesiredSeperation(int _distance);
  void setNeighborhoodRadius(int _radius);
  void setFlockSize(int _size);
  void toggleEdgeBarrier();
  void setAgentSize(int _scale);
  void toggleObsticles();

private:
  std::unique_ptr<Flock> m_flock;
  void timerEvent(QTimerEvent *_timer) override;
  std::unique_ptr<ngl::BBox> m_bbox;
  std::unique_ptr<ngl::BBox> m_obsticle[4];
  bool m_obsticles = false;
  std::chrono::high_resolution_clock::time_point m_previousTime;


protected:
  //----------------------------------------------------------------------------------------------------------------------
  /// @brief the windows params such as mouse and rotations etc
  //----------------------------------------------------------------------------------------------------------------------
  WinParams m_win;
  /// @brief  The following methods must be implimented in the sub class
  /// this is called when the window is created
  void initializeGL() override;

  /// @brief this is called whenever the window is re-sized
  /// @param[in] _w the width of the resized window
  /// @param[in] _h the height of the resized window
  void resizeGL(int _w, int _h) override;
  /// @brief this is the main gl drawing routine which is called whenever the window needs to
  // be re-drawn
  void paintGL() override;

  /// @brief our model position
  ngl::Vec3 m_modelPos;
  ngl::Mat4 m_mouseGlobalTX;
  /// @brief our camera
  ngl::Mat4 m_view;
  ngl::Mat4 m_project;
  /// @brief our transform for objects
  ngl::Transformation m_transform;

private:
  /// @brief this method is called every time a mouse is moved
  /// @param _event the Qt Event structure

  void mouseMoveEvent(QMouseEvent *_event) override;
  /// @brief this method is called everytime the mouse button is pressed
  /// inherited from QObject and overridden here.
  /// @param _event the Qt Event structure

  void mousePressEvent(QMouseEvent *_event) override;

  /// @brief this method is called everytime the mouse button is released
  /// inherited from QObject and overridden here.
  /// @param _event the Qt Event structure
  void mouseReleaseEvent(QMouseEvent *_event) override;
  void wheelEvent(QWheelEvent *_event) override;

  void loadMatricesToShader();
};

#endif
