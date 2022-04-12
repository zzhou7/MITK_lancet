#ifndef POLISH_H
#define POLISH_H

#include <itkObject.h>
#include <itkCommand.h>
#include <mitkCommon.h>
#include "MitkLancetGeoUtilExports.h"
#include "mitkImage.h"
#include "mitkSurface.h"
#include "mitkSurfaceToImageFilter.h"
#include <vtkDiscreteFlyingEdges3D.h>
#include <vtkWindowedSincPolyDataFilter.h>

class Timer
{
public:
  Timer();
  Timer(std::string name);
  ~Timer();
private:
  clock_t m_time_start{};
  float m_time_past{0.0};
  std::string m_name{"timer"};
};

class MITKLANCETGEOUTIL_EXPORT Polish : public itk::Command
{
public:
  mitkClassMacroItkParent(Polish, itk::Command);
  itkNewMacro(Self)

  enum PolishState { Setup, Ready, Polishing };

  void Execute(Object *caller, const itk::EventObject &event) override;
  void Execute(const Object *caller, const itk::EventObject &event) override;
  Polish();
  ~Polish() override;

  /**
     * \brief  change object state
     */
  void SetState(PolishState state);

  PolishState GetState() const;
  /**
   * @brief Starts the polishing.
   * @return Returns true if the polishing is started. Returns false if there was an error.
   */
  bool StartPolishing();
  /**
   * @brief Stop the polishing.
   * @return Returns true if the polishing is stopped. Returns false if there was an error.
   */
  bool StopPolishing();

  void RestorePolish();

  
  /**
   * @brief The actual time-consuming algorithm.
   *  first turn tool surface into stencil to cut the bone image,
   *  then extract and smooth the iso surface from it to get res bone surface
   * 
   */
  void PolishWorkflow();
  /**
   * @brief other way. TEST ONLY
   */
  void PolishWorkflow2();
  /**
   * \brief static start method for the work thread.
   */
  static ITK_THREAD_RETURN_TYPE ThreadStartPolishing(void* pInfoStruct);


  //helper func
  static void CopyVolume(mitk::Image::Pointer dist, mitk::Image::Pointer src);

  mitk::Surface::Pointer DiscreteFlyingEdges3D(mitk::Image *mitkImage);
  mitk::Image::Pointer SurfaceCutImage(mitk::Surface *surface,
                                       mitk::Image *image,
                                       bool isReverseStencil,
                                       bool binary
    );
  
  itkGetMacro(toolSurface, mitk::Surface::Pointer);
  itkGetMacro(boneImage, mitk::Image::Pointer);
  itkSetMacro(toolSurface, mitk::Surface::Pointer);
  void SetboneImage(mitk::Image::Pointer boneImage);
  itkGetMacro(boneSurface, mitk::Surface::Pointer);
  void SetboneSurface(mitk::Surface::Pointer boneSurface);
  itkGetMacro(boneSurface_polished, mitk::Surface::Pointer);
  itkGetMacro(boneImage_polished, mitk::Image::Pointer);
 
  
private:
  /**
   * \brief doPolish() continuously do the actual time-consuming algorithm in PolishWorkflow() for polishing until StopPolish is
   * called. This function is executed by the polishing thread (through StartPolish() and ThreadStartPolishing()). It
   * should not be called directly.
   */
  void doPolish();
  //input
  mitk::Surface::Pointer m_toolSurface{nullptr};

  /**
   * \brief Pointer to the bone image.
   *
   * @attention modify m_boneImage will effect origin pointer directly
   */
  mitk::Image::Pointer m_boneImage{nullptr};

  /**
   * \brief Pointer to the bone surface.
   *
   * @attention modify m_boneSurface will effect origin pointer directly
   */
  mitk::Surface::Pointer m_boneSurface{nullptr};
  /**
   * \brief Pointer to the bone Surface.
   *  Copy from input boneSurface, Polish actual apply to it;
   * @attention modify this will not effect origin pointer
   */
  mitk::Surface::Pointer m_boneSurface_polished{nullptr};
  /**
   * \brief Pointer to the bone Image.
   *  Copy from input boneSurface, Polish actual apply to it;
   * @attention modify this will not effect origin pointer
   */
  mitk::Image::Pointer m_boneImage_polished{nullptr};
  //filter
  vtkSmartPointer<vtkDiscreteFlyingEdges3D> m_flyingEdgeFilter{nullptr};
  vtkSmartPointer<vtkWindowedSincPolyDataFilter> m_wsFilter{nullptr};
  mitk::SurfaceToImageFilter::Pointer m_surface2imagefilter{nullptr};
  //vtkSmartPointer<vtkPolyData> m_Femur_PolyData;

  ///< creates tracking thread that continuously do workflow for new data
  itk::MultiThreader::Pointer m_MultiThreader;
  int m_ThreadID;

  PolishState m_State; ///< current object state (Setup, Ready, Polishing)

  bool m_StopPolish; ///< signal stop to polish thread
  itk::FastMutexLock::Pointer m_StopPolishMutex; ///< mutex to control access to m_StopTracking
  itk::FastMutexLock::Pointer m_PolishFinishedMutex; ///< mutex to manage control flow of StopPolishing()
  itk::FastMutexLock::Pointer m_StateMutex; ///< mutex to control access to m_State
};

#endif
