#include "itkCastImageFilter.h"
#include "itkCommand.h"
#include "itkNormalizedCorrelationTwoImageToOneImageMetric.h"
#include "itkPowellOptimizer.h"
#include "itkTwoProjectionImageRegistrationMethod.h"

#include "itkEuler3DTransform.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRayCastInterpolateImageFunction.h"
// --------
#include "itkFlipImageFilter.h"
#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkSiddonJacobsRayCastInterpolateImageFunction.h"
#include "itkTimeProbesCollectorBase.h"
// #include "twoprojectionregistration.h"


//------------
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "mitkITKImageImport.h"
#include "mitkImageAccessByItk.h"
#include "mitkImageCast.h"
#include <itkShiftScaleImageFilter.h>
#include <mitkImageToItk.h>
#include <ITKOptimizer.h>

class CommandIterationUpdate : public itk::Command
{
public:
  typedef CommandIterationUpdate Self;
  typedef itk::Command Superclass;
  typedef itk::SmartPointer<Self> Pointer;
  itkNewMacro(Self);

protected:
  CommandIterationUpdate(){};

public:
  typedef itk::PowellOptimizer OptimizerType;
  typedef const OptimizerType *OptimizerPointer;

  void Execute(itk::Object *caller, const itk::EventObject &event) { Execute((const itk::Object *)caller, event); }

  void Execute(const itk::Object *object, const itk::EventObject &event)
  {
    OptimizerPointer optimizer = dynamic_cast<OptimizerPointer>(object);
    if (typeid(event) != typeid(itk::IterationEvent))
    {
      return;
    }
    //    std::cout << "Iteration: " << optimizer->GetCurrentIteration() << std::endl;
    std::cout << "Similarity: " << optimizer->GetValue() << std::endl;
    std::cout << "Position: " << optimizer->GetCurrentPosition() << std::endl;
  }
};