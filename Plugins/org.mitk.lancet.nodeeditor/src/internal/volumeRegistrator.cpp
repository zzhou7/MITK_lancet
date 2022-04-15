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
#include "drrInterpolator.h"
#include "itkTimeProbesCollectorBase.h"
#include "twoprojectionregistration.h"
#include "volumeRegistrator.h"


//------------
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "mitkITKImageImport.h"
#include "mitkImageAccessByItk.h"
#include "mitkImageCast.h"
#include <itkShiftScaleImageFilter.h>
#include <mitkImageToItk.h>
#include <ITKOptimizer.h>
#include "vtkTransform.h"
#include "vtkSmartPointer.h"

class CommandIterationUpdate : public itk::Command
{
public:
	typedef CommandIterationUpdate Self;
	typedef itk::Command Superclass;
	typedef itk::SmartPointer<Self> Pointer;
	itkNewMacro(Self);

protected:
	CommandIterationUpdate() {};

public:
	typedef itk::PowellOptimizer OptimizerType;
	typedef const OptimizerType* OptimizerPointer;

	void Execute(itk::Object* caller, const itk::EventObject& event) { Execute((const itk::Object*)caller, event); }

	void Execute(const itk::Object* object, const itk::EventObject& event)
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

void VolumeRegistrator::link_drr1_cast(mitk::Image::ConstPointer inputImage)
{
	if (inputImage->GetDimension() != 3)
	{
		MITK_ERROR << "works only with 3D images, sorry.";
		// itkExceptionMacro("works only with 3D images, sorry.");
		return;
	}
	// AccessFixedDimensionByItk(inputImage.GetPointer(), drr1_cast, 3);
	mitk::CastToItkImage(inputImage, m_image_tmp1);

}

void VolumeRegistrator::link_drr2_cast(mitk::Image::ConstPointer inputImage)
{
	if (inputImage->GetDimension() != 3)
	{
		MITK_ERROR << "works only with 3D images, sorry.";
		// itkExceptionMacro("works only with 3D images, sorry.");
		return;
	}
	//AccessFixedDimensionByItk(inputImage.GetPointer(), drr2_cast, 3);
	mitk::CastToItkImage(inputImage, m_image_tmp2);
}

void VolumeRegistrator::link_3d_cast(mitk::Image::ConstPointer inputImage)
{
	if (inputImage->GetDimension() != 3)
	{
		MITK_ERROR << "works only with 3D images, sorry.";
		// itkExceptionMacro("works only with 3D images, sorry.");
		return;
	}
	//AccessFixedDimensionByItk(inputImage.GetPointer(), ct3d_cast, 3);
	mitk::CastToItkImage(inputImage, m_image3Df);
}

template <typename TPixel>
void VolumeRegistrator::drr1_cast(const itk::Image<TPixel, 3>* itkImage)
{
	typedef float InternalPixelType;
	typedef itk::Image<TPixel, 3> RawImageType;
	typedef itk::Image<InternalPixelType, 3> InternalImageType;

	typedef itk::CastImageFilter<RawImageType, InternalImageType> CastFilterType3D;

	CastFilterType3D::Pointer caster3D = CastFilterType3D::New();
	caster3D->SetInput(itkImage);
	caster3D->Update();

	m_image_tmp1 = caster3D->GetOutput();
}

template <typename TPixel>
void VolumeRegistrator::drr2_cast(const itk::Image<TPixel, 3>* itkImage)
{
	typedef float InternalPixelType;
	typedef itk::Image<TPixel, 3> RawImageType;
	typedef itk::Image<InternalPixelType, 3> InternalImageType;

	typedef itk::CastImageFilter<RawImageType, InternalImageType> CastFilterType3D;

	CastFilterType3D::Pointer caster3D = CastFilterType3D::New();
	caster3D->SetInput(itkImage);
	caster3D->Update();

	m_image_tmp2 = caster3D->GetOutput();
}

template <typename TPixel>
void VolumeRegistrator::ct3d_cast(const itk::Image<TPixel, 3>* itkImage)
{
	typedef float InternalPixelType;
	typedef itk::Image<TPixel, 3> RawImageType;
	typedef itk::Image<InternalPixelType, 3> InternalImageType;

	typedef itk::CastImageFilter<RawImageType, InternalImageType> CastFilterType3D;

	CastFilterType3D::Pointer caster3D = CastFilterType3D::New();
	caster3D->SetInput(itkImage);
	caster3D->Update();

	m_image3Df = caster3D->GetOutput();
}

void VolumeRegistrator::Clear() {}

VolumeRegistrator::VolumeRegistrator() = default;
VolumeRegistrator::~VolumeRegistrator() = default;

void VolumeRegistrator::registration()
{
	if (m_image3Df == nullptr || m_image_tmp1 == nullptr || m_image_tmp2 == nullptr)
	{
		MITK_ERROR << "SurfaceRegistration Error: Input not ready";
		return;
	}

  // Retrieve the initial euler transform from the matrix


	typedef float InternalPixelType;
	typedef itk::Image<InternalPixelType, 3> InternalImageType;
	typedef itk::Euler3DTransform<double> TransformType;
	typedef itk::PowellOptimizer OptimizerType;
	typedef itk::NormalizedCorrelationTwoImageToOneImageMetric<InternalImageType, InternalImageType> MetricType;
	// typedef itk::SiddonJacobsRayCastInterpolateImageFunction<InternalImageType, double> InterpolatorType;
  typedef itk::DrrInterpolator<InternalImageType, double> InterpolatorType;
	typedef itk::TwoProjectionImageRegistrationMethod<InternalImageType, InternalImageType> RegistrationType;


	MetricType::Pointer metric = MetricType::New();
	TransformType::Pointer transform = TransformType::New();
	OptimizerType::Pointer optimizer = OptimizerType::New();
	InterpolatorType::Pointer interpolator1 = InterpolatorType::New();
	InterpolatorType::Pointer interpolator2 = InterpolatorType::New();
	RegistrationType::Pointer registrator = RegistrationType::New();

	metric->ComputeGradientOff();
	metric->SetSubtractMean(true);

	registrator->SetMetric(metric);
	registrator->SetOptimizer(optimizer);
	registrator->SetTransform(transform);
	registrator->SetInterpolator1(interpolator1);
	registrator->SetInterpolator2(interpolator2);

	if (m_debug)
	{
		metric->DebugOn();
		// transform->DebugOn();
		// optimizer->DebugOn();
		interpolator1->DebugOn();
		interpolator2->DebugOn();
		// registration->DebugOn();
	}

	// To simply Siddon-Jacob's fast ray-tracing algorithm, we force the origin of the CT image
	// to be (0,0,0). Because we align the CT isocenter with the central axis, the projection
	// geometry is fully defined. The origin of the CT image becomes irrelavent.
	InternalImageType::PointType image3DOrigin;
	image3DOrigin[0] = 0.0;
	image3DOrigin[1] = 0.0;
	image3DOrigin[2] = 0.0;
	m_image3Df->SetOrigin(image3DOrigin);


	// set spacing for DRR 1 and DRR 2
	InternalImageType::SpacingType spacing;
	spacing[0] = m_sx_1;
	spacing[1] = m_sy_1;
	spacing[2] = 1.0;
	m_image_tmp1->SetSpacing(spacing);

	spacing[0] = m_sx_2;
	spacing[1] = m_sy_2;
	m_image_tmp2->SetSpacing(spacing);

	// Flip in y-direction for DRR 1 and DRR 2 (might be redundant ??)
	typedef itk::FlipImageFilter<InternalImageType> FlipFilterType;
	FlipFilterType::Pointer flipFilter1 = FlipFilterType::New();
	FlipFilterType::Pointer flipFilter2 = FlipFilterType::New();

	typedef FlipFilterType::FlipAxesArrayType FlipAxesArrayType;
	FlipAxesArrayType flipArray;
	flipArray[0] = 0;
	flipArray[1] = 0; // do not flip, because in the new drrGenerator, the resulted image is not flipped along y axis 
	flipArray[2] = 0;

	flipFilter1->SetFlipAxes(flipArray);
	flipFilter2->SetFlipAxes(flipArray);

	flipFilter1->SetInput(m_image_tmp1);
	flipFilter2->SetInput(m_image_tmp2);

	// The input 2D images may have 16 bits. We rescale the pixel value to between 0-255.
	typedef itk::RescaleIntensityImageFilter<InternalImageType, InternalImageType> Input2DRescaleFilterType;

	Input2DRescaleFilterType::Pointer rescaler2D1 = Input2DRescaleFilterType::New();
	rescaler2D1->SetOutputMinimum(0);
	rescaler2D1->SetOutputMaximum(255);
	rescaler2D1->SetInput(flipFilter1->GetOutput());

	Input2DRescaleFilterType::Pointer rescaler2D2 = Input2DRescaleFilterType::New();
	rescaler2D2->SetOutputMinimum(0);
	rescaler2D2->SetOutputMaximum(255);
	rescaler2D2->SetInput(flipFilter2->GetOutput());


	rescaler2D1->Update();
	rescaler2D2->Update();
	registrator->SetFixedImage1(rescaler2D1->GetOutput());
	registrator->SetFixedImage2(rescaler2D2->GetOutput());
	registrator->SetMovingImage(m_image3Df);
	// Initialise the transform
	// ~~~~~~~~~~~~~~~~~~~~~~~~

	// Set the order of the computation. Default ZXY
	transform->SetComputeZYX(true);

	// The transform is initialised with the translation [tx,ty,tz] and
	// rotation [rx,ry,rz] specified on the command line

	TransformType::OutputVectorType translation;

	// translation[0] = m_tx;
	// translation[1] = m_ty;
	// translation[2] = m_tz;
  translation[0] = m_ArrayMatrixWorldToCt[3];
  translation[1] = m_ArrayMatrixWorldToCt[7];
  translation[2] = m_ArrayMatrixWorldToCt[11];
  transform->SetTranslation(translation);

	// constant for converting degrees to radians
	const double dtr = (atan(1.0) * 4.0) / 180.0;
	Eigen::Matrix4d eigenMatrixWorldToCt{m_ArrayMatrixWorldToCt};
  eigenMatrixWorldToCt.transposeInPlace();
  double rx, ry, rz;
  // double piParameter = 180 / 3.1415926;
  // Problematic euler angle extraction
  // if (eigenMatrixWorldToCt(0, 2) < 1)
  // {
  //   if (eigenMatrixWorldToCt(0, 2) > -1)
  //   {
  //     ry = asin(eigenMatrixWorldToCt(0, 2));
  //     rx = atan2(-eigenMatrixWorldToCt(1, 2), eigenMatrixWorldToCt(2, 2));
  //     rz = atan2(-eigenMatrixWorldToCt(0, 1), eigenMatrixWorldToCt(0, 0));
  //   }
  //   else
  //   {
  //     ry = -3.1415926 / 2;
  //     rx = -atan2(eigenMatrixWorldToCt(1, 0), eigenMatrixWorldToCt(1, 1));
  //     rz = 0;
  //   }
  // }
  // else
  // {
  //   ry = 3.1415926 / 2;
  //   rx = atan2(eigenMatrixWorldToCt(1, 0), eigenMatrixWorldToCt(1, 1));
  //   rz = 0;
  // }
  // m_rx = rx;
  // m_ry = ry;
  // m_rz = rz;
  Eigen::Matrix3d eigenRotationMatrixWorldToCt;
  for (int i = 0; i < 3; i = i + 1)
  {
    for (int j = 0; i < 3; i = i + 1)
    {
      eigenRotationMatrixWorldToCt(i, j) = eigenMatrixWorldToCt(i, j);
    }
  }
  Eigen::Vector3d eulerAngles = eigenRotationMatrixWorldToCt.eulerAngles(
    2, 1, 0); // extrinsic rotation order: along X --> along Y --> along Z in radian
  m_rx = eulerAngles[2];
  m_ry = eulerAngles[1];
  m_rz = eulerAngles[0];

  transform->SetRotation(m_rx, m_ry, m_rz);
  TransformType::InputPointType worldOrigin;
  worldOrigin[0] = 0;
  worldOrigin[1] = 0;
  worldOrigin[2] = 0;
  transform->SetCenter(worldOrigin);

	// The centre of rotation is set by default to the centre of the 3D
	// volume but can be offset from this position using a command
	// line specified translation [cx,cy,cz]

	InternalImageType::PointType origin3D = m_image3Df->GetOrigin(); // might have some problem, since the image has be cast, the original code used uncast image
	const itk::Vector<double, 3> resolution3D = m_image3Df->GetSpacing();

	typedef InternalImageType::RegionType ImageRegionType3D;
	typedef ImageRegionType3D::SizeType SizeType3D;

	ImageRegionType3D region3D = m_image3Df->GetBufferedRegion();
	SizeType3D size3D = region3D.GetSize();

	TransformType::InputPointType isocenter;
	// isocenter[0] = m_cx + origin3D[0] + resolution3D[0] * static_cast<double>(size3D[0]) / 2.0;
	// isocenter[1] = m_cy + origin3D[1] + resolution3D[1] * static_cast<double>(size3D[1]) / 2.0;
	// isocenter[2] = m_cz + origin3D[2] + resolution3D[2] * static_cast<double>(size3D[2]) / 2.0;

  int MovingImagePixelNums[3]
  {
    static_cast<int>(size3D[0]),
    static_cast<int>(size3D[1]),
    static_cast<int>(size3D[2])
  };
  double MovingImageResolution[3]{resolution3D[0], resolution3D[1], resolution3D[2]};

	// transform->SetCenter(isocenter);

	if (m_verbose)
	{
		std::cout << "3D image size: " << size3D[0] << ", " << size3D[1] << ", " << size3D[2] << std::endl
			<< "   resolution: " << resolution3D[0] << ", " << resolution3D[1] << ", " << resolution3D[2] << std::endl
			<< "Transform: " << transform << std::endl;
	}

	// Set the origin of the 2D image
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// For correct (perspective) projection of the 3D volume, the 2D
	// image needs to be placed at a certain distance (the source-to-
	// isocenter distance {scd} ) from the focal point, and the normal
	// from the imaging plane to the focal point needs to be specified.
	//
	// By default, the imaging plane normal is set by default to the
	// center of the 2D image but may be modified from this using the
	// command line parameters [image1centerX, image1centerY,
	// image2centerX, image2centerY].

	double origin2D1[3];
	double origin2D2[3];

	// Note: Two 2D images may have different image sizes and pixel dimensions, although
	// scd are the same.

	const itk::Vector<double, 3> resolution2D1 = m_image_tmp1->GetSpacing();
	const itk::Vector<double, 3> resolution2D2 = m_image_tmp2->GetSpacing();

	typedef InternalImageType::RegionType ImageRegionType2D;
	typedef ImageRegionType2D::SizeType SizeType2D;

	ImageRegionType2D region2D1 = rescaler2D1->GetOutput()->GetBufferedRegion();
	ImageRegionType2D region2D2 = rescaler2D2->GetOutput()->GetBufferedRegion();
	SizeType2D size2D1 = region2D1.GetSize();
	SizeType2D size2D2 = region2D2.GetSize();
  
	origin2D1[0] = - m_RaySource1[0]; // m_o2Dx_1 - resolution2D1[0] * (size2D1[0] - 1.) / 2.;
  origin2D1[1] = - m_RaySource1[1]; // m_o2Dy_1 - resolution2D1[1] * (size2D1[1] - 1.) / 2.;
	origin2D1[2] = - m_RaySource1[2];

	rescaler2D1->GetOutput()->SetOrigin(origin2D1);

	origin2D2[0] = - m_RaySource2[0]; // m_o2Dx_1 - resolution2D2[0] * (size2D2[0] - 1.) / 2.;
  origin2D2[1] = - m_RaySource2[1]; // m_o2Dy_1 - resolution2D2[1] * (size2D2[1] - 1.) / 2.;
  origin2D2[2] = - m_RaySource2[2];

	rescaler2D2->GetOutput()->SetOrigin(origin2D2);

  // TODO: ROI is set here
	registrator->SetFixedImageRegion1(rescaler2D1->GetOutput()->GetBufferedRegion());
	registrator->SetFixedImageRegion2(rescaler2D2->GetOutput()->GetBufferedRegion());

	if (m_verbose)
	{
		std::cout << "2D image 1 size: " << size2D1[0] << ", " << size2D1[1] << ", " << size2D1[2] << std::endl
			<< "   resolution: " << resolution2D1[0] << ", " << resolution2D1[1] << ", " << resolution2D1[2]
			<< std::endl
			<< "   and position: " << origin2D1[0] << ", " << origin2D1[1] << ", " << origin2D1[2] << std::endl
			<< "2D image 2 size: " << size2D2[0] << ", " << size2D2[1] << ", " << size2D2[2] << std::endl
			<< "   resolution: " << resolution2D2[0] << ", " << resolution2D2[1] << ", " << resolution2D2[2]
			<< std::endl
			<< "   and position: " << origin2D2[0] << ", " << origin2D2[1] << ", " << origin2D2[2] << std::endl;
	}

	// Initialize the ray cast interpolator
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// The ray cast interpolator is used to project the 3D volume. It
	// does this by casting rays from the (transformed) focal point to
	// each (transformed) pixel coordinate in the 2D image.
	//
	// In addition a threshold may be specified to ensure that only
	// intensities greater than a given value contribute to the
	// projected volume. This can be used, for instance, to remove soft
	// tissue from projections of CT data and force the registration
	// to find a match which aligns bony structures in the images.

  	// Transform from Imager1 to Internal Ct
  auto vtkTransformImager1ToInternalCt = vtkSmartPointer<vtkTransform>::New();
  vtkTransformImager1ToInternalCt->PostMultiply();
  vtkTransformImager1ToInternalCt->Identity();
  vtkTransformImager1ToInternalCt->RotateX(-90);
  double translationImager1ToInternalCt[3]{m_RaySource1[0] - resolution3D[0] * static_cast<double>(size3D[0]) / 2.0,
                                           m_RaySource1[1] - resolution3D[2] * static_cast<double>(size3D[2]) / 2.0,
                                           resolution3D[1] * static_cast<double>(size3D[1]) / 2.0};
  vtkTransformImager1ToInternalCt->Translate(translationImager1ToInternalCt);
  vtkTransformImager1ToInternalCt->Update();

  // Calculate transform offset 1
  Eigen::Matrix4d eigenMatrixWorldToImager1{m_ArrayMatrixWorldToImager1};
  Eigen::Matrix4d eigenMatrixImager1ToInernalCt{vtkTransformImager1ToInternalCt->GetMatrix()->GetData()};
  eigenMatrixWorldToImager1.transposeInPlace();
  eigenMatrixImager1ToInernalCt.transposeInPlace();
  Eigen::Matrix4d eigenMatrixTransformOffset1 = (eigenMatrixWorldToImager1 * eigenMatrixImager1ToInernalCt).inverse();
  eigenMatrixTransformOffset1.transposeInPlace();

  double arrayTransformOffset1[16];
  for (int i = 0; i < 16; i = i + 1)
  {
    arrayTransformOffset1[i] = double(eigenMatrixTransformOffset1(i));
  }

  // Transform from Imager2 to Internal Ct
  auto vtkTransformImager2ToInternalCt = vtkSmartPointer<vtkTransform>::New();
  vtkTransformImager2ToInternalCt->PostMultiply();
  vtkTransformImager2ToInternalCt->Identity();
  vtkTransformImager2ToInternalCt->RotateX(-90);
  double translationImager2ToInternalCt[3]{m_RaySource2[0] - resolution3D[0] * static_cast<double>(size3D[0]) / 2.0,
                                           m_RaySource2[1] - resolution3D[2] * static_cast<double>(size3D[2]) / 2.0,
                                           resolution3D[1] * static_cast<double>(size3D[1]) / 2.0};
  vtkTransformImager2ToInternalCt->Translate(translationImager2ToInternalCt);
  vtkTransformImager2ToInternalCt->Update();

  // Calculate transform offset 2
  Eigen::Matrix4d eigenMatrixWorldToImager2{m_ArrayMatrixWorldToImager2};
  Eigen::Matrix4d eigenMatrixImager2ToInernalCt{vtkTransformImager2ToInternalCt->GetMatrix()->GetData()};
  eigenMatrixWorldToImager2.transposeInPlace();
  eigenMatrixImager2ToInernalCt.transposeInPlace();
  Eigen::Matrix4d eigenMatrixTransformOffset2 = (eigenMatrixWorldToImager2 * eigenMatrixImager2ToInernalCt).inverse();
  eigenMatrixTransformOffset2.transposeInPlace();

  double arrayTransformOffset2[16];
  for (int i = 0; i < 16; i = i + 1)
  {
    arrayTransformOffset2[i] = double(eigenMatrixTransformOffset2(i));
  }

	// 2D Image 1
	// interpolator1->SetProjectionAngle(dtr * m_angleDRR1);
	interpolator1->SetFocalPointToIsocenterDistance(m_RaySource1[2]);
	interpolator1->SetThreshold(m_threshold);
	interpolator1->SetTransform(transform);
  interpolator1->SetMovingImageSpacing(MovingImageResolution);
  interpolator1->SetMovingImagePixelNumbers(MovingImagePixelNums);
  interpolator1->SetArrayTransformOffset(arrayTransformOffset1);
	interpolator1->Initialize();

	// 2D Image 2
	// interpolator2->SetProjectionAngle(dtr * m_angleDRR2);
  interpolator2->SetFocalPointToIsocenterDistance(m_RaySource2[2]);
	interpolator2->SetThreshold(m_threshold);
	interpolator2->SetTransform(transform);
  interpolator2->SetMovingImageSpacing(MovingImageResolution);
  interpolator2->SetMovingImagePixelNumbers(MovingImagePixelNums);
  interpolator2->SetArrayTransformOffset(arrayTransformOffset2);
	interpolator2->Initialize();

	// Set up the transform and start position
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// The registration start position is intialised using the
	// transformation parameters.

	registrator->SetInitialTransformParameters(transform->GetParameters());

	// We wish to minimize the negative normalized correlation similarity measure.

	// optimizer->SetMaximize( true );  // for GradientDifferenceTwoImageToOneImageMetric
	optimizer->SetMaximize(false); // for NCC Normalized Cross Correlation

	if (m_switchOffOptimizer)
	{
		optimizer->SetMaximumIteration(0);
		optimizer->SetMaximumLineIteration(0);
		optimizer->SetStepLength(0);
		optimizer->SetStepTolerance(10);
		optimizer->SetValueTolerance(1);
	}
	else
	{
		optimizer->SetMaximumIteration(10);
		optimizer->SetMaximumLineIteration(4); // for Powell's method
		optimizer->SetStepLength(4);
		optimizer->SetStepTolerance(0.02);
		optimizer->SetValueTolerance(0.001);
	}


	// The optimizer weightings are set such that one degree equates to
	// one millimeter.

	itk::Optimizer::ScalesType weightings(transform->GetNumberOfParameters());

	weightings[0] = 1. / dtr;
	weightings[1] = 1. / dtr;
	weightings[2] = 1. / dtr;
	weightings[3] = 1.;
	weightings[4] = 1.;
	weightings[5] = 1.;

	optimizer->SetScales(weightings);

	if (m_verbose)
	{
		optimizer->Print(std::cout);
	}

	// Create the observers
	// ~~~~~~~~~~~~~~~~~~~~

	CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();

	optimizer->AddObserver(itk::IterationEvent(), observer);

	// Create a timer to record calculation time.
	itk::TimeProbesCollectorBase timer;

	if (m_verbose)
	{
		std::cout << "Starting the registration now..." << std::endl;
	}

	try
	{
		timer.Start("Registration");
		// Start the registration.
		registrator->StartRegistration();
		timer.Stop("Registration");
	}
	catch (itk::ExceptionObject & err)
	{
		std::cout << "ExceptionObject caught !" << std::endl;
		std::cout << err << std::endl;
		return;
	}

	typedef RegistrationType::ParametersType ParametersType;

  // Error here !!
	ParametersType finalParameters = registrator->GetLastTransformParameters();

	const double RotationAlongX = finalParameters[0] / dtr; // Convert radian to degree
	const double RotationAlongY = finalParameters[1] / dtr;
	const double RotationAlongZ = finalParameters[2] / dtr;
	const double TranslationAlongX = finalParameters[3];
	const double TranslationAlongY = finalParameters[4];
	const double TranslationAlongZ = finalParameters[5];

	m_RX = RotationAlongX;
	m_RY = RotationAlongY;
	m_RZ = RotationAlongZ;
	m_TX = TranslationAlongX;
	m_TY = TranslationAlongY;
	m_TZ = TranslationAlongZ;

	const int numberOfIterations = optimizer->GetCurrentIteration();

	const double bestValue = optimizer->GetValue();
	m_metric = bestValue;
	std::cout << "Result = " << std::endl;
	std::cout << " Rotation Along X = " << RotationAlongX << " deg" << std::endl;
	std::cout << " Rotation Along Y = " << RotationAlongY << " deg" << std::endl;
	std::cout << " Rotation Along Z = " << RotationAlongZ << " deg" << std::endl;
	std::cout << " Translation X = " << TranslationAlongX << " mm" << std::endl;
	std::cout << " Translation Y = " << TranslationAlongY << " mm" << std::endl;
	std::cout << " Translation Z = " << TranslationAlongZ << " mm" << std::endl;
	std::cout << " Number Of Iterations = " << numberOfIterations << std::endl;
	std::cout << " Metric value  = " << bestValue << std::endl;

	timer.Report();

}

void VolumeRegistrator::SetRaySource1(double array[3])
{
  for (int i = 0; i < 3; i = i + 1)
  {
    m_RaySource1[i] = array[i];
  }
}

void VolumeRegistrator::SetRaySource2(double array[3])
{
  for (int i = 0; i < 3; i = i + 1)
  {
    m_RaySource2[i] = array[i];
  }
}

void VolumeRegistrator::SetArrayMatrixWorldToImager1(double array[16])
{
  for (int i = 0; i < 16; i = i + 1)
  {
    m_ArrayMatrixWorldToImager1[i] = array[i];
  }
}

void VolumeRegistrator::SetArrayMatrixWorldToImager2(double array[16])
{
  for (int i = 0; i < 16; i = i + 1)
  {
    m_ArrayMatrixWorldToImager2[i] = array[i];
  }
}

void VolumeRegistrator::SetArrayMatrixWorldToCt(double array[16])
{
  for (int i = 0; i < 16; i = i + 1)
  {
    m_ArrayMatrixWorldToCt[i] = array[i];
  }
}
