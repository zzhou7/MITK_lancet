#include "itkCenteredEuler3DTransform.h"
#include "itkEuler3DTransform.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRayCastInterpolateImageFunction.h"
// --------
#include "itkSiddonJacobsRayCastInterpolateImageFunction.h"
#include "itkTimeProbesCollectorBase.h"
#include "drrGenerator.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkFlipImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageDuplicator.h"

//------------
#include "itkResampleImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "mitkImageAccessByItk.h"
#include "mitkImageCast.h"
#include "mitkITKImageImport.h"
#include <itkShiftScaleImageFilter.h>
#include <mitkImageToItk.h>
#include <vtkSmartPointer.h>
#include <eigen3/Eigen/Eigen>
#include "drrInterpolator.h"
#include "spineDrrInterpolator.h"

template <typename TPixel, unsigned VDimension>
void DrrGenerator::ItkImageProcessing(const itk::Image<TPixel, VDimension>* itkImage_0)
{

	// Although we generate a 2D projection of the 3D volume for the
	// purposes of the interpolator both images must be three dimensional.

	typedef itk::Image<TPixel, VDimension> InputImageType;
	typedef itk::Image<TPixel, VDimension> OutputImageType;

	//duplicate a const input so that we can change its content

	typedef itk::ImageDuplicator<InputImageType> DupType;
	typename DupType::Pointer duplicator = DupType::New();
	duplicator->SetInputImage(itkImage_0);
	duplicator->Update();
	typename InputImageType::Pointer itkImage = duplicator->GetOutput();

	// To simply Siddon-Jacob's fast ray-tracing algorithm, we force the origin of the CT image
	// to be (0,0,0). Because we align the CT center with the central axis, the projection
	// geometry is fully defined. The origin of the CT image becomes irrelavent.
	typename InputImageType::PointType ctOrigin;
	ctOrigin[0] = 0.0;
	ctOrigin[1] = 0.0;
	ctOrigin[2] = 0.0;
	//ctOrigin.Fill(0.0);
	itkImage->SetOrigin(ctOrigin);



	// Print out the details of the input volume
	//DONE
	if (m_verbose)
	{
		unsigned int i;
		const typename InputImageType::SpacingType spacing = itkImage->GetSpacing();
		std::cout << std::endl << "Input ";

		typename InputImageType::RegionType region = itkImage->GetBufferedRegion();
		region.Print(std::cout);

		std::cout << "  Resolution: [";
		for (i = 0; i < VDimension; i++)
		{
			std::cout << spacing[i];
			if (i < VDimension - 1)
				std::cout << ", ";
		}
		std::cout << "]" << std::endl;

		const typename InputImageType::PointType origin = itkImage->GetOrigin();
		std::cout << "  Origin: [";
		for (i = 0; i < VDimension; i++)
		{
			std::cout << origin[i];
			if (i < VDimension - 1)
				std::cout << ", ";
		}
		std::cout << "]" << std::endl << std::endl;
	}

	// Creation of a {ResampleImageFilter} enables coordinates for
	// each of the pixels in the DRR image to be generated. These
	// coordinates are used by the \code{RayCastInterpolateImageFunction}
	// to determine the equation of each corresponding ray which is cast
	// through the input volume.

	typedef itk::ResampleImageFilter<InputImageType, InputImageType> FilterType;

	typename FilterType::Pointer filter = FilterType::New();

	filter->SetInput(itkImage);  // CHECKED
	filter->SetDefaultPixelValue(0);  // CHECKED

	// An Euler transformation is defined to position the input volume.
	// The {ResampleImageFilter} uses this transform to position the
	// output DRR image for the desired view.

	// typedef itk::CenteredEuler3DTransform<double> TransformType; // MODIFIED
	typedef itk::Euler3DTransform<double> TransformType; // MODIFIED

	// Define the transform as the transform of
	// the axes from World coordinate to the CT volume coordinate-- zzhou 31/31/2022
	TransformType::Pointer transform = TransformType::New(); // CHECKED

	transform->SetComputeZYX(true); // CHECKED

	TransformType::OutputVectorType translation; // CHECKED

	// translation[0] = m_tx; // CHECKED
	// translation[1] = m_ty; // CHECKED
	// translation[2] = m_tz; // CHECKED
	//
	// // constant for converting degrees into radians
	const double dtr = (std::atan(1.0) * 4.0) / 180.0; // CHECKED
	
	// transform->SetTranslation(translation); // CHECKED
	// transform->SetRotation(dtr * m_rx, dtr * m_ry, dtr * m_rz); // CHECKED
	translation[0] = m_ArrayMatrixWorldToCt[3];
	translation[1] = m_ArrayMatrixWorldToCt[7];
	translation[2] = m_ArrayMatrixWorldToCt[11];
	transform->SetTranslation(translation);

	Eigen::Matrix4d eigenMatrixWorldToCt{ m_ArrayMatrixWorldToCt };
	eigenMatrixWorldToCt.transposeInPlace();
	double rx, ry, rz;
	// double piParameter = 180 / 3.1415926;
	if (eigenMatrixWorldToCt(0, 2) < 1)
	{
		if (eigenMatrixWorldToCt(0, 2) > -1)
		{
			ry = asin(eigenMatrixWorldToCt(0, 2));
			rx = atan2(-eigenMatrixWorldToCt(1, 2), eigenMatrixWorldToCt(2, 2));
			rz = atan2(-eigenMatrixWorldToCt(0, 1), eigenMatrixWorldToCt(0, 0));
		}
		else
		{
			ry = -3.1415926 / 2;
			rx = -atan2(eigenMatrixWorldToCt(1, 0), eigenMatrixWorldToCt(1, 1));
			rz = 0;
		}
	}
	else
	{
		ry = 3.1415926 / 2;
		rx = atan2(eigenMatrixWorldToCt(1, 0), eigenMatrixWorldToCt(1, 1));
		rz = 0;
	}
	m_rx = rx ;
	m_ry = ry ;
	m_rz = rz ;
	transform->SetRotation( m_rx,  m_ry,  m_rz);
	TransformType::InputPointType worldOrigin;
	worldOrigin[0] = 0;
	worldOrigin[1] = 0;
	worldOrigin[2] = 0;
	transform->SetCenter(worldOrigin);

	typename InputImageType::PointType imOrigin = itkImage->GetOrigin(); // CHECKED
	typename InputImageType::SpacingType imRes = itkImage->GetSpacing(); // CHECKED

	typedef typename InputImageType::RegionType InputImageRegionType; // CHECKED
	typedef typename InputImageRegionType::SizeType InputImageSizeType; // CHECKED

	InputImageRegionType imRegion = itkImage->GetBufferedRegion(); // CHECKED
	InputImageSizeType imSize = imRegion.GetSize(); // CHECKED
	int ctPixelNumbers[3]{
		static_cast<int>(imSize[0]),
		static_cast<int>(imSize[1]),
		static_cast<int>(imSize[2])
	};
	double ctResolution[3]
	{
		static_cast<double>(imRes[0]),
		static_cast<double>(imRes[1]),
		static_cast<double>(imRes[2])
	};

  // isocenter is also the image rotation center, the rotation center and isocenter cannot be defined
  // separately coz the Siddon intetrpolator catches the isocenter from the image rotation center
	// imOrigin[0] += imRes[0] * static_cast<double>(imSize[0]) / 2.0; // ?
	// imOrigin[1] += imRes[1] * static_cast<double>(imSize[1]) / 2.0; // ?
	// imOrigin[2] += imRes[2] * static_cast<double>(imSize[2]) / 2.0; // ?

	TransformType::InputPointType isocenter; // ?
	isocenter[0] = m_cx + imOrigin[0] + imRes[0] * static_cast<double>(imSize[0]) / 2.0; // ?
	isocenter[1] = m_cy + imOrigin[1] + imRes[1] * static_cast<double>(imSize[1]) / 2.0; // ?
	isocenter[2] = m_cz + imOrigin[2] + imRes[2] * static_cast<double>(imSize[2]) / 2.0; // ?


	// the original twoProject code
	// image rotation center is the isocenter, which aligns with the source 
	// if (customized_iso)
	// {
	//   // Isocenter location given by the user.
	//   isocenter[0] = imOrigin[0] + imRes[0] * cx;
	//   isocenter[1] = imOrigin[1] + imRes[1] * cy;
	//   isocenter[2] = imOrigin[2] + imRes[2] * cz;
	// }
	// else
	// {
	//   // Set the center of the image as the isocenter.
	//   isocenter[0] = imOrigin[0] + imRes[0] * static_cast<double>(imSize[0]) / 2.0;
	//   isocenter[1] = imOrigin[1] + imRes[1] * static_cast<double>(imSize[1]) / 2.0;
	//   isocenter[2] = imOrigin[2] + imRes[2] * static_cast<double>(imSize[2]) / 2.0;
	// }

	

	if (m_verbose) // CHECKED
	{
		std::cout << "Image size: "
			<< imSize[0] << ", " << imSize[1] << ", " << imSize[2]
			<< std::endl << "   resolution: "
			<< imRes[0] << ", " << imRes[1] << ", " << imRes[2]
			<< std::endl << "   origin: "
			<< imOrigin[0] << ", " << imOrigin[1] << ", " << imOrigin[2]
			<< std::endl << "   Isocenter: "
			<< isocenter[0] << ", " << isocenter[1] << ", " << isocenter[2]
			<< std::endl << "Transform: " << transform << std::endl;
	}

	// The \code{RayCastInterpolateImageFunction} is instantiated and passed the transform
	// object. The \code{RayCastInterpolateImageFunction} uses this
	// transform to reposition the x-ray source such that the DRR image
	// and x-ray source move as one around the input volume. This coupling
	// mimics the rigid geometry of the x-ray gantry.

	// typedef itk::RayCastInterpolateImageFunction<InputImageType, double>
	//     InterpolatorType;
	// typedef itk::SiddonJacobsRayCastInterpolateImageFunction<InputImageType, double> InterpolatorType; // MODIFIED
	typedef itk::DrrInterpolator<InputImageType, double> InterpolatorType;
  // typedef itk::SpineDrrInterpolator<InputImageType, double> InterpolatorType;

	typename InterpolatorType::Pointer interpolator = InterpolatorType::New(); // MODIFIED
	interpolator->SetTransform(transform); // MODIFIED

	// Set angle between projection central axis and -z axis
	// interpolator->SetProjectionAngle(dtr * m_rprojection);  //MODIFIED

	// We can then specify a threshold above which the volume's
	// intensities will be integrated.
	m_scd = m_RaySource[2];
	interpolator->SetThreshold(m_threshold); // MODIFIED
	interpolator->SetFocalPointToIsocenterDistance(m_scd); // MODIFIED
	interpolator->SetMovingImageSpacing(ctResolution);
	interpolator->SetMovingImagePixelNumbers(ctPixelNumbers);

	// Transform from Imager to Internal Ct
	auto vtkTransformImagerToInternalCt = vtkSmartPointer<vtkTransform>::New();
	vtkTransformImagerToInternalCt->PostMultiply();
	vtkTransformImagerToInternalCt->Identity();
	vtkTransformImagerToInternalCt->RotateX(-90);
	double translationImagerToInternalCt[3]
	{
		m_RaySource[0] - imRes[0] * static_cast<double>(imSize[0]) / 2.0,
		m_RaySource[1] - imRes[2] * static_cast<double>(imSize[2]) / 2.0,
		imRes[1] * static_cast<double>(imSize[1]) / 2.0
	};
	vtkTransformImagerToInternalCt->Translate(translationImagerToInternalCt);
	vtkTransformImagerToInternalCt->Update();

	// Calculate transform offset
	Eigen::Matrix4d eigenMatrixWorldToImager{ m_ArrayMatrixWorldToImager };
	Eigen::Matrix4d eigenMatrixImagerToInernalCt{ vtkTransformImagerToInternalCt->GetMatrix()->GetData() };
	eigenMatrixWorldToImager.transposeInPlace();
	eigenMatrixImagerToInernalCt.transposeInPlace();
	Eigen::Matrix4d eigenMatrixTransformOffset = (eigenMatrixWorldToImager * eigenMatrixImagerToInernalCt).inverse();
	eigenMatrixTransformOffset.transposeInPlace();

	double arrayTransformOffset[16];
	for (int i = 0; i < 16; i = i + 1)
	{
		arrayTransformOffset[i] = double(eigenMatrixTransformOffset(i));
	}
	
	interpolator->SetArrayTransformOffset(arrayTransformOffset);

	// The ray-cast interpolator needs to know the initial position of the
	// ray source or focal point. In this example we place the input
	// volume at the origin and halfway between the ray source and the
	// screen. The distance between the ray source and the screen
	// is the "source to image distance" \code{sid} and is specified by
	// the user.

	// typename InterpolatorType::InputPointType focalpoint;
	//
	// focalpoint[0] = imOrigin[0]; // MODIFIED
	// focalpoint[1] = imOrigin[1]; // MODIFIED
	// focalpoint[2] = imOrigin[2] - 1024 / 2.; // MODIFIED

	// interpolator->SetFocalPoint(focalpoint); // MODIFIED

	// if (m_verbose) // MODIFIED
	// {
	//     std::cout << "Focal Point: "
	//         << focalpoint[0] << ", "
	//         << focalpoint[1] << ", "
	//         << focalpoint[2] << std::endl;
	// }

	// Having initialised the interpolator we pass the object to the resample filter.

	interpolator->Initialize(); // MODIFIED ??
	interpolator->Print(std::cout);

	filter->SetInterpolator(interpolator);
	// filter->SetTransform(transform); // MODIFIED ??

	// The size and resolution of the output DRR image is specified via the resample filter.
	// setup the scene
	typename InputImageType::SizeType size; // CHECKED

	size[0] = m_dx; // number of pixels along X of the 2D DRR image // CHECKED
	size[1] = m_dy; // number of pixels along Y of the 2D DRR image // CHECKED
	size[2] = 1;  // only one slice // CHECKED

	filter->SetSize(size); // CHECKED

	typename InputImageType::SpacingType spacing; // CHECKED

	spacing[0] = m_im_sx;  // pixel spacing along X of the 2D DRR image [mm]  //MODIFIED 
	spacing[1] = m_im_sy;  // pixel spacing along Y of the 2D DRR image [mm]  //MODIFIED
	spacing[2] = 1.0; // slice thickness of the 2D DRR image [mm] //CHECKED 

	filter->SetOutputSpacing(spacing); // CHECKED

	if (m_verbose) // CHECKED
	{
		std::cout << "Output image size: "
			<< size[0] << ", "
			<< size[1] << ", "
			<< size[2] << std::endl;

		std::cout << "Output image spacing: "
			<< spacing[0] << ", "
			<< spacing[1] << ", "
			<< spacing[2] << std::endl;
	}

	// In addition the position of the DRR is specified. The default
	// position of the input volume, prior to its transformation is
	// half-way between the ray source and screen and unless specified
	// otherwise the normal from the "screen" to the ray source passes
	// directly through the centre of the DRR.

	double origin[VDimension]; // CHECKED

	// origin[0] = m_o2Dx - m_im_sx * ((double)m_dx - 1.) / 2.; // MODIFIED 
	// origin[1] = m_o2Dy - m_im_sy * ((double)m_dy - 1.) / 2.; // MODIFIED
	origin[0] = -(m_RaySource[0]);
	origin[1] = -(m_RaySource[1]);
	origin[2] = -m_scd;                                       // MODIFIED
	// origin[2] =  0;

	// origin[0] = -m_im_sx * m_o2Dx; // MODIFIED
	// origin[1] = -m_im_sy * m_o2Dy; // MODIFIED
	// origin[2] = -m_scd; // MODIFIED


	filter->SetOutputOrigin(origin); // CHECKED

	if (m_verbose) // CHECKED
	{
		std::cout << "Output image origin: "
			<< origin[0] << ", "
			<< origin[1] << ", "
			<< origin[2] << std::endl;
	}


	filter->Update();

	// flipping and rescaling haven't been touched yet

	// typedef itk::RescaleIntensityImageFilter<
	//     InputImageType, InputImageType> RescaleFilterType;
	// typename RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	// rescaler->SetOutputMinimum(0);
	// rescaler->SetOutputMaximum(255);
	// rescaler->SetInput(filter->GetOutput());
	// rescaler->Update();

	typedef itk::RescaleIntensityImageFilter<InputImageType, OutputImageType> RescaleFilterType;
	RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	rescaler->SetOutputMinimum(0);
	rescaler->SetOutputMaximum(255);
	rescaler->SetInput(filter->GetOutput());

	rescaler->Update();

	// Out of some reason, the computed projection is upsided-down.
	// Here we use a FilpImageFilter to flip the images in y direction.
	typedef itk::FlipImageFilter<OutputImageType> FlipFilterType;
	FlipFilterType::Pointer flipFilter = FlipFilterType::New();

	typedef FlipFilterType::FlipAxesArrayType FlipAxesArrayType;
	FlipAxesArrayType flipArray;
	flipArray[0] = 0;
	flipArray[1] = 1; // zzhou: this flipping is actually not required in the MITK environment??
	flipArray[2] = 0; // tricky, this line cannot be missed

	flipFilter->SetFlipAxes(flipArray);
	flipFilter->SetInput(rescaler->GetOutput());
	flipFilter->Update();


	// get  Pointer to output image
	mitk::Image::Pointer resultImage = this->GetOutput();
	// write into output image
	// mitk::CastToMitkImage(flipFilter->GetOutput(), resultImage); // zzhou: this flipping is actually not required in the MITK environment??
	mitk::CastToMitkImage(rescaler->GetOutput(), resultImage);
	//m_out = mitk::ImportItkImage(rescaler->GetOutput())->Clone();
}

DrrGenerator::DrrGenerator() = default;

DrrGenerator::~DrrGenerator() = default;

void DrrGenerator::GenerateOutputInformation()
{
	mitk::Image::Pointer inputImage = (mitk::Image*)this->GetInput();
	mitk::Image::Pointer output = this->GetOutput();
	itkDebugMacro(<< "GenerateOutputInformation()");
	if (inputImage.IsNull())
		return;
}

void DrrGenerator::GenerateData()
{
	mitk::Image::ConstPointer inputImage = this->GetInput(0);
	if (inputImage->GetDimension() != 3)
	{
		MITK_ERROR << "DRRSidonJacobsRayTracingFilter:GenerateData works only with 3D images, sorry.";
		itkExceptionMacro("DRRSidonJacobsRayTracingFilter:GenerateData works only with 3D images, sorry.");
		return;
	}
	AccessFixedDimensionByItk(inputImage.GetPointer(), ItkImageProcessing, 3);
}


// MODIFIED
// void DrrGenerator::SetObjTranslate(double tx, double ty, double tz)
// {
// 	m_tx = tx;
// 	m_ty = ty;
// 	m_tz = tz;
// }


// MODIFIED
// void DrrGenerator::SetObjRotate(double rx, double ry, double rz)
// {
// 	m_rx = rx;
// 	m_ry = ry;
// 	m_rz = rz;
//
// }

void DrrGenerator::SetArrayMatrixWorldToCt(double array[16])
{
	for(int i = 0 ; i < 16 ; i = i + 1)
	{
		m_ArrayMatrixWorldToCt[i] = array[i];
	}
}

void DrrGenerator::SetArrayMatrixWorldToImager(double array[16])
{
	for (int i = 0; i < 16; i = i + 1)
	{
		m_ArrayMatrixWorldToImager[i] = array[i];
	}
}

void DrrGenerator::SetRaySource(double array[3])
{
	for (int i = 0; i < 3; i = i + 1)
	{
		m_RaySource[i] = array[i];
	}
}
