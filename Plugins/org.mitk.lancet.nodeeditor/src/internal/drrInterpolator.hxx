#ifndef drrInterpolator_hxx
#define drrInterpolator_hxx

#include "drrInterpolator.h"

#include "stdlib.h"
#include "vnl/vnl_math.h"
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>


namespace itk
{
	template <typename TInputImage, typename TCoordRep>
	DrrInterpolator<TInputImage,
		TCoordRep>::DrrInterpolator()
	{
		m_FocalPointToIsocenterDistance = 1000.; // Focal point to isocenter distance in mm.
		m_ProjectionAngle = 0.;                  // Angle in radians betweeen projection central axis and reference axis
		m_Threshold = 0.;                        // Intensity threshold, below which is ignored.

		m_SourcePoint[0] = 0.;
		m_SourcePoint[1] = 0.;
		m_SourcePoint[2] = 0.;

		m_InverseTransform = TransformType::New();
		m_InverseTransform->SetComputeZYX(true);

		m_ComposedTransform = TransformType::New();
		m_ComposedTransform->SetComputeZYX(true);

		m_GantryRotTransform = TransformType::New();
		m_GantryRotTransform->SetComputeZYX(true);
		m_GantryRotTransform->SetIdentity();

		m_CamShiftTransform = TransformType::New();
		m_CamShiftTransform->SetComputeZYX(true);
		m_CamShiftTransform->SetIdentity();

		m_CamRotTransform = TransformType::New();
		m_CamRotTransform->SetComputeZYX(true);
		m_CamRotTransform->SetIdentity();
		// constant for converting degrees into radians
		const float dtr = (atan(1.0) * 4.0) / 180.0;
		m_CamRotTransform->SetRotation(dtr * (-90.0), 0.0, 0.0);

		m_Threshold = 0;


	}

	template <typename TInputImage, typename TCoordRep>
	void DrrInterpolator<TInputImage, TCoordRep>::PrintSelf(std::ostream& os,
		Indent indent) const
	{
		this->Superclass::PrintSelf(os, indent);

		os << indent << "Threshold: " << m_Threshold << std::endl;
		os << indent << "Transform: " << m_Transform.GetPointer() << std::endl;
	}

	template <typename TInputImage, typename TCoordRep>
	typename DrrInterpolator<TInputImage, TCoordRep>::OutputType
		DrrInterpolator<TInputImage, TCoordRep>::Evaluate(const PointType& point) const
	{
		float rayVector[3];
		IndexType cIndex;

		PointType drrPixelWorld; // Coordinate of a DRR pixel in the world coordinate system
		OutputType pixval;

		float firstIntersection[3];
		float alphaX1, alphaXN, alphaXmin, alphaXmax;
		float alphaY1, alphaYN, alphaYmin, alphaYmax;
		float alphaZ1, alphaZN, alphaZmin, alphaZmax;
		float alphaMin, alphaMax;
		float alphaX, alphaY, alphaZ, alphaCmin, alphaCminPrev;
		float alphaUx, alphaUy, alphaUz;
		float alphaIntersectionUp[3], alphaIntersectionDown[3];
		float d12, value;
		float firstIntersectionIndex[3];
		int firstIntersectionIndexUp[3], firstIntersectionIndexDown[3];
		int iU, jU, kU;

		// Min/max values of the output pixel type AND these values
		// represented as the output type of the interpolator
		const OutputType minOutputValue = itk::NumericTraits<OutputType>::NonpositiveMin();
		const OutputType maxOutputValue = itk::NumericTraits<OutputType>::max();

		// If the volume was shifted, recalculate the overall inverse transform
		unsigned long int interpMTime = this->GetMTime();
		unsigned long int vTransformMTime = m_Transform->GetMTime();

		if (interpMTime < vTransformMTime)
		{
			this->ComputeInverseTransform();
			// The m_SourceWorld should be computed here to avoid the repeatedly calculation
			// for each projection ray. However, we are in a const function, which prohibits
			// the modification of class member variables. So the world coordinate of the source
			// point is calculated for each ray as below. Performance improvement may be made
			// by using a static variable?
			// m_SourceWorld = m_InverseTransform->TransformPoint(m_SourcePoint);
		}

		PointType SourceWorld = m_InverseTransform->TransformPoint(m_SourcePoint);

		// Get ths input pointers
		InputImageConstPointer inputPtr = this->GetInputImage();

		typename InputImageType::SizeType sizeCT;
		typename InputImageType::RegionType regionCT;
		typename InputImageType::SpacingType ctPixelSpacing;
		typename InputImageType::PointType ctOrigin;

		ctPixelSpacing = inputPtr->GetSpacing();
		ctOrigin = inputPtr->GetOrigin();
		regionCT = inputPtr->GetLargestPossibleRegion();
		sizeCT = regionCT.GetSize();

		drrPixelWorld = m_InverseTransform->TransformPoint(point);

		// The following is the Siddon-Jacob fast ray-tracing algorithm

		rayVector[0] = drrPixelWorld[0] - SourceWorld[0];
		rayVector[1] = drrPixelWorld[1] - SourceWorld[1];
		rayVector[2] = drrPixelWorld[2] - SourceWorld[2];

		/* Calculate the parametric  values of the first  and  the  last
		intersection points of  the  ray  with the X,  Y, and Z-planes  that
		define  the  CT volume. */
		if (rayVector[0] != 0)
		{
			alphaX1 = (0.0 - SourceWorld[0]) / rayVector[0];
			alphaXN = (sizeCT[0] * ctPixelSpacing[0] - SourceWorld[0]) / rayVector[0];
			alphaXmin = std::min(alphaX1, alphaXN);
			alphaXmax = std::max(alphaX1, alphaXN);
		}
		else
		{
			alphaXmin = -2;
			alphaXmax = 2;
		}

		if (rayVector[1] != 0)
		{
			alphaY1 = (0.0 - SourceWorld[1]) / rayVector[1];
			alphaYN = (sizeCT[1] * ctPixelSpacing[1] - SourceWorld[1]) / rayVector[1];
			alphaYmin = std::min(alphaY1, alphaYN);
			alphaYmax = std::max(alphaY1, alphaYN);
		}
		else
		{
			alphaYmin = -2;
			alphaYmax = 2;
		}

		if (rayVector[2] != 0)
		{
			alphaZ1 = (0.0 - SourceWorld[2]) / rayVector[2];
			alphaZN = (sizeCT[2] * ctPixelSpacing[2] - SourceWorld[2]) / rayVector[2];
			alphaZmin = std::min(alphaZ1, alphaZN);
			alphaZmax = std::max(alphaZ1, alphaZN);
		}
		else
		{
			alphaZmin = -2;
			alphaZmax = 2;
		}

		/* Get the very first and the last alpha values when the ray
		intersects with the CT volume. */
		alphaMin = std::max(std::max(alphaXmin, alphaYmin), alphaZmin);
		alphaMax = std::min(std::min(alphaXmax, alphaYmax), alphaZmax);

		/* Calculate the parametric values of the first intersection point
		of the ray with the X, Y, and Z-planes after the ray entered the
		CT volume. */

		firstIntersection[0] = SourceWorld[0] + alphaMin * rayVector[0];
		firstIntersection[1] = SourceWorld[1] + alphaMin * rayVector[1];
		firstIntersection[2] = SourceWorld[2] + alphaMin * rayVector[2];

		/* Transform world coordinate to the continuous index of the CT volume*/
		firstIntersectionIndex[0] = firstIntersection[0] / ctPixelSpacing[0];
		firstIntersectionIndex[1] = firstIntersection[1] / ctPixelSpacing[1];
		firstIntersectionIndex[2] = firstIntersection[2] / ctPixelSpacing[2];

		firstIntersectionIndexUp[0] = (int)ceil(firstIntersectionIndex[0]);
		firstIntersectionIndexUp[1] = (int)ceil(firstIntersectionIndex[1]);
		firstIntersectionIndexUp[2] = (int)ceil(firstIntersectionIndex[2]);

		firstIntersectionIndexDown[0] = (int)floor(firstIntersectionIndex[0]);
		firstIntersectionIndexDown[1] = (int)floor(firstIntersectionIndex[1]);
		firstIntersectionIndexDown[2] = (int)floor(firstIntersectionIndex[2]);

		if (rayVector[0] == 0)
		{
			alphaX = 2;
		}
		else
		{
			alphaIntersectionUp[0] = (firstIntersectionIndexUp[0] * ctPixelSpacing[0] - SourceWorld[0]) / rayVector[0];
			alphaIntersectionDown[0] = (firstIntersectionIndexDown[0] * ctPixelSpacing[0] - SourceWorld[0]) / rayVector[0];
			alphaX = std::max(alphaIntersectionUp[0], alphaIntersectionDown[0]);
		}

		if (rayVector[1] == 0)
		{
			alphaY = 2;
		}
		else
		{
			alphaIntersectionUp[1] = (firstIntersectionIndexUp[1] * ctPixelSpacing[1] - SourceWorld[1]) / rayVector[1];
			alphaIntersectionDown[1] = (firstIntersectionIndexDown[1] * ctPixelSpacing[1] - SourceWorld[1]) / rayVector[1];
			alphaY = std::max(alphaIntersectionUp[1], alphaIntersectionDown[1]);
		}

		if (rayVector[2] == 0)
		{
			alphaZ = 2;
		}
		else
		{
			alphaIntersectionUp[2] = (firstIntersectionIndexUp[2] * ctPixelSpacing[2] - SourceWorld[2]) / rayVector[2];
			alphaIntersectionDown[2] = (firstIntersectionIndexDown[2] * ctPixelSpacing[2] - SourceWorld[2]) / rayVector[2];
			alphaZ = std::max(alphaIntersectionUp[2], alphaIntersectionDown[2]);
		}

		/* Calculate alpha incremental values when the ray intercepts with x, y, and z-planes */
		if (rayVector[0] != 0)
		{
			alphaUx = ctPixelSpacing[0] / fabs(rayVector[0]);
		}
		else
		{
			alphaUx = 999;
		}
		if (rayVector[1] != 0)
		{
			alphaUy = ctPixelSpacing[1] / fabs(rayVector[1]);
		}
		else
		{
			alphaUy = 999;
		}
		if (rayVector[2] != 0)
		{
			alphaUz = ctPixelSpacing[2] / fabs(rayVector[2]);
		}
		else
		{
			alphaUz = 999;
		}
		/* Calculate voxel index incremental values along the ray path. */
		if (SourceWorld[0] < drrPixelWorld[0])
		{
			iU = 1;
		}
		else
		{
			iU = -1;
		}
		if (SourceWorld[1] < drrPixelWorld[1])
		{
			jU = 1;
		}
		else
		{
			jU = -1;
		}

		if (SourceWorld[2] < drrPixelWorld[2])
		{
			kU = 1;
		}
		else
		{
			kU = -1;
		}

		d12 = 0.0; /* Initialize the sum of the voxel intensities along the ray path to zero. */

		/* Initialize the current ray position. */
		alphaCmin = std::min(std::min(alphaX, alphaY), alphaZ);

		/* Initialize the current voxel index. */
		cIndex[0] = firstIntersectionIndexDown[0];
		cIndex[1] = firstIntersectionIndexDown[1];
		cIndex[2] = firstIntersectionIndexDown[2];

		while (alphaCmin < alphaMax) /* Check if the ray is still in the CT volume */
		{
			/* Store the current ray position */
			alphaCminPrev = alphaCmin;

			if ((alphaX <= alphaY) && (alphaX <= alphaZ))
			{
				/* Current ray front intercepts with x-plane. Update alphaX. */
				alphaCmin = alphaX;
				cIndex[0] = cIndex[0] + iU;
				alphaX = alphaX + alphaUx;
			}
			else if ((alphaY <= alphaX) && (alphaY <= alphaZ))
			{
				/* Current ray front intercepts with y-plane. Update alphaY. */
				alphaCmin = alphaY;
				cIndex[1] = cIndex[1] + jU;
				alphaY = alphaY + alphaUy;
			}
			else
			{
				/* Current ray front intercepts with z-plane. Update alphaZ. */
				alphaCmin = alphaZ;
				cIndex[2] = cIndex[2] + kU;
				alphaZ = alphaZ + alphaUz;
			}

			if ((cIndex[0] >= 0) && (cIndex[0] < static_cast<IndexValueType>(sizeCT[0])) && (cIndex[1] >= 0) &&
				(cIndex[1] < static_cast<IndexValueType>(sizeCT[1])) && (cIndex[2] >= 0) &&
				(cIndex[2] < static_cast<IndexValueType>(sizeCT[2])))
			{
				/* If it is a valid index, get the voxel intensity. */
				value = static_cast<float>(inputPtr->GetPixel(cIndex));
				if (value > m_Threshold) /* Ignore voxels whose intensities are below the threshold. */
				{
					d12 += (alphaCmin - alphaCminPrev) * (value - m_Threshold);
				}
			}
		}

		if (d12 < minOutputValue)
		{
			pixval = minOutputValue;
		}
		else if (d12 > maxOutputValue)
		{
			pixval = maxOutputValue;
		}
		else
		{
			pixval = static_cast<OutputType>(d12);
		}
		return (pixval);
	}

	template <typename TInputImage, typename TCoordRep>
	typename DrrInterpolator<TInputImage, TCoordRep>::OutputType
		DrrInterpolator<TInputImage, TCoordRep>::EvaluateAtContinuousIndex(
			const ContinuousIndexType& index) const
	{
		OutputPointType point;
		this->m_Image->TransformContinuousIndexToPhysicalPoint(index, point);
		// std::cout << "Point" << std::endl;
		return this->Evaluate(point);
	}

	template <typename TInputImage, typename TCoordRep>
	void DrrInterpolator<TInputImage, TCoordRep>::AppendTransformOffset() const
	{
		// retrieve the matrix form of the transform offset
		Eigen::Matrix4d eigenMatrixTransformOffset{ m_ArrayTransformOffset };
		eigenMatrixTransformOffset.transposeInPlace();

		// get the matrix form of m_Transform (world to real initial Ct)
		double vtkTranslation[3]{
		  (m_Transform->GetTranslation())[0],
		  (m_Transform->GetTranslation())[1],
		  (m_Transform->GetTranslation())[2] };
		double piParameter = 180 / 3.141592654;
		auto vtkInitialTransform = vtkSmartPointer<vtkTransform>::New();
		vtkSmartPointer<vtkMatrix4x4> vtkMatrixInitialTransform = vtkSmartPointer<vtkMatrix4x4>::New();
		vtkInitialTransform->Identity();
		vtkInitialTransform->PostMultiply();
    // m_Transform is ZYX, i.e. : extrinsic rotation order: along X --> along Y --> along Z
    vtkInitialTransform->RotateX(m_Transform->GetAngleX() * piParameter);
		vtkInitialTransform->RotateY(m_Transform->GetAngleY() * piParameter);
		vtkInitialTransform->RotateZ(m_Transform->GetAngleZ() * piParameter);
		vtkInitialTransform->Translate(vtkTranslation);
		vtkInitialTransform->GetMatrix(vtkMatrixInitialTransform);
		Eigen::Matrix4d eigenMatrixInitialTransform{ vtkMatrixInitialTransform->GetData() };
		eigenMatrixInitialTransform.transposeInPlace(); // world to real initial Ct

		// Composite transform matrix i.e. the transform from internal CT center coordinate system to real Ct center coordinate system
		Eigen::Matrix4d eigenMatrixCompositeTransform = eigenMatrixTransformOffset * eigenMatrixInitialTransform;

		// The volume center of the internal Ct volume under the internal Ct coordinate system
		Eigen::Vector4d internalCtCenter{ m_MovingImageSpacing[0] * double(m_MovingImagePixelNumbers[0]) / 2.0,
										 m_MovingImageSpacing[1] * double(m_MovingImagePixelNumbers[1]) / 2.0,
										 m_MovingImageSpacing[2] * double(m_MovingImagePixelNumbers[2]) / 2.0,
										 1 };
		// The center of the real Ct volume under the real Ct coordinate system
		Eigen::Vector4d ctCenter{ m_MovingImageSpacing[0] * double(m_MovingImagePixelNumbers[0]) / 2.0,
								 m_MovingImageSpacing[1] * double(m_MovingImagePixelNumbers[1]) / 2.0,
								 m_MovingImageSpacing[2] * double(m_MovingImagePixelNumbers[2]) / 2.0,
								 1 };

		// The volume center of the real Ct volume under the internal Ct coordinate system
		Eigen::Vector4d targetCenterPoint = eigenMatrixCompositeTransform * ctCenter;

		// Get composite translation
		typename TransformType::OutputVectorType compositeTranslation;
		compositeTranslation[0] = targetCenterPoint[0] - internalCtCenter[0];
		compositeTranslation[1] = targetCenterPoint[1] - internalCtCenter[1];
		compositeTranslation[2] = targetCenterPoint[2] - internalCtCenter[2];

		// m_Transform->SetTranslation(compositeTranslation);

		// Get composite rotation rx, ry, rz, extrinsic rotation order: along X --> along Y --> along Z in radian
		double rx, ry, rz;


    Eigen::Matrix3d eigenRotationMatrixCompositeTransform;
    for (int i = 0; i < 3; i = i + 1)
    {
      for (int j = 0; i < 3; i = i + 1)
      {
        eigenRotationMatrixCompositeTransform(i, j) = eigenMatrixCompositeTransform(i, j);
      }
    }
    // extrinsic rotation order: along X --> along Y --> along Z in radian
    Eigen::Vector3d eulerAngles = eigenRotationMatrixCompositeTransform.eulerAngles(2, 1, 0); 
    rx = eulerAngles[2];
    ry = eulerAngles[1];
    rz = eulerAngles[0];

    // Problematic way of retrieving the euler angles
		// if (eigenMatrixCompositeTransform(0, 2) < 1)
		// {
		// 	if (eigenMatrixCompositeTransform(0, 2) > -1)
		// 	{
		// 		ry = asin(eigenMatrixCompositeTransform(0, 2));
		// 		rx = atan2(-eigenMatrixCompositeTransform(1, 2), eigenMatrixCompositeTransform(2, 2));
		// 		rz = atan2(-eigenMatrixCompositeTransform(0, 1), eigenMatrixCompositeTransform(0, 0));
		// 	}
		// 	else
		// 	{
		// 		ry = -3.1415926 / 2;
		// 		rx = -atan2(eigenMatrixCompositeTransform(1, 0), eigenMatrixCompositeTransform(1, 1));
		// 		rz = 0;
		// 	}
		// }
		// else
		// {
		// 	ry = 3.1415926 / 2;
		// 	rx = atan2(eigenMatrixCompositeTransform(1, 0), eigenMatrixCompositeTransform(1, 1));
		// 	rz = 0;
		// }

		// Update m_Transform
		typename TransformType::InputPointType newIsocenter;
		newIsocenter[0] = m_MovingImageSpacing[0] * double(m_MovingImagePixelNumbers[0]) / 2.0;
		newIsocenter[1] = m_MovingImageSpacing[1] * double(m_MovingImagePixelNumbers[1]) / 2.0;
		newIsocenter[2] = m_MovingImageSpacing[2] * double(m_MovingImagePixelNumbers[2]) / 2.0;

		m_TransformCopy->SetIdentity();
    m_TransformCopy->SetComputeZYX(true);
    m_TransformCopy->SetCenter(newIsocenter);
    m_TransformCopy->SetTranslation(compositeTranslation);
    m_TransformCopy->SetRotation(rx, ry, rz);
	}

	template <typename TInputImage, typename TCoordRep>
	void DrrInterpolator<TInputImage, TCoordRep>::ComputeInverseTransform() const
	{
		this->AppendTransformOffset();
		m_ComposedTransform->SetIdentity();
		m_ComposedTransform->Compose(m_TransformCopy, 0);

		typename TransformType::InputPointType isocenter;
		isocenter = m_TransformCopy->GetCenter();
		// An Euler 3D transform is used to rotate the volume to simulate the roation of the linac gantry.
		// The rotation is about z-axis. After the transform, a AP projection geometry (projecting
		// towards positive y direction) is established.
		m_GantryRotTransform->SetRotation(0.0, 0.0, -m_ProjectionAngle);
		m_GantryRotTransform->SetCenter(isocenter);
		m_ComposedTransform->Compose(m_GantryRotTransform, 0);

		// An Euler 3D transfrom is used to shift the source to the origin.
		typename TransformType::OutputVectorType focalpointtranslation;
		focalpointtranslation[0] = -isocenter[0];
		focalpointtranslation[1] = m_FocalPointToIsocenterDistance - isocenter[1];
		focalpointtranslation[2] = -isocenter[2];
		m_CamShiftTransform->SetTranslation(focalpointtranslation);
		m_ComposedTransform->Compose(m_CamShiftTransform, 0);

		// A Euler 3D transform is used to establish the standard negative z-axis projection geometry. (By
		// default, the camera is situated at the origin, points down the negative z-axis, and has an up-
		// vector of (0, 1, 0).)

		m_ComposedTransform->Compose(m_CamRotTransform, 0);

		// The overall inverse transform is computed. The inverse transform will be used by the interpolation
		// procedure.
		m_ComposedTransform->GetInverse(m_InverseTransform);
		this->Modified();
	}

	template <typename TInputImage, typename TCoordRep>
	void DrrInterpolator<TInputImage, TCoordRep>::SetArrayTransformOffset(double array[16])
	{
		for (int i = 0; i < 16; i = i + 1)
		{
			m_ArrayTransformOffset[i] = array[i];
		}
	}

	template <typename TInputImage, typename TCoordRep>
	void DrrInterpolator<TInputImage, TCoordRep>::SetMovingImagePixelNumbers(int array[3])
	{
		for (int i = 0; i < 3; i = i + 1)
		{
			m_MovingImagePixelNumbers[i] = array[i];
		}
	}

	template <typename TInputImage, typename TCoordRep>
	void DrrInterpolator<TInputImage, TCoordRep>::SetMovingImageSpacing(double array[3])
	{
		for (int i = 0; i < 3; i = i + 1)
		{
			m_MovingImageSpacing[i] = array[i];
		}
	}


	template <typename TInputImage, typename TCoordRep>
	void DrrInterpolator<TInputImage, TCoordRep>::Initialize()
	{
		this->ComputeInverseTransform();
		m_SourceWorld = m_InverseTransform->TransformPoint(m_SourcePoint);
	}

} // namespace itk

#endif
