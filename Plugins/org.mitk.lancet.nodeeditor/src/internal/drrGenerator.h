#ifndef DRRGENERATOR_H
#define DRRGENERATOR_H
#include "mitkImage.h"
#include "mitkImageToImageFilter.h"



class DrrGenerator :public mitk::ImageToImageFilter
{
public:
	mitkClassMacro(DrrGenerator, ImageToImageFilter);
	itkFactorylessNewMacro(Self) itkCloneMacro(Self)

		// modified Flag to  determine whether the output is for visualization

		// modified Projection angle in degrees
		// itkSetMacro(rprojection, float);

	//modified  Translation parameter of the isocenter in mm
	// itkSetMacro(tx, float);
	// itkSetMacro(ty, float);
	// itkSetMacro(tz, float);

	// modified CT volume rotation around isocenter along x,y,z axis in degrees
	// itkSetMacro(rx, float);
	// itkSetMacro(ry, float);
	// itkSetMacro(rz, float);

	// modified The pixel indices of the isocenter
	// itkSetMacro(cx, float);
	// itkSetMacro(cy, float);
	// itkSetMacro(cz, float);

	// modified 
	itkSetMacro(threshold, float);

	// modified Source to isocenter distance in mm
	// itkSetMacro(scd, float);

	// modified Default pixel spacing in the iso-center plane in mm
	itkSetMacro(im_sx, float);
	itkSetMacro(im_sy, float);

	// modified Size of the output image in number of pixels
	itkSetMacro(dx, int);
	itkSetMacro(dy, int);

	// modified The central axis positions of the 2D images in continuous indices, not mm !!
	// itkSetMacro(o2Dx, float);
	// itkSetMacro(o2Dy, float);

	// Set transform matrix arrays
	void SetArrayMatrixWorldToCt(double array[16]);
	void SetArrayMatrixWorldToImager(double array[16]);

	// Set ray source location under imager coordinate system
	void SetRaySource(double array[3]);

	//modified 
	itkSetMacro(verbose, bool);

	/*!
	\brief easy way to set obj trans at once
	*/
	// MODIFIED
	// void SetObjTranslate(double tx, double ty, double tz);
	/*!
	  \brief easy way to set obj rotate at once
	  */
	  // MODIFIED
	// void SetObjRotate(double rx, double ry, double rz);

protected:
	/*!
	\brief standard constructor
	*/
	// MODIFIED
	DrrGenerator();
	/*!
	\brief standard destructor
	*/
	// MODIFIED
	~DrrGenerator() override;
	/*!
	\brief Method generating the output information of this filter (e.g. image dimension, image type, etc.).
	The interface ImageToImageFilter requires this implementation. Everything is taken from the input image.
	*/

	void GenerateOutputInformation() override;
	/*!
	\brief Method generating the output of this filter. Called in the updated process of the pipeline.
	This method generates the smoothed output image.
	*/

	void GenerateData() override;

	/*!
	\brief Internal templated method calling the ITK drr filter. Here the actual filtering is performed.
	*/

	template <typename TPixel, unsigned int VDimension>
	void ItkImageProcessing(const itk::Image<TPixel, VDimension>* itkImage_0); // MODIFIED

	// template <typename TPixel, unsigned int VDimension>
	// mitk::Image::Pointer GenDRR(const itk::Image<TPixel, VDimension>* input_image, DrrFilter* generator);


private:

	int m_visualFlag = 0;

	// modified Projection angle in degrees
	float m_rprojection = 0.;

	//modified obj rotation
	float m_rx{ 0. };
	float m_ry{ 0. };
	float m_rz{ 0. };

	//modified obj translation
	float m_tx{ 0. };
	float m_ty{ 0. };
	float m_tz{ 0. };

	//modified obj center
	float m_cx{ 0. };
	float m_cy{ 0. };
	float m_cz{ 0. };

	// Transform matrix array: World axes to real Ct axes
	double m_ArrayMatrixWorldToCt[16]
	{
		  1,0,0,0,
		  0,1,0,0,
		  0,0,1,0,
		  0,0,0,1
	};

	// Transform matrix array: World axes to imager axes
	double m_ArrayMatrixWorldToImager[16]
	{
		  1,0,0,0,
		  0,1,0,0,
		  0,0,1,0,
		  0,0,0,1
	};

	// Ray source location under imager coordinate system
	double m_RaySource[3]{ 255.5,255.5,700 };
	   
	//drr para
	  //double m_focalpoint{ 0.0 };
	float m_threshold{ 0. }; // modifed 

	float m_scd{ 1000.0 }; // modified 

	float m_im_sx{ 0.51 };// modified pixel spacing along X of the 2D DRR image[mm]
	float m_im_sy{ 0.51 };// modified pixel spacing along Y of the 2D DRR image[mm]

  //modified output image size
	int m_dx = 512;
	int m_dy = 512;

	double m_o2Dx{ 0.0f }; // modified 
	double m_o2Dy{ 0.0f };// modified

	//modified double m_direction;
	bool m_verbose{ true };
};





#endif // DRRGENERATOR_H

