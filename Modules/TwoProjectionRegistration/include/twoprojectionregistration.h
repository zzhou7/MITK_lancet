#ifndef TWOPROJECTIONREGISTRATION_H
#define TWOPROJECTIONREGISTRATION_H
#include "mitkImage.h"
#include "MitkTwoProjectionRegistrationExports.h"
#include "mitkImageToImageFilter.h"



class MITKTWOPROJECTIONREGISTRATION_EXPORT TwoProjectionRegistration : public itk::Object
{
public:
  mitkClassMacroItkParent(TwoProjectionRegistration, itk::Object);
  itkNewMacro(Self);

  // A switch to turn on/off the optimizer, if off, the register becomes a pure metric calculator at the current point
  itkSetMacro(switchOffOptimizer, bool);
  itkGetMacro(metric, float);
  // Projection angle of DRR 1 and DRR 2
  itkSetMacro(angleDRR1, float);
  itkSetMacro(angleDRR2, float);

  // initial Translation parameter of the isocenter in mm
  itkSetMacro(tx, float);
  itkSetMacro(ty, float);
  itkSetMacro(tz, float);

  // initial CT volume rotation around isocenter along x,y,z axis in degrees
  itkSetMacro(rx, float);
  itkSetMacro(ry, float);
  itkSetMacro(rz, float);

  // The pixel indices of the isocenter
  itkSetMacro(cx, float);
  itkSetMacro(cy, float);
  itkSetMacro(cz, float);

  itkSetMacro(threshold, float);

  // Source to isocenter distance in mm
  itkSetMacro(scd, float);

  // DRR 1 pixel spacing in the iso-center plane in mm
  itkSetMacro(sx_1, float);
  itkSetMacro(sy_1, float);

  // DRR 2 pixel spacing in the iso-center plane in mm
  itkSetMacro(sx_2, float);
  itkSetMacro(sy_2, float);
  
  // The central axis offset for DRR 1
  itkSetMacro(o2Dx_1, float);
  itkSetMacro(o2Dy_1, float);

  // The central axis offset for DRR 2
  itkSetMacro(o2Dx_2, float);
  itkSetMacro(o2Dy_2, float);

  
  itkSetMacro(verbose, bool);
  itkSetMacro(debug, bool);

  itkGetMacro(RX, double);
  itkGetMacro(RY, double);
  itkGetMacro(RZ, double);
  itkGetMacro(TX, double);
  itkGetMacro(TY, double);
  itkGetMacro(TZ, double);
  

  // typedef float InternalPixelType;
  // typedef itk::Image<InternalPixelType, 3> InternalImageType;
  // itkSetMacro(image_tmp1, InternalImageType::Pointer);
  // itkSetMacro(image_tmp2, InternalImageType::Pointer);
  // itkSetMacro(image3Df, InternalImageType::Pointer);

  void link_drr1_cast(mitk::Image::ConstPointer inputImage); // link the mitk image to itk function
  template <typename TPixel>
  void drr1_cast(
    const itk::Image<TPixel, 3> *itkImage); // for DRR 1: cast short pixel type input image to float pixel type

  void link_drr2_cast(mitk::Image::ConstPointer inputImage); // link the mitk image to itk function
  template <typename TPixel>
  void drr2_cast(
    const itk::Image<TPixel, 3> *itkImage); // for DRR 2: cast short pixel type input image to float pixel type

  void link_3d_cast(mitk::Image::ConstPointer inputImage); // link the mitk image to itk function
  template <typename TPixel>
  void ct3d_cast(
    const itk::Image<TPixel, 3> *itkImage); // for the CT image: cast short pixel type input image to float pixel type

  void twoprojection_registration();

  void Clear();


protected:
  TwoProjectionRegistration();
  ~TwoProjectionRegistration();



private:
  // three internal itk images with float pixel type
  itk::Image<float, 3>::Pointer m_image3Df;   // input CT with float pixel type
  itk::Image<float, 3>::Pointer m_image_tmp1; // DRR 1
  itk::Image<float, 3>::Pointer m_image_tmp2; // DRR 2

  bool m_switchOffOptimizer{false};
  float m_metric{0};

  // DRR 1 projection angle
  float m_angleDRR1{0.0};
  // DRR 2 projection angle
  float m_angleDRR2{90.0};

  //obj rotation
  float m_rx{0.};
  float m_ry{0.};
  float m_rz{0.};

  //obj translation
  float m_tx{0.};
  float m_ty{0.};
  float m_tz{0.};

  // obj center
  float m_cx{0.};
  float m_cy{0.};
  float m_cz{0.};


  float m_threshold{0.}; // modifed

  float m_scd{1000.0}; // modified

  float m_sx_1{1}; // pixel spacing along X of DRR 1 [mm]
  float m_sy_1{1}; // pixel spacing along Y of DRR 1 [mm]
  float m_sx_2{1}; // pixel spacing along X of DRR 2 [mm]
  float m_sy_2{1}; // pixel spacing along Y of DRR 2 [mm]

  // Central axis offset of DRR 1
  double m_o2Dx_1{0.0f};
  double m_o2Dy_1{0.0f};

  // Central axis offset of DRR 2
  double m_o2Dx_2{0.0f};
  double m_o2Dy_2{0.0f};

  bool m_verbose{true};
  bool m_debug{true};


  // Registration results
  double m_RX{0.0};
  double m_RY{0.0};
  double m_RZ{0.0};
  double m_TX{0.0};
  double m_TY{0.0};
  double m_TZ{0.0};

};


#endif // TWOPROJECTIONREGISTRATION_H
