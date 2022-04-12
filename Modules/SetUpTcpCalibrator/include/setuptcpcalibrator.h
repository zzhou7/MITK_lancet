#ifndef SETUPTCPCALIBRATOR_H
#define SETUPTCPCALIBRATOR_H
#include "mitkImage.h"
#include "mitkImageToImageFilter.h"
#include "MitkSetUpTcpCalibratorExports.h"



class MITKSETUPTCPCALIBRATOR_EXPORT SetUpTcpCalibrator : public itk::Object
{
public:
  mitkClassMacroItkParent(SetUpTcpCalibrator, itk::Object);
  itkNewMacro(Self);

  itkGetMacro(Rx, double);
  itkGetMacro(Ry, double);
  itkGetMacro(Rz, double);

  itkGetMacro(Tx, double);
  itkGetMacro(Ty, double);
  itkGetMacro(Tz, double);

  // void calibrate(double toolPointA[3],
  //                double toolPointB[3],
  //                double toolPointC[3],
  //                double sawPointD[3],
  //                double sawPlanePointP[3],
  //                double sawPlanePointQ[3],
  //                double sawPlanePointS[3]);

  void calibrateGooseSaw(double MatrixRefToPointACoordinate[16],
                   double sawPointD[3],
                   double sawPlanePointP[3],
                   double sawPlanePointQ[3],
                   double sawPlanePointS[3]);

protected:
  SetUpTcpCalibrator();
  ~SetUpTcpCalibrator();



private:
  double m_Rx{0};
  double m_Ry{0};
  double m_Rz{0};

  double m_Tx{0};
  double m_Ty{0};
  double m_Tz{0};


};


#endif // SETUPTCPCALIBRATOR_H
