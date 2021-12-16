#include "RRTC.h"

#include <vtkLandmarkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkTransform.h>
//#define STAUBLI
#define KUKA

// initialize
void RRTC::init()
{
	hasInitialPoints = false;
	hasVerifyPoints = false;
	hasSetTcpdata = false;
	hasRegistrationPara = false;
	verify_t_num = 0;
	RobotRegistrationRMS = 0.0;
	ToolCalibrationRMS = 0.0;
	Pr_p.clear();
	Pr_r.clear();
	Pv_p.clear();
	R4r.clear();
	Tcpr_p.clear();
	Tcpr_r.clear();
	Tcpv_p.clear();
	R4tcp.clear();
	Verifyr_p.clear();
	Verifyr_r.clear();
	Verifyv_p.clear();	
	R4verify.clear();
}

// set initial points
void RRTC::setInitialPoints(std::vector<Eigen::Vector3d> p, int t_num)
{
	for (int i = 0; i + 2 < 3 * t_num; i += 3)
	{
		Pr_p.push_back(p.at(i));
		Pr_r.push_back(p.at(i + 1));
		Pv_p.push_back(p.at(i + 2));
	}
	for (int j = 3 * t_num; j + 2 < p.size(); j += 3)
	{
		Tcpr_p.push_back(p.at(j));
		Tcpr_r.push_back(p.at(j + 1));
		Tcpv_p.push_back(p.at(j + 2));

		//source_points_->InsertNextPoint(p.at(j)(0, 0), p.at(j)(1, 0), p.at(j)(2, 0));
		//target_points_->InsertNextPoint(p.at(j + 2)(0, 0), p.at(j + 2)(1, 0), p.at(j + 2)(2, 0));
	}
	hasInitialPoints = true;
	calculateRX(false);
}

// set verify points
void RRTC::setVerifyPoints(std::vector<Eigen::Vector3d> p, int t_num)
{
	hasRegistrationPara = false;
	for (int i = 0; i + 2 < p.size(); i += 3)
	{
		Verifyr_p.push_back(p.at(i));
		Verifyr_r.push_back(p.at(i + 1));
		Verifyv_p.push_back(p.at(i + 2));

		//source_points_->InsertNextPoint(p.at(i)(0, 0), p.at(i)(1, 0), p.at(i)(2, 0));
		//target_points_->InsertNextPoint(p.at(i + 2)(0, 0), p.at(i + 2)(1, 0), p.at(i + 2)(2, 0));
	}
	verify_t_num = t_num;
	calculateRX(true);
}

// set tcp data
void RRTC::setTcpdata(double x, double y, double z)
{
	hasSetTcpdata = true;
	Tcp << x, y, z;
	std::cout << "\nSet Tcp data: \n" << Tcp << std::endl;
}


// get Robot Registration Matrix
void RRTC::calculateResult()
{
	if (hasInitialPoints)
	{
		calculateR0();
		if(!hasSetTcpdata)
			calculateTcp();
		calculateT();
		calculateRobotRegistrationRMS();
		calculateToolCalibrationRMS();
		hasRegistrationPara = true;
	}
	else
		std::cout << "\nSet InitialPoints First!\n" << std::endl;
}

// update TCP and Robot Registration Matrix
void RRTC::updateCalculateResult()
{
	incrementHandle();
	if (hasVerifyPoints)
	{
		calculateR0();
		if(!hasSetTcpdata)
			calculateTcp();
		calculateT();
		calculateRobotRegistrationRMS();
		calculateToolCalibrationRMS();
		verify_t_num = 0;
		hasRegistrationPara = true;
		hasVerifyPoints = false;
	}
	else
		std::cout << "\nSet VerifyPoints First!\n" << std::endl;
}

// increment handle
void RRTC::incrementHandle()
{
	if (verify_t_num > 0)
		hasVerifyPoints = true;
	// merge Verify's t_num points to P
	for (int i = 0; i < verify_t_num; i++)
	{
		Pr_p.push_back(Verifyr_p.at(i));
		Pr_r.push_back(Verifyr_r.at(i));
		Pv_p.push_back(Verifyv_p.at(i));
		R4r.push_back(R4verify.at(i));
	}
	// merge other Verify points to Tcp
	for (int j = verify_t_num; j < Verifyr_p.size(); j++)
	{
		Tcpr_p.push_back(Verifyr_p.at(j));
		Tcpr_r.push_back(Verifyr_r.at(j));
		Tcpv_p.push_back(Verifyv_p.at(j));
		R4tcp.push_back(R4verify.at(j));
	}
	Verifyr_p.clear();
	Verifyr_r.clear();
	Verifyv_p.clear();
	R4verify.clear();
}

// Calculate R
Eigen::Matrix3d RRTC::calculateR(Eigen::Vector3d rxyz)
{	
#ifdef KUKA
	Eigen::AngleAxisd rx(rxyz(0, 0) / d2r, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd ry(rxyz(1, 0) / d2r, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd rz(rxyz(2, 0) / d2r, Eigen::Vector3d::UnitX());
	//return rz.matrix()*ry.matrix()*rx.matrix(); // kuka
#endif

#ifdef STAUBLI
	Eigen::AngleAxisd rx(rxyz(0, 0) / d2r, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd ry(rxyz(1, 0) / d2r, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd rz(rxyz(2, 0) / d2r, Eigen::Vector3d::UnitZ());
	//return rx.matrix()*ry.matrix()*rz.matrix(); // staubli
#endif
	return rx.matrix()*ry.matrix()*rz.matrix();
}



// Calculate RX
void RRTC::calculateRX(bool isVerifyflag)
{
	if (!isVerifyflag)
	{
		for (int i = 0; i < Pr_r.size(); i++)
		{
#ifdef KUKA
			Eigen::AngleAxisd rx(Pr_r.at(i)(0, 0) / d2r, Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd ry(Pr_r.at(i)(1, 0) / d2r, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rz(Pr_r.at(i)(2, 0) / d2r, Eigen::Vector3d::UnitX());
			//R4r.push_back(rz.matrix()*ry.matrix()*rx.matrix());
#endif
#ifdef STAUBLI
			Eigen::AngleAxisd rx(Pr_r.at(i)(0, 0) / d2r, Eigen::Vector3d::UnitX());
			Eigen::AngleAxisd ry(Pr_r.at(i)(1, 0) / d2r, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rz(Pr_r.at(i)(2, 0) / d2r, Eigen::Vector3d::UnitZ());
			//R4r.push_back(rx.matrix()*ry.matrix()*rz.matrix());
#endif
			R4r.push_back(rx.matrix()*ry.matrix()*rz.matrix());
			//std::cout << "\nRX:\n" << R4r.at(i) << std::endl;
		}
		for (int j = 0; j < Tcpr_r.size(); j++)
		{
#ifdef KUKA
			Eigen::AngleAxisd rx(Tcpr_r.at(j)(0, 0) / d2r, Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd ry(Tcpr_r.at(j)(1, 0) / d2r, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rz(Tcpr_r.at(j)(2, 0) / d2r, Eigen::Vector3d::UnitX());
			//R4tcp.push_back(rz.matrix()*ry.matrix()*rx.matrix());
#endif
#ifdef STAUBLI
			Eigen::AngleAxisd rx(Tcpr_r.at(j)(0, 0) / d2r, Eigen::Vector3d::UnitX());
			Eigen::AngleAxisd ry(Tcpr_r.at(j)(1, 0) / d2r, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rz(Tcpr_r.at(j)(2, 0) / d2r, Eigen::Vector3d::UnitZ());
			//R4tcp.push_back(rx.matrix()*ry.matrix()*rz.matrix());
#endif
			R4tcp.push_back(rx.matrix()*ry.matrix()*rz.matrix());
			//std::cout << "\nRXtcp:\n" << R4tcp.at(j) << std::endl;
		}
	}
	else
	{
		for (int i = 0; i < Verifyr_r.size(); i++)
		{
#ifdef KUKA
			Eigen::AngleAxisd rx(Verifyr_r.at(i)(0, 0) / d2r, Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd ry(Verifyr_r.at(i)(1, 0) / d2r, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rz(Verifyr_r.at(i)(2, 0) / d2r, Eigen::Vector3d::UnitX());
			//R4verify.push_back(rz.matrix()*ry.matrix()*rx.matrix());
#endif
#ifdef STAUBLI
			Eigen::AngleAxisd rx(Verifyr_r.at(i)(0, 0) / d2r, Eigen::Vector3d::UnitX());
			Eigen::AngleAxisd ry(Verifyr_r.at(i)(1, 0) / d2r, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rz(Verifyr_r.at(i)(2, 0) / d2r, Eigen::Vector3d::UnitZ());
			//R4verify.push_back(rx.matrix()*ry.matrix()*rz.matrix());
#endif
			R4verify.push_back(rx.matrix()*ry.matrix()*rz.matrix());
		}
	}
}

// step 1: Calculate R0
void RRTC::calculateR0()
{
	Eigen::MatrixXf ABt;
	Eigen::MatrixXf V;
	Eigen::MatrixXf U;
	calculateAMax();
	calculateBMax();
	int tempCol = calculateRowCol((int)Pr_p.size());
	ABt = AMax.block(0, 0, 3, tempCol) *
		BMax.block(0, 0, 3, tempCol).transpose();
	//std::cout << "\nA:\n" << AMax.block(0, 0, 3, tempCol) << std::endl;
	//std::cout << "\nB:\n" << BMax.block(0, 0, 3, tempCol) << std::endl;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(ABt, Eigen::ComputeThinU | Eigen::ComputeThinV);
	V = svd.matrixV();
	U = svd.matrixU();
	//std::cout << "V:\n" << V << std::endl;
	//std::cout << "U:\n" << U << std::endl;
	R = V * U.transpose();
	Ro << R(0, 0), R(0, 1), R(0, 2),
		R(1, 0), R(1, 1), R(1, 2),
		R(2, 0), R(2, 1), R(2, 2);
	std::cout << "R:\n" << R << std::endl;

	vtkSmartPointer<vtkLandmarkTransform> landmark_transform = vtkLandmarkTransform::New();
	landmark_transform->SetModeToRigidBody();
	vtkSmartPointer<vtkPoints>  p_vega = vtkPoints::New();
	vtkSmartPointer<vtkPoints>  p_robot = vtkPoints::New();

	for (int i = 0; i < Pv_p.size(); i++)
	{
		p_vega->InsertPoint(i, Pv_p[i].data());
		p_robot->InsertPoint(i, Pr_p[i].data());
	}

	landmark_transform->SetSourceLandmarks(p_robot);
	landmark_transform->SetTargetLandmarks(p_vega);
	landmark_transform->Update();
	landmark_transform->GetMatrix()->Print(std::cout);
}

void RRTC::calculateRT()
{
	vtkSmartPointer<vtkLandmarkTransform> landmark_transform = vtkLandmarkTransform::New();
	landmark_transform->SetModeToRigidBody();
	vtkSmartPointer<vtkPoints>  p_vega = vtkPoints::New();
	vtkSmartPointer<vtkPoints>  p_robot = vtkPoints::New();

	for (int i = 0; i < Pv_p.size(); i++)
	{
		p_vega->InsertPoint(i, Pv_p[i].data());
		p_robot->InsertPoint(i, Pr_p[i].data());
	}

	landmark_transform->SetSourceLandmarks(p_vega);
	landmark_transform->SetTargetLandmarks(p_robot);
	landmark_transform->Update();
	landmark_transform->GetMatrix()->Print(std::cout);
}

// step 2: Calculate Tcp
void RRTC::calculateTcp()
{
	Eigen::Vector3d B;
	std::vector<Eigen::Vector3d> J, K, H;
	Eigen::Matrix3d A;
	Eigen::Matrix<double, 1000, 1> NMax;
	Eigen::Matrix<double, 1000, 3> MMax;
	int temprowN, temprowM;
	temprowN = 0; temprowM = 0;
	for (int i = 0; i < Tcpr_p.size() - 1; i++)
		for (int j = i + 1; j < Tcpr_p.size(); j++)
		{
			Eigen::Vector3d jj;
			jj << Tcpr_p.at(i)(0, 0) - Tcpr_p.at(j)(0, 0),
				Tcpr_p.at(i)(1, 0) - Tcpr_p.at(j)(1, 0),
				Tcpr_p.at(i)(2, 0) - Tcpr_p.at(j)(2, 0);

			Eigen::Vector3d hh;
			hh << Tcpv_p.at(i)(0, 0) - Tcpv_p.at(j)(0, 0),
				Tcpv_p.at(i)(1, 0) - Tcpv_p.at(j)(1, 0),
				Tcpv_p.at(i)(2, 0) - Tcpv_p.at(j)(2, 0);
			B = hh - Ro * jj;
			A = Ro * R4tcp.at(i) - Ro * R4tcp.at(j);
			for (int a = 0; a < 3; a++)
			{
				NMax(temprowN++, 0) = B(a, 0);
				for (int b = 0; b < 3; b++)
				{
					MMax(temprowM, b) = A(a, b);
				}
				temprowM++;
			}

		}
	int mn = 3 * calculateRowCol((int)Tcpr_p.size());
	//std::cout << "\nN=\n" << NMax.block(0, 0, mn, 1) << " \nM=\n " << MMax.block(0, 0, mn, 3) << std::endl;
	Tcp = (MMax.block(0, 0, mn, 3).transpose()*MMax.block(0, 0, mn, 3)).inverse()*
		MMax.block(0, 0, mn, 3).transpose()*NMax.block(0, 0, mn, 1);

	std::cout << "\nTcp:\n" << Tcp << std::endl;
}

// step 3: Calculate To
void RRTC::calculateT()
{
	To << 0, 0, 0;
	for (int j = 0; j < Tcpv_p.size(); j++)
	{
		To += Tcpv_p.at(j) - Ro * R4tcp.at(j) * Tcp - Ro * Tcpr_p.at(j);
		std::cout << "\nT" << j+1 << ":\n" << Tcpv_p.at(j) - Ro * R4tcp.at(j) * Tcp - Ro * Tcpr_p.at(j) << std::endl;
	}
	To = To / Tcpv_p.size();
	std::cout << "\nT:\n" << To << std::endl;
}

// step 4: Calculate Robot Registration RMS
void RRTC::calculateRobotRegistrationRMS()
{
	RobotRegistrationRMS = 0.0;
	Eigen::Vector3d v;
	for (int i = 0; i < Tcpr_p.size(); i++)
	{
		//v = Pv_p.at(i) - Ro * R4r.at(i) * Tcp - Ro * Pr_p.at(i) - To;
		v = Tcpv_p.at(i) - Ro * R4tcp.at(i) * Tcp - Ro * Tcpr_p.at(i) - To;
		RobotRegistrationRMS += sqrt(v(0, 0)*v(0, 0) + v(1, 0)*v(1, 0) + v(2, 0)*v(2, 0));
	}
	RobotRegistrationRMS /= Tcpr_p.size();
	std::cout << "\nRobotRegistrationRMS:\n" << RobotRegistrationRMS << std::endl;
}

// step 5: Calculate Tool Calibration RMS
void RRTC::calculateToolCalibrationRMS()
{
	ToolCalibrationRMS = 0.0;
	Eigen::Vector3d v;
	for (int i = 0; i < Tcpr_p.size(); i++)
	{
		//v = Tcpv_p.at(i) - Ro * R4tcp.at(i) * Tcp - Ro * Tcpr_p.at(i) - To;
		v = R4tcp.at(i).inverse() * Ro.inverse() * (Tcpv_p.at(i) - Ro * Tcpr_p.at(i) - To) - Tcp;
		ToolCalibrationRMS += sqrt(v(0, 0)*v(0, 0) + v(1, 0)*v(1, 0) + v(2, 0)*v(2, 0));
	}
	ToolCalibrationRMS /= Tcpr_p.size();
	std::cout << "\nToolCalibrationRMS:\n" << ToolCalibrationRMS << std::endl;
}

Eigen::Vector3d RRTC::calculateRobotPositionInRobotbase(Eigen::Vector3d targetPositionInNDI, Eigen::Vector3d targetPoseInRobotbase)
{
	if (hasRegistrationPara)
		return Ro.inverse() * (targetPositionInNDI - Ro * calculateR(targetPoseInRobotbase) * Tcp - To);
	else
		return targetPositionInNDI;
}

Eigen::Vector3d RRTC::calculateRobotPositionInNDIvega(Eigen::Vector3d currentPositionInRobotbase, Eigen::Vector3d currentPoseInRobotbase)
{
	if (hasRegistrationPara)
		return Ro * calculateR(currentPoseInRobotbase) * Tcp + Ro * currentPositionInRobotbase + To;
	else
		return currentPositionInRobotbase;
}

void RRTC::calculateAMax()
{
	int tempcol;
	tempcol = 0;
	for (int i = 0; i < Pr_p.size() - 1; i++)
		for (int j = i + 1; j < Pr_p.size(); j++)
		{
			AMax(0, tempcol++) = Pr_p.at(i)(0, 0) - Pr_p.at(j)(0, 0);
		}
	tempcol = 0;
	for (int i = 0; i < Pr_p.size() - 1; i++)
		for (int j = i + 1; j < Pr_p.size(); j++)
		{
			AMax(1, tempcol++) = Pr_p.at(i)(1, 0) - Pr_p.at(j)(1, 0);
		}
	tempcol = 0;
	for (int i = 0; i < Pr_p.size() - 1; i++)
		for (int j = i + 1; j < Pr_p.size(); j++)
		{
			AMax(2, tempcol++) = Pr_p.at(i)(2, 0) - Pr_p.at(j)(2, 0);
		}
	//std::cout << "AMax:\n" << AMax << std::endl;
}

void RRTC::calculateBMax()
{
	int tempcol;
	tempcol = 0;
	for (int i = 0; i < Pv_p.size() - 1; i++)
		for (int j = i + 1; j < Pv_p.size(); j++)
		{
			BMax(0, tempcol++) = Pv_p.at(i)(0, 0) - Pv_p.at(j)(0, 0);
		}
	tempcol = 0;
	for (int i = 0; i < Pv_p.size() - 1; i++)
		for (int j = i + 1; j < Pv_p.size(); j++)
		{
			BMax(1, tempcol++) = Pv_p.at(i)(1, 0) - Pv_p.at(j)(1, 0);
		}
	tempcol = 0;
	for (int i = 0; i < Pv_p.size() - 1; i++)
		for (int j = i + 1; j < Pv_p.size(); j++)
		{
			BMax(2, tempcol++) = Pv_p.at(i)(2, 0) - Pv_p.at(j)(2, 0);
		}
	//std::cout << "BMax:\n" << BMax << std::endl;
}

int RRTC::calculateRowCol(int p)
{
	int n = 0;
	for (int i = 1; i < p; i++)
		n += i;
	return n;
}

//void RRTC::robotregistrationtoolcalibrationtest()
//{
//	std::vector<Eigen::Vector3d> testp;
//	////P0 PC0
//	//testp.push_back(Eigen::Vector3d(703.33, -333.7, -16.33));	testp.push_back(Eigen::Vector3d(166.65, 25.43, -11.77));
//	//testp.push_back(Eigen::Vector3d(-100.77, 158.30, -1493.55));
//	//P1 PC1
//	testp.push_back(Eigen::Vector3d(687.94, -317.32, -25.01));	testp.push_back(Eigen::Vector3d(166.65, 25.43, -11.77));
//	testp.push_back(Eigen::Vector3d(-103.42, 144.25, -1513.10));
//	//P2 PC2
//	testp.push_back(Eigen::Vector3d(677.57, -314.75, -65.36));	testp.push_back(Eigen::Vector3d(166.65, 25.43, -11.77));
//	testp.push_back(Eigen::Vector3d(-71.54, 131.63, -1536.75));
//	//P3 PC3
//	testp.push_back(Eigen::Vector3d(707.06, -323.31, -5.31));	testp.push_back(Eigen::Vector3d(166.65, 25.43, -11.77));
//	testp.push_back(Eigen::Vector3d(-126.70, 172.57, -1485.69));
//	//P4 PC4
//	testp.push_back(Eigen::Vector3d(698.63, -377.78, -2.09));	testp.push_back(Eigen::Vector3d(166.65, 25.43, -11.77));
//	testp.push_back(Eigen::Vector3d(-90.97, 149.59, -1449.07));
//
//	//P5 PC5
//	testp.push_back(Eigen::Vector3d(695.44, -289.57, 24.43));	testp.push_back(Eigen::Vector3d(152.73, 37.29, -25.62));
//	testp.push_back(Eigen::Vector3d(-156.75, 218.53, -1409.35));
//	//P6 PC6
//	testp.push_back(Eigen::Vector3d(706.38, -381.12, -61.69));	testp.push_back(Eigen::Vector3d(172.47, 16.54, 0.05));
//	testp.push_back(Eigen::Vector3d(-61.37, 111.58, -1529.45));
//	//P7 PC7
//	testp.push_back(Eigen::Vector3d(666.1, -315.13, -28.49));	testp.push_back(Eigen::Vector3d(162.73, 16.53, -9.71));
//	testp.push_back(Eigen::Vector3d(-109.07, 76.97, -1505.32));
//	////P8 PC8
//	//testp.push_back(Eigen::Vector3d(747.48, -366.04, -50.64));	testp.push_back(Eigen::Vector3d(172.68, 32.09, -4.69));
//	//testp.push_back(Eigen::Vector3d(-60.85, 231.22, -1520.94));
//
//	setInitialPoints(testp, 4);
//	calculateResult();
//
//	std::vector<Eigen::Vector3d> testverifyp;
//	//P0 PC0
//	testverifyp.push_back(Eigen::Vector3d(703.33, -333.7, -16.33));	testverifyp.push_back(Eigen::Vector3d(166.65, 25.43, -11.77));
//	testverifyp.push_back(Eigen::Vector3d(-100.77, 158.30, -1493.55));
//	//P8 PC8
//	testverifyp.push_back(Eigen::Vector3d(747.48, -366.04, -50.64));	testverifyp.push_back(Eigen::Vector3d(172.68, 32.09, -4.69));
//	testverifyp.push_back(Eigen::Vector3d(-60.85, 231.22, -1520.94));
//
//	setVerifyPoints(testverifyp, 1);
//	updateCalculateResult();
//}

//void RRTC::InitialTestPoints()
//{
//	Eigen::Vector3d P0(703.33, -333.7, -16.33);	Pr_p.push_back(P0);	Eigen::Vector3d r0(166.65, 25.43, -11.77);	Pr_r.push_back(r0);
//	Eigen::Vector3d P1(687.94, -317.32, -25.01);	Pr_p.push_back(P1);	Eigen::Vector3d r1(166.65, 25.43, -11.77);	Pr_r.push_back(r1);
//	Eigen::Vector3d P2(677.57, -314.75, -65.36);	Pr_p.push_back(P2);	Eigen::Vector3d r2(166.65, 25.43, -11.77);	Pr_r.push_back(r2);
//	Eigen::Vector3d P3(707.06, -323.31, -5.31);	Pr_p.push_back(P3);	Eigen::Vector3d r3(166.65, 25.43, -11.77);	Pr_r.push_back(r3);
//	Eigen::Vector3d P4(698.63, -377.78, -2.09);	Pr_p.push_back(P4);	Eigen::Vector3d r4(166.65, 25.43, -11.77);	Pr_r.push_back(r4);
//
//	Eigen::Vector3d P5(695.44, -289.57, 24.43);	Tcpr_p.push_back(P5);	Eigen::Vector3d r5(152.73, 37.29, -25.62);	Tcpr_r.push_back(r5);
//	Eigen::Vector3d P6(706.38, -381.12, -61.69);	Tcpr_p.push_back(P6);	Eigen::Vector3d r6(172.47, 16.54, 0.05);	Tcpr_r.push_back(r6);
//	Eigen::Vector3d P7(666.1, -315.13, -28.49);	Tcpr_p.push_back(P7);	Eigen::Vector3d r7(162.73, 16.53, -9.71);	Tcpr_r.push_back(r7);
//	Eigen::Vector3d P8(747.48, -366.04, -50.64);	Tcpr_p.push_back(P8);	Eigen::Vector3d r8(172.68, 32.09, -4.69);	Tcpr_r.push_back(r8);
//
//
//	Eigen::Vector3d PC0(-100.77, 158.30, -1493.55);	Pv_p.push_back(PC0);
//	Eigen::Vector3d PC1(-103.42, 144.25, -1513.10);	Pv_p.push_back(PC1);
//	Eigen::Vector3d PC2(-71.54, 131.63, -1536.75);	Pv_p.push_back(PC2);
//	Eigen::Vector3d PC3(-126.70, 172.57, -1485.69);	Pv_p.push_back(PC3);
//	Eigen::Vector3d PC4(-90.97, 149.59, -1449.07);	Pv_p.push_back(PC4);
//
//	Eigen::Vector3d PC5(-156.75, 218.53, -1409.35);	Tcpv_p.push_back(PC5);
//	Eigen::Vector3d PC6(-61.37, 111.58, -1529.45);	Tcpv_p.push_back(PC6);
//	Eigen::Vector3d PC7(-109.07, 76.97, -1505.32);	Tcpv_p.push_back(PC7);
//	Eigen::Vector3d PC8(-60.85, 231.22, -1520.94);	Tcpv_p.push_back(PC8);
//}