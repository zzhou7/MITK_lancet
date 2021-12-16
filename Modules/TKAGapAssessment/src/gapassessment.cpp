#include "gapassessment.h"
#include <iostream>
#include "mitkPoint.h"
#include "vtkSmartPointer.h"
#include "vtkOBBTree.h"
#include "vtkPoints.h"

lancetAlgorithm::GapAssessment::GapAssessment()
{
	m_CurrentTibiaToFemurPose = vtkSmartPointer<vtkMatrix4x4>::New();
}

void lancetAlgorithm::GapAssessment::SetAndUpdataData(vtkMatrix4x4 * pose, double flexion, vtkPolyData * set, double medial[3], double lateral[3], double normal[3])
{
	SetCurrentTibiaToFemurPose(pose, flexion);
	if (CalculateGap(set, medial, lateral, normal))
	{
		UpdateResult();
	}
	else
	{
		MITK_INFO << "current data is invalid";
		return;
	}
}

void lancetAlgorithm::GapAssessment::SetCurrentTibiaToFemurPose(vtkMatrix4x4* pose, double flexion)
{
	if (pose == nullptr)
	{
		MITK_INFO << "TibiaToFemurPose is nullptr";
		return;
	}
	m_CurrentTibiaToFemurPose->DeepCopy(pose);
	m_CurrentLimbFlexion = flexion;
}

bool lancetAlgorithm::GapAssessment::CalculateGap(vtkPolyData *set, double medial[3], double lateral[3], double normal[3])
{
  if (!set)
  {
    MITK_INFO << "femur implant model is nullptr";
    return false;
  }

  mitk::Point3D MedialPoint{ medial };
  mitk::Point3D LateralPoint{ lateral };

  // Create the locator
  vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
  tree->SetDataSet(set);
  tree->BuildLocator();

  double len = 100;

  double medial1[3] = { medial[0] - normal[0] * len,medial[1] - normal[1] * len,medial[2] - normal[2] * len };
  double medial2[3] = { medial[0] + normal[0] * len,medial[1] + normal[1] * len,medial[2] + normal[2] * len };
  vtkSmartPointer<vtkPoints> intersectPointsMedial =
	  vtkSmartPointer<vtkPoints>::New();
  if (tree->IntersectWithLine(medial1, medial2, intersectPointsMedial, NULL) != 0)
  {
	  auto p = intersectPointsMedial->GetPoint(0);
	  double Mdir[3] = { p[0] - medial[0] , p[1] - medial[1], p[2] - medial[2] };
	  m_CurrentMedialGap = sqrt(Mdir[0] * Mdir[0] + Mdir[1] * Mdir[1] + Mdir[2] * Mdir[2]);
	  double ang = Mdir[0] * normal[0] + Mdir[1] * normal[1] + Mdir[2] * normal[2];
	  if (ang < 0)
		  m_CurrentMedialGap = -m_CurrentMedialGap;
	  if (m_CurrentMedialGap > 5)
	  {
		  isMedialValid = false;
	  }
	  else
	  {
		  isMedialValid = true;
	  }
	  MITK_INFO << "medial gap:" << m_CurrentMedialGap;
  }
  else
  {
	  MITK_INFO << "medial not intersected";
	  isMedialValid = false;
	  //没相交
  }

  double latera11[3] = { lateral[0] - normal[0] * len,lateral[1] - normal[1] * len,lateral[2] - normal[2] * len };
  double lateral2[3] = { lateral[0] + normal[0] * len,lateral[1] + normal[1] * len,lateral[2] + normal[2] * len };
  vtkSmartPointer<vtkPoints> intersectPointsLateral =
	  vtkSmartPointer<vtkPoints>::New();
  if (tree->IntersectWithLine(latera11, lateral2, intersectPointsLateral, NULL) != 0)
  {
	  auto p = intersectPointsLateral->GetPoint(0);
	  double Ldir[3] = { p[0] - lateral[0] ,p[1] - lateral[1] ,p[2] - lateral[2] };
	  m_CurrentLateralGap = sqrt(Ldir[0] * Ldir[0] + Ldir[1] * Ldir[1] + Ldir[2] * Ldir[2]);
	  double ang = Ldir[0] * normal[0] + Ldir[1] * normal[1] + Ldir[2] * normal[2];
	  if (ang < 0)
		  m_CurrentLateralGap = -m_CurrentLateralGap;
	  if (m_CurrentLateralGap > 5)
	  {
		  isLateralValid = false;
	  }
	  else
	  {
		  isLateralValid = true;
	  }
	  
	  MITK_INFO << "lateral gap:" << m_CurrentLateralGap;
  }
  else
  {
	  MITK_INFO << "lateral not intersected";
	  isLateralValid = false;
	  //没相交
  }
  return true;
 
}

void lancetAlgorithm::GapAssessment::UpdateResult()
{
	int val = DoubleToInt(m_CurrentLimbFlexion);
	auto iter = m_map_assessment_data.find(val);
	if (iter != m_map_assessment_data.end())
	{
		AssessmentDataType data = iter->second;
		if (isMedialValid && data.isMedialValid)
		{
			if (data.MedialGap < m_CurrentMedialGap)
			{
				data.MedialGap = m_CurrentMedialGap;
				data.TibiaToFemurMedialTensedPose = m_CurrentTibiaToFemurPose;
			}
		}
		else if (isMedialValid && !data.isMedialValid)
		{
			data.MedialGap = m_CurrentMedialGap;
			data.TibiaToFemurMedialTensedPose = m_CurrentTibiaToFemurPose;
			data.isMedialValid = true;
		}

		if (isLateralValid && data.isLateralValid)
		{
			if (data.LateralGap < m_CurrentLateralGap)
			{
				data.LateralGap = m_CurrentLateralGap;
				data.TibiaToFemurLateralTensedPose = m_CurrentTibiaToFemurPose;
			}
		}
		if (isLateralValid && !data.isLateralValid)
		{
			data.LateralGap = m_CurrentLateralGap;
			data.TibiaToFemurLateralTensedPose = m_CurrentTibiaToFemurPose;
			data.isLateralValid = true;
		}
		m_map_assessment_data.insert_or_assign(val, data);
	}
	else
	{
		AssessmentDataType data{};
		data.MedialGap = m_CurrentMedialGap;
		data.LateralGap = m_CurrentLateralGap;
		data.TibiaToFemurMedialTensedPose = m_CurrentTibiaToFemurPose;
		data.TibiaToFemurLateralTensedPose = m_CurrentTibiaToFemurPose;
		data.isMedialValid = isMedialValid;
		data.isLateralValid = isLateralValid;
		m_map_assessment_data.insert_or_assign(val, data);
	}
}



int lancetAlgorithm::GapAssessment::DoubleToInt(double dValue)
{
	if (dValue < 0.0)
		return static_cast<int>(dValue - 0.5);
	else
		return static_cast<int>(dValue + 0.5);

	return 0;
}

bool lancetAlgorithm::GapAssessment::GetMedialGap(int angle, double &gap)
{
	auto iter = m_map_assessment_data.find(angle);
	if (iter != m_map_assessment_data.end())
	{
		AssessmentDataType data = iter->second;
		if (data.isMedialValid)
		{
			gap = data.MedialGap;
			return true;
		}
		else
		{
			MITK_INFO << "medial gap is invalid";
			return false;
		}
	}
	else
	{
		MITK_INFO << "Error: Cant find gap information:" << std::to_string(angle) << "degree" << endl;
		return false;
	}
}

bool lancetAlgorithm::GapAssessment::GetLateralGap(int angle, double & gap)
{
	auto iter = m_map_assessment_data.find(angle);
	if (iter != m_map_assessment_data.end())
	{
		AssessmentDataType data = iter->second;
		if (data.isLateralValid)
		{
			gap = data.LateralGap;
			return true;
		}
		else
		{
			MITK_INFO << "lateral gap is invalid";
			return false;
		}
	}
	else
	{
		MITK_INFO << "Error: Cant find gap information:" << std::to_string(angle) << "degree" << endl;
		return false;
	}
}

void lancetAlgorithm::GapAssessment::ClearAllData()
{
	if (!m_map_assessment_data.empty())
	{
		m_map_assessment_data.clear();
	}
}

bool lancetAlgorithm::GapAssessment::ClearData(int angle)
{
	auto iter = m_map_assessment_data.find(angle);
	if (iter != m_map_assessment_data.end())
	{
		m_map_assessment_data.erase(iter);
		return true;
	}
	MITK_INFO << "Error: Cant find gap information:" << std::to_string(angle) << "degree" << std::endl;
	return false;
}
