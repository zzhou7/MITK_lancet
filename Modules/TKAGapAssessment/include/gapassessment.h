#include <vector>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include "vtkDataSet.h"
#include "vtkPolydata.h"
#include <map>

namespace lancetAlgorithm 
{
	class AssessmentDataType
	{
	public:
		vtkMatrix4x4* TibiaToFemurMedialTensedPose{ nullptr };
		vtkMatrix4x4* TibiaToFemurLateralTensedPose{ nullptr };
		double MedialGap{ 0 };
		double LateralGap{ 0 };
		bool isMedialValid{ false };
		bool isLateralValid{ false };
	};

	class GapAssessment 
	{
	public:
		GapAssessment();

		~GapAssessment() = default;

		void SetAndUpdataData(vtkMatrix4x4* pose, double flexion, vtkPolyData* set, double medial[3], double lateral[3], double normal[3]);

	   /**
		*\brief record pose from tibia to femur under the certain limb flexion
		*
		*@param pose [Input] current pose from tibia to femur 
		*@param flexion [Input] current limb flexion
	    */
		void SetCurrentTibiaToFemurPose(vtkMatrix4x4* pose,double flexion);

		/**
		*\brief calculate current medial gap and lateral gap based on femur implant polydata and
		*two contact points on tibia insert and the normal of the tibia insert
		*
		*@param set [Input] femur implant polydata
		*@param medial [Input] tibia insert medial contact point 
		*@param lateral [Input] tibia insert lateral contact point
		*@param normal [Input] tibia insert normal
		*/
		bool CalculateGap(vtkPolyData* set,double medial[3],double lateral[3],double normal[3]);

		/**
		*\brief compare current gap with the stored gap under the same limb flexion, 
		*then add or update the data depends on the comparison
		*
		*/
		void UpdateResult();

		int DoubleToInt(double dValue);

		/**
		*\brief get stored medial gap under the given limb flexion,return false if the data can not be found
		*
		*@param angle [Input] limb flexion
		*@param gap [Output] medial gap
		*/
		bool GetMedialGap(int angle,double &gap);

		/**
		*\brief get stored lateral gap under the given limb flexion,return false if the data can not be found
		*
		*@param angle [Input] limb flexion	
		*@param gap [Output] lateral gap
		*/
		bool GetLateralGap(int angle,double &gap);

		void ClearAllData();

		bool ClearData(int angle);
	private:
		std::map<int, AssessmentDataType> m_map_assessment_data{};

		vtkSmartPointer<vtkMatrix4x4> m_CurrentTibiaToFemurPose;
		//std::unique_ptr<double[], std::default_delete<double[]>> m_tCurrentTibiaToFemurPose;
		double m_CurrentLimbFlexion{ 0 };
		double m_CurrentMedialGap{ 0 };
		double m_CurrentLateralGap{ 0 };
		bool isMedialValid{ false };
		bool isLateralValid{ false };
	};

}