#pragma once
#include <iostream>

#include "bundle_adjustment.h"

using namespace std;

namespace RSCalibration
{
	class BAManager
	{
	private:
		BALProblem bal_problem;
		map<string, Mat> camera_intrinsics_map, dist_coeffs_map;

	public:
		BAManager(const map<string, Mat>& camera_intrinsics_map, const map<string, Mat>& dist_coeffs_map);
		void StartBA();
		void Write();
	};
}