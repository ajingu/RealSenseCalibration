#pragma warning(disable: 4996)

#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace ceres;

class BALProblem 
{
public:
	~BALProblem() 
	{
		delete[] point_index_;
		delete[] camera_index_;
		delete[] observations_;
		delete[] parameters_;
	}

	int num_observations() const 
	{ 
		return num_observations_; 
	}

	const double* observations() const 
	{ 
		return observations_; 
	}

	double* mutable_cameras() 
	{ 
		return parameters_; 
	}
	
	double* mutable_points() 
	{ 
		return parameters_ + 6 * num_cameras_; 
	}

	double* mutable_camera_for_observation(int i) 
	{
		return mutable_cameras() + camera_index_[i] * 6;
	}

	double* mutable_point_for_observation(int i) 
	{
		return mutable_points() + point_index_[i] * 3;
	}

	bool LoadFile(const char* filename) 
	{
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL) 
		{
			return false;
		};
		FscanfOrDie(fptr, "%d", &num_cameras_);
		FscanfOrDie(fptr, "%d", &num_points_);
		//FscanfOrDie(fptr, "%d", &num_observations_);
		num_observations_ = num_points_;
		point_index_ = new int[num_observations_];
		camera_index_ = new int[num_observations_];
		observations_ = new double[2 * num_observations_];
		num_parameters_ = 6 * num_cameras_ + 3 * num_points_;
		parameters_ = new double[num_parameters_];

		for (int i = 0; i < num_observations_; ++i) 
		{
			FscanfOrDie(fptr, "%d", camera_index_ + i);
			FscanfOrDie(fptr, "%d", point_index_ + i);
			for (int j = 0; j < 2; ++j) 
			{
				FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j); //double
			}
		}
		for (int i = 0; i < num_parameters_; ++i) 
		{
			FscanfOrDie(fptr, "%lf", parameters_ + i);
		}
		return true;
	}
private:
	template<typename T>
	void FscanfOrDie(FILE *fptr, const char *format, T *value) 
	{
		int num_scanned = fscanf(fptr, format, value);
		if (num_scanned != 1) 
		{
			LOG(FATAL) << "Invalid UW data file.";
		}
	}
	int num_cameras_;
	int num_points_;
	int num_observations_;
	int num_parameters_;
	int* point_index_;
	int* camera_index_;
	double* observations_;
	double* parameters_;
};

struct ReprojectionError 
{
	ReprojectionError(const double observed_x, const double observed_y, const Mat& intrinsics, const Mat& dist_coeffs)
		: observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), dist_coeffs(dist_coeffs) {}
	
	template <typename T>
	bool operator()(const T* const camera, const T* const point, T* residuals) const 
	{
		Eigen::Matrix<T, 3, 1> object_points, camera_rvec, camera_tvec;
		object_points << point[0], point[1], point[2];
		camera_rvec << camera[0], camera[1], camera[2];
		camera_tvec << camera[3], camera[4], camera[5];


		Eigen::Matrix<T, 2, 1> reprojected_points;
		projectPoints(object_points, camera_rvec, camera_tvec, intrinsics, dist_coeffs, reprojected_points);

		residuals[0] = reprojected_points(0, 0) - T(observed_x);
		residuals[1] = reprojected_points(1, 0) - T(observed_y);

		return true;
	}
	
	static ceres::CostFunction* Create(const double observed_x, const double observed_y, const Mat& intrinsics, const Mat& dist_coeffs)
	{
		return (new AutoDiffCostFunction<ReprojectionError, 2, 3, 3>(
			new ReprojectionError(observed_x, observed_y, intrinsics, dist_coeffs)));
	}
	double observed_x;
	double observed_y;
	Mat intrinsics;
	Mat dist_coeffs;
};