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
	double observed_x;
	double observed_y;
	double fx, fy, ppx, ppy;
	Mat dist_coeffs;

	ReprojectionError(const double observed_x, const double observed_y, const Mat& intrinsics, const Mat& dist_coeffs)
		: observed_x(observed_x), observed_y(observed_y), dist_coeffs(dist_coeffs) 
	{
		fx = intrinsics.at<double>(0, 0);
		fy = intrinsics.at<double>(1, 1);
		ppx = intrinsics.at<double>(0, 2);
		ppy = intrinsics.at<double>(1, 2);
	}
	
	template <typename T>
	bool operator()(const T* const camera, const T* const point, T* residuals) const 
	{
		//cout << "camera_r: " << camera[0] << ", " << camera[1] << ", " << camera[2] << endl;
		//cout << "camera_t: " << camera[3] << ", " << camera[4] << ", " << camera[5] << endl;
		//cout << "point: " << point[0] << ", " << point[1] << ", " << point[2] << endl;

		T p[3];
		AngleAxisRotatePoint(camera, point, p);
		p[0] += camera[3];
		p[1] += camera[4];
		p[2] += camera[5];

		T xp = T(fx) * p[0] / p[2] + T(ppx);
		T yp = T(fy) * p[1] / p[2] + T(ppy);

		//FIXME Distortion
		//cout << "ox: " << observed_x << " px: " << xp << " oy: " << observed_y << " py: " << yp << endl;
		
		residuals[0] = xp - T(observed_x);
		residuals[1] = yp - T(observed_y);

		return true;
	}
	
	static ceres::CostFunction* Create(const double observed_x, const double observed_y, const Mat& intrinsics, const Mat& dist_coeffs)
	{
		return (new AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
			new ReprojectionError(observed_x, observed_y, intrinsics, dist_coeffs)));
	}
};