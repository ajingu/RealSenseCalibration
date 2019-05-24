#pragma warning(disable: 4996)

#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/opencv.hpp>

#define MARKER_SIDE 0.048

using namespace std;
using namespace cv;
using namespace ceres;


class BALProblem
{
public:
	~BALProblem()
	{
		delete[] time_index_;
		delete[] camera_index_;
		delete[] marker_index_;
		delete[] observations_;
		delete[] parameters_;

		for (int i = 0; i < num_times_; i++) {
			delete[] num_observations_per_time_camera_[i];
		}
		delete[] num_observations_per_time_camera_;
	}

	int num_cameras() const
	{
		return num_cameras_;
	}

	int num_observations() const
	{
		return num_observations_;
	}

	int num_observations_per_time_camera(int time_idx, int camera_idx) const
	{
		return num_observations_per_time_camera_[time_idx][camera_idx] * 4;
	}

	const double* observations() const
	{
		return observations_;
	}

	int num_parameters() const
	{
		return num_parameters_;
	}

	const double* parameters() const
	{
		return parameters_;
	}

	int num_times() const
	{
		return num_times_;
	}

	int camera_idx(int observation_id) const
	{
		return camera_index_[observation_id];
	}

	double* camera_parameters(int camera_idx)
	{
		return parameters_ + 6 * camera_idx;
	}

	double* mutable_camera_transform_from_base_camera(int observation_idx)
	{
		return parameters_ + 6 * camera_index_[observation_idx];
	}

	double* mutable_base_marker_transform_from_base_camera(int observation_idx)
	{
		return parameters_ + 6 * num_cameras_ + 6 * time_index_[observation_idx];
	}

	double* mutable_marker_transform_from_base_marker(int observation_idx)
	{
		return parameters_ + 6 * num_cameras_ + 6 * num_times_ + 6 * marker_index_[observation_idx];
	}

	void getPoint3dCoordinates(vector<Point3d>& points)
	{
		for (int i = 0; i < num_observations(); i++)
		{
			double* base_marker_transform = mutable_base_marker_transform_from_base_camera(i);
			double* marker_transform = mutable_marker_transform_from_base_marker(i);

			double marker_points[4][3];
			double half_marker_side = MARKER_SIDE/2;
			marker_points[0][0] = -half_marker_side;
			marker_points[0][1] = half_marker_side;
			marker_points[0][2] = 0;
			marker_points[1][0] = half_marker_side;
			marker_points[1][1] = half_marker_side;
			marker_points[1][2] = 0;
			marker_points[2][0] = half_marker_side;
			marker_points[2][1] = -half_marker_side;
			marker_points[2][2] = 0;
			marker_points[3][0] = -half_marker_side;
			marker_points[3][1] = -half_marker_side;
			marker_points[3][2] = 0;

			for (int j = 0; j < 4; j++)
			{
				double p[3];
				AngleAxisRotatePoint(marker_transform, marker_points[j], p);
				p[0] += marker_transform[3];
				p[1] += marker_transform[4];
				p[2] += marker_transform[5];

				AngleAxisRotatePoint(base_marker_transform, p, p);
				p[0] += base_marker_transform[3];
				p[1] += base_marker_transform[4];
				p[2] += base_marker_transform[5];

				Point3d point(p[0], p[1], p[2]);

				points.emplace_back(point);
			}
			
		}
	}
	
	bool loadFile(const char* filename)
	{
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL)
		{
			return false;
		}

		fscanfOrDie(fptr, "%d", &num_times_);
		fscanfOrDie(fptr, "%d", &num_cameras_);
		fscanfOrDie(fptr, "%d", &num_markers_);
		fscanfOrDie(fptr, "%d", &num_observations_);

		time_index_ = new int[num_observations_];
		camera_index_ = new int[num_observations_];
		marker_index_ = new int[num_observations_];
		observations_ = new double[8 * num_observations_];
		num_observations_per_time_camera_ = new int*[num_times_];
		for (int i = 0; i < num_times_; i++)
		{
			num_observations_per_time_camera_[i] = new int[num_cameras_];
		}
		
		num_parameters_ = 6 * num_cameras_ + 6 * num_times_ + 6 * num_markers_;
		parameters_ = new double[num_parameters_];

		for (int time_idx = 0; time_idx < num_times_; time_idx++)
		{
			int tmp;
			fscanfOrDie(fptr, "%d", &tmp);

			for (int camera_idx = 0; camera_idx < num_cameras_; camera_idx++)
			{
				fscanfOrDie(fptr, "%d", &num_observations_per_time_camera_[time_idx][camera_idx]);
			}
		}
		
		for (int i = 0; i < num_observations_; i++)
		{
			fscanfOrDie(fptr, "%d", time_index_ + i);
			fscanfOrDie(fptr, "%d", camera_index_ + i);
			fscanfOrDie(fptr, "%d", marker_index_ + i);
			
			for (int j = 0; j < 8; j++)
			{
				fscanfOrDie(fptr, "%lf", observations_ + 8 * i + j);
			}
		}

		for (int i = 0; i < num_parameters_; i++)
		{
			fscanfOrDie(fptr, "%lf", parameters_ + i);
		}

		return true;
	}
private:
	template<typename T>
	void fscanfOrDie(FILE *fptr, const char *format, T *value)
	{
		int num_scanned = fscanf(fptr, format, value);
		if (num_scanned != 1)
		{
			LOG(FATAL) << "Invalid UW data file.";
		}
	}
	
	int num_times_;
	int num_cameras_;
	int num_markers_;
	int num_observations_;
	int num_parameters_;
	int* time_index_;
	int* camera_index_;
	int* marker_index_;
	int** num_observations_per_time_camera_;
	double* observations_;
	double* parameters_;
};

struct TargetCameraReprojectionError
{
	const double* const observations;
	double half_marker_side;
	double fx, fy, ppx, ppy;
	Mat dist_coeffs;

	TargetCameraReprojectionError(const double* const observations, const double marker_side, const Mat& intrinsics, const Mat& dist_coeffs)
		: observations(observations), dist_coeffs(dist_coeffs)
	{
		fx = intrinsics.at<double>(0, 0);
		fy = intrinsics.at<double>(1, 1);
		ppx = intrinsics.at<double>(0, 2);
		ppy = intrinsics.at<double>(1, 2);

		half_marker_side = marker_side / 2;
	}
	
	template <typename T>
	bool operator()(const T* const camera_transform, const T* const base_marker_transform_from_camera, const T* const marker_transform_from_base_marker, T* residuals) const
	{
		T marker_points[4][3];
		marker_points[0][0] = T(-half_marker_side);
		marker_points[0][1] = T(half_marker_side);
		marker_points[0][2] = T(0);
		marker_points[1][0] = T(half_marker_side);
		marker_points[1][1] = T(half_marker_side);
		marker_points[1][2] = T(0);
		marker_points[2][0] = T(half_marker_side);
		marker_points[2][1] = T(-half_marker_side);
		marker_points[2][2] = T(0);
		marker_points[3][0] = T(-half_marker_side);
		marker_points[3][1] = T(-half_marker_side);
		marker_points[3][2] = T(0);

		for (int i = 0; i < 4; i++)
		{
			T p[3];
			

			//coordinate on base marker
			AngleAxisRotatePoint(marker_transform_from_base_marker, marker_points[i], p);
			p[0] += marker_transform_from_base_marker[3];
			p[1] += marker_transform_from_base_marker[4];
			p[2] += marker_transform_from_base_marker[5];

			//coordinate on base camera
			AngleAxisRotatePoint(base_marker_transform_from_camera, p, p);
			p[0] += base_marker_transform_from_camera[3];
			p[1] += base_marker_transform_from_camera[4];
			p[2] += base_marker_transform_from_camera[5];

			//coordinate on target camera
			AngleAxisRotatePoint(camera_transform, p, p);
			p[0] += camera_transform[3];
			p[1] += camera_transform[4];
			p[2] += camera_transform[5];

			T xp = T(fx) * p[0] / p[2] + T(ppx);
			T yp = T(fy) * p[1] / p[2] + T(ppy);

			//Distortion is not considered in this case
			//because D400's default distortion coefficients are all 0.

			residuals[i*2]   = xp - T(observations[i*2]);
			residuals[i*2+1] = yp - T(observations[i*2+1]);
		}

		return true;
	}

	static CostFunction* create(const double* const observations, const double marker_side, const Mat& intrinsics, const Mat& dist_coeffs)
	{
		return (new AutoDiffCostFunction<TargetCameraReprojectionError, 8, 6, 6, 6>(
			new TargetCameraReprojectionError(observations, marker_side, intrinsics, dist_coeffs)));
	}
};

struct BaseCameraReprojectionError
{
	const double* const observations;
	double half_marker_side;
	double fx, fy, ppx, ppy;
	Mat dist_coeffs;

	BaseCameraReprojectionError(const double* const observations, const double marker_side, const Mat& intrinsics, const Mat& dist_coeffs)
		: observations(observations), dist_coeffs(dist_coeffs)
	{
		fx = intrinsics.at<double>(0, 0);
		fy = intrinsics.at<double>(1, 1);
		ppx = intrinsics.at<double>(0, 2);
		ppy = intrinsics.at<double>(1, 2);

		half_marker_side = marker_side / 2;
	}

	template <typename T>
	bool operator()(const T* const base_marker_transform_from_camera, const T* const marker_transform_from_base_marker, T* residuals) const
	{
		T marker_points[4][3];
		
		marker_points[0][0] = T(-half_marker_side);
		marker_points[0][1] = T(half_marker_side);
		marker_points[0][2] = T(0);
		marker_points[1][0] = T(half_marker_side);
		marker_points[1][1] = T(half_marker_side);
		marker_points[1][2] = T(0);
		marker_points[2][0] = T(half_marker_side);
		marker_points[2][1] = T(-half_marker_side);
		marker_points[2][2] = T(0);
		marker_points[3][0] = T(-half_marker_side);
		marker_points[3][1] = T(-half_marker_side);
		marker_points[3][2] = T(0);

		for (int i = 0; i < 4; i++)
		{
			T p[3];


			//coordinate on base marker
			AngleAxisRotatePoint(marker_transform_from_base_marker, marker_points[i], p);
			p[0] += marker_transform_from_base_marker[3];
			p[1] += marker_transform_from_base_marker[4];
			p[2] += marker_transform_from_base_marker[5];

			//coordinate on base camera
			AngleAxisRotatePoint(base_marker_transform_from_camera, p, p);
			p[0] += base_marker_transform_from_camera[3];
			p[1] += base_marker_transform_from_camera[4];
			p[2] += base_marker_transform_from_camera[5];

			T xp = T(fx) * p[0] / p[2] + T(ppx);
			T yp = T(fy) * p[1] / p[2] + T(ppy);

			//Distortion is not considered in this case
			//because D400's default distortion coefficients are all 0.

			residuals[i * 2] = xp - T(observations[i * 2]);
			residuals[i * 2 + 1] = yp - T(observations[i * 2 + 1]);
		}

		return true;
	}

	static CostFunction* create(const double* const observations, const double marker_side, const Mat& intrinsics, const Mat& dist_coeffs)
	{
		return (new AutoDiffCostFunction<BaseCameraReprojectionError, 8, 6, 6>(
			new BaseCameraReprojectionError(observations, marker_side, intrinsics, dist_coeffs)));
	}
};