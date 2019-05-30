#include "bundle_adjustment.h"

namespace RSCalibration
{
	BALProblem::~BALProblem()
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

	int BALProblem::num_cameras() const
	{
		return num_cameras_;
	}

	int BALProblem::num_observations() const
	{
		return num_observations_;
	}

	int BALProblem::num_observations_per_time_camera(int time_idx, int camera_idx) const
	{
		return num_observations_per_time_camera_[time_idx][camera_idx] * 4;
	}

	const double* BALProblem::observations() const
	{
		return observations_;
	}

	int BALProblem::num_parameters() const
	{
		return num_parameters_;
	}

	const double* BALProblem::parameters() const
	{
		return parameters_;
	}

	int BALProblem::num_times() const
	{
		return num_times_;
	}

	int BALProblem::camera_idx(int observation_id) const
	{
		return camera_index_[observation_id];
	}

	int BALProblem::marker_idx(int observation_id) const
	{
		return marker_index_[observation_id];
	}

	double* BALProblem::camera_parameters(int camera_idx)
	{
		return parameters_ + 6 * camera_idx;
	}

	double* BALProblem::marker_transform(int marker_idx)
	{
		return parameters_ + 6 * num_cameras_ + 6 * num_times_ + 6 * marker_idx;
	}

	double* BALProblem::mutable_camera_transform_from_base_camera(int observation_idx)
	{
		return parameters_ + 6 * camera_index_[observation_idx];
	}

	double* BALProblem::mutable_base_marker_transform_from_base_camera(int observation_idx)
	{
		return parameters_ + 6 * num_cameras_ + 6 * time_index_[observation_idx];
	}

	double* BALProblem::mutable_marker_transform_from_base_marker(int observation_idx)
	{
		return parameters_ + 6 * num_cameras_ + 6 * num_times_ + 6 * marker_index_[observation_idx];
	}
	
	void BALProblem::getPoint3dCoordinates(vector<Point3d>& points)
	{
		for (int i = 0; i < num_observations(); i++)
		{
			double* base_marker_transform = mutable_base_marker_transform_from_base_camera(i);
			double* marker_transform = mutable_marker_transform_from_base_marker(i);

			double marker_points[4][3];
			double half_marker_side = MARKER_SIDE / 2;
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
				AngleAxisRotatePoint<double>(marker_transform, marker_points[j], p);
				p[0] += marker_transform[3];
				p[1] += marker_transform[4];
				p[2] += marker_transform[5];

				AngleAxisRotatePoint<double>(base_marker_transform, p, p);
				p[0] += base_marker_transform[3];
				p[1] += base_marker_transform[4];
				p[2] += base_marker_transform[5];

				Point3d point(p[0], p[1], p[2]);

				points.emplace_back(point);
			}

		}
	}

	bool BALProblem::loadFile(const char* filename)
	{
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL)
		{
			return false;
		}

		fscanf(fptr, "%d", &num_times_);
		fscanf(fptr, "%d", &num_cameras_);
		fscanf(fptr, "%d", &num_markers_);
		fscanf(fptr, "%d", &num_observations_);

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
			fscanf(fptr, "%d", &tmp);

			for (int camera_idx = 0; camera_idx < num_cameras_; camera_idx++)
			{
				fscanf(fptr, "%d", &num_observations_per_time_camera_[time_idx][camera_idx]);
			}
		}

		for (int i = 0; i < num_observations_; i++)
		{
			fscanf(fptr, "%d", time_index_ + i);
			fscanf(fptr, "%d", camera_index_ + i);
			fscanf(fptr, "%d", marker_index_ + i);

			for (int j = 0; j < 8; j++)
			{
				fscanf(fptr, "%lf", observations_ + 8 * i + j);
			}
		}

		for (int i = 0; i < num_parameters_; i++)
		{
			fscanf(fptr, "%lf", parameters_ + i);
		}

		return true;
	}
}