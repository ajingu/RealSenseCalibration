#pragma once
#pragma warning(disable: 4996)

#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

#include "my_const.h"

using namespace std;
using namespace cv;
using namespace ceres;

namespace RSCalibration
{
	class BALProblem
	{
	private:

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

	public:

		~BALProblem();

		int num_cameras() const;
		int num_observations() const;
		int num_observations_per_time_camera(int time_idx, int camera_idx) const;
		const double* observations() const;
		int num_parameters() const;
		const double* parameters() const;
		int num_times() const;
		int camera_idx(int observation_id) const;
		int marker_idx(int observation_id) const;
		double* camera_parameters(int camera_idx);
		double* marker_transform(int marker_idx);
		double* mutable_camera_transform_from_base_camera(int observation_idx);
		double* mutable_base_marker_transform_from_base_camera(int observation_idx);
		double* mutable_marker_transform_from_base_marker(int observation_idx);
		void getPoint3dCoordinates(vector<Point3d>& points);
		bool loadFile(const char* filename);
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

				residuals[i * 2] = xp - T(observations[i * 2]);
				residuals[i * 2 + 1] = yp - T(observations[i * 2 + 1]);
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

	struct TargetCameraBaseMarkerReprojectionError
	{
		const double* const observations;
		double half_marker_side;
		double fx, fy, ppx, ppy;
		Mat dist_coeffs;

		TargetCameraBaseMarkerReprojectionError(const double* const observations, const double marker_side, const Mat& intrinsics, const Mat& dist_coeffs)
			: observations(observations), dist_coeffs(dist_coeffs)
		{
			fx = intrinsics.at<double>(0, 0);
			fy = intrinsics.at<double>(1, 1);
			ppx = intrinsics.at<double>(0, 2);
			ppy = intrinsics.at<double>(1, 2);

			half_marker_side = marker_side / 2;
		}

		template <typename T>
		bool operator()(const T* const camera_transform, const T* const base_marker_transform_from_camera, T* residuals) const
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

				//coordinate on base camera
				AngleAxisRotatePoint(base_marker_transform_from_camera, marker_points[i], p);
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

				residuals[i * 2] = xp - T(observations[i * 2]);
				residuals[i * 2 + 1] = yp - T(observations[i * 2 + 1]);
			}

			return true;
		}

		static CostFunction* create(const double* const observations, const double marker_side, const Mat& intrinsics, const Mat& dist_coeffs)
		{
			return (new AutoDiffCostFunction<TargetCameraBaseMarkerReprojectionError, 8, 6, 6>(
				new TargetCameraBaseMarkerReprojectionError(observations, marker_side, intrinsics, dist_coeffs)));
		}
	};

	struct BaseCameraBaseMarkerReprojectionError
	{
		const double* const observations;
		double half_marker_side;
		double fx, fy, ppx, ppy;
		Mat dist_coeffs;

		BaseCameraBaseMarkerReprojectionError(const double* const observations, const double marker_side, const Mat& intrinsics, const Mat& dist_coeffs)
			: observations(observations), dist_coeffs(dist_coeffs)
		{
			fx = intrinsics.at<double>(0, 0);
			fy = intrinsics.at<double>(1, 1);
			ppx = intrinsics.at<double>(0, 2);
			ppy = intrinsics.at<double>(1, 2);

			half_marker_side = marker_side / 2;
		}

		template <typename T>
		bool operator()(const T* const base_marker_transform_from_camera, T* residuals) const
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


				//coordinate on base camera
				AngleAxisRotatePoint(base_marker_transform_from_camera, marker_points[i], p);
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
			return (new AutoDiffCostFunction<BaseCameraBaseMarkerReprojectionError, 8, 6>(
				new BaseCameraBaseMarkerReprojectionError(observations, marker_side, intrinsics, dist_coeffs)));
		}
	};
}