#include "main.h"

int main(int argc, char** argv)
{
	//initialization
	map<string, Mat> camera_intrinsics_map, dist_coeffs_map;
	IO::GetIntrinsics(camera_intrinsics_map, dist_coeffs_map);

	map<int, Transform> marker_transforms_from_base, marker_transforms_from_camera;
	IO::GetMarkerGeometry(marker_transforms_from_base);
	
	//Get Correspondence
	vector<map<string, Mat>> images(TIMES);
	vector<Vec3d> base_rvecs(TIMES), base_tvecs(TIMES);
	vector<vector<Point3d>> object_points(CAMERAS);
	vector<vector<Point2f>> image_points(CAMERAS);
	vector<vector<vector<Point2f>>> image_points_per_time(TIMES, vector<vector<Point2f>>(CAMERAS));
	vector<vector<vector<Observation>>> observations(TIMES, vector<vector<Observation>>(CAMERAS));
	Correspondencer correspondencer = Correspondencer(camera_intrinsics_map, dist_coeffs_map);
	correspondencer.GetCorrespondencePoints(images, object_points, image_points, image_points_per_time, observations, marker_transforms_from_camera, marker_transforms_from_base, base_rvecs, base_tvecs);
	
	vector<Vec3d> camera_rvecs(CAMERAS), camera_tvecs(CAMERAS);
	correspondencer.CalculateTransforms(object_points, image_points, camera_rvecs, camera_tvecs);
	correspondencer.Write(observations, camera_rvecs, camera_tvecs, base_rvecs, base_tvecs, marker_transforms_from_base);
	//correspondencer.ReprojectionCheck(images, object_points, image_points, observations, camera_rvecs, camera_tvecs);
	/*
	while (true)
	{
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}*/
	//Bundle Adjustment
	google::InitGoogleLogging(argv[0]);
	
	BAManager ba_manager = BAManager(camera_intrinsics_map, dist_coeffs_map);
	ba_manager.StartBA();
	ba_manager.Write();


	//ReprojectionCheck
	ReprojectionCheck::Reproject(images, image_points_per_time, camera_intrinsics_map, dist_coeffs_map);
	
	while (true)
	{
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}

