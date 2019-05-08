#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct SnavelyReprojectionError {
	SnavelyReprojectionError(float observed_x, float observed_y, float predicted_x, float predicted_y)
		: observed_x(observed_x), observed_y(observed_y), predicted_x(predicted_x), predicted_y(predicted_y) {}
	
	template <typename T>
	bool operator()(const T* const camera,
		const T* const point,
		T* residuals) const {
		
		residuals[0] = predicted_x - observed_x;
		residuals[1] = predicted_y - observed_y;
		return true;
	}
	
	static ceres::CostFunction* Create(const float observed_x, 
		const float observed_y, const float predicted_x, const float predicted_y) {

		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 3>(
			new SnavelyReprojectionError(observed_x, observed_y, predicted_x, predicted_y)));
	}
	float observed_x;
	float observed_y;
	float predicted_x;
	float predicted_y;
};