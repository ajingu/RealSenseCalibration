#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
	std::vector<int> array = { 3, 2, 5, 7, 1 };

	// �z��̃C���f�b�N�X indiecs = {0, 1, 2, 3, 4} ���쐬����B
	std::vector<size_t> indices(array.size());
	std::iota(indices.begin(), indices.end(), 0);

	// �\�[�g����B
	std::sort(indices.begin(), indices.end(), [&array](size_t i1, size_t i2) {
		return array[i1] < array[i2];
	});

	for (auto v : indices)
		std::cout << v << " ";
	std::cout << std::endl;

	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}
}