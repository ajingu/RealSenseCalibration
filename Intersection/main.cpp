#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	vector<int> a{ 1, 2, 3, 4, 5 };
	vector<int> b{ 3, 5, 6 };
	vector<int> result;
	vector<int> match_index_a, match_index_b;

	set_intersection(
		a.begin(), a.end(),
		b.begin(), b.end(),
		inserter(result, result.end()));

	cout << "result: ";
	for_each(result.begin(), result.end(), [](int i) {
		cout << i << " ";
	});
	cout << endl;

	for (int i = 0; i < a.size(); i++)
	{
		if (binary_search(result.begin(), result.end(), a[i]))
		{
			match_index_a.push_back(i);
		}
	}

	for (int i = 0; i < b.size(); i++)
	{
		if (binary_search(result.begin(), result.end(), b[i]))
		{
			match_index_b.push_back(i);
		}
	}

	cout << "a: ";
	for_each(match_index_a.begin(), match_index_a.end(), [](int i) {
		cout << i << " ";
	});
	cout << endl;

	cout << "b: ";
	for_each(match_index_b.begin(), match_index_b.end(), [](int i) {
		cout << i << " ";
	});
	cout << endl;

	while (true)
	{
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}

	return 0;
}