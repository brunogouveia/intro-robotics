#include "Knn.h"

void Knn::pointsInBall(PointCloud<PointXYZ> & cloud, Eigen::Vector4f & center, float r,  std::vector<int> & inds)
{
	float r2 = r*r;
	float tinyR = r/sqrt(3);
	Eigen::Vector3f diff;

	inds.clear(); //Make sure the vector is empty;
	for (int i = 0; i < cloud.points.size(); i++)
	{
		diff(0) = fabs(center(0) - cloud.points[i].x);
		diff(1) = fabs(center(1) - cloud.points[i].y);
		diff(2) = fabs(center(2) - cloud.points[i].z);

		//Quick comparison to ignore a bunch of points
		if (diff(0) < r && diff(1) < r && diff(2) < r)
		{
			//If it's inside the cube with tinyR, definitely it's inside the ball
			if ((diff(0) < tinyR && diff(1) < tinyR && diff(2) < tinyR) || (diff.norm() < r2))
				inds.push_back(i);
		}
	}
}

Eigen::Vector4f Knn::pointsInBallWithClosest(PointCloud<PointXYZ> & cloud, Eigen::Vector4f & center, float r,  std::vector<int> & inds)
{
	float r2 = r*r;
	Eigen::Vector3f diff;

	float closestDist = 10 * r;
	Eigen::Vector4f closestPoint;

	inds.clear(); //Make sure the vector is empty;
	for (int i = 0; i < cloud.points.size(); i++)
	{
		diff(0) = fabs(center(0) - cloud.points[i].x);
		diff(1) = fabs(center(1) - cloud.points[i].y);
		diff(2) = fabs(center(2) - cloud.points[i].z);

		//Quick comparison to ignore a bunch of points
		if (diff(0) < r && diff(1) < r && diff(2) < r)
		{
			double norm = diff.norm();
			if (norm < r2)
			{
				inds.push_back(i);
				if (norm < closestDist)
				{
					closestDist = norm;
					closestPoint(0) = cloud.points[i].x;
					closestPoint(1) = cloud.points[i].y;
					closestPoint(2) = cloud.points[i].z;
				}
			}
		}
	}
	return closestPoint;
}