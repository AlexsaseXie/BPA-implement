#include "BPA.h"

#include <Eigen/src/Core/Matrix.h>

#define IN_BALL_THRESHOLD	1e-7

void BPA::do_bpa(pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PolygonMesh &mesh) {
	init(cloud, mesh);

	while (true) {
		EdgePtr eij;

		while (eij = get_active_edge()) {
			TrianglePtr tri = ball_pivot(eij);

			if (tri == NULL) {
				mark_as_boundary(eij);
				continue;
			}

			PointData& ek_data = tri->points[1];
			int ek_index = ek_data.second;
			PointNormal &ek = *ek_data.first;

			if (ek_index >= 0 && (pt_used[ek_index] == 0 || F.on_front(ek_data))) {
				output_triangle(eij->vertices[0], eij->vertices[1], ek_data);

				// join & glue
				F.join_glue(eij, ek_data, tri, pt_used[ek_index] == 1);

				// set new point used
				this->set_used(ek_index);
			}
			else {
				mark_as_boundary(eij);
			}
		}

		F.clear();

		auto new_tri_data = find_seed_triangle();

		if (new_tri_data.second) {
			Triangle & new_tri = new_tri_data.first;

			output_triangle(new_tri.points[0], new_tri.points[1], new_tri.points[2]);

			F.insert_triangle_edges(new_tri);
		}
		else {
			break;
		}
	}
}

EdgePtr BPA::get_active_edge() {
	return F.get_first_active();
}

void BPA::mark_as_boundary(EdgePtr ek) {
	ek->active = false;
}

TrianglePtr BPA::ball_pivot(EdgePtr eij) {
	PointData v0 = eij->vertices[0];
	PointData v1 = eij->vertices[1];
	PointData op = eij->opposite_vertex;

	PointNormal middle_point = eij->middle_point;
	PointNormal ball_center = eij->ball_center;


	// create plane _|_ (v0 - v1) (the new ball center must be on this plane)
	Eigen::Vector3f middle = middle_point.getVector3fMap();
	//Eigen::Vector3f diff1 = 100 * (v0.first->getVector3fMap() - middle);
	//Eigen::Vector3f diff2 = 100 * (ball_center.getVector3fMap() - middle);

	//Eigen::Vector3f y = diff1.cross(diff2).normalized();
	//Eigen::Vector3f normal = diff2.cross(y).normalized();

	Eigen::Vector3f normal = 100 * (v0.first->getArray3fMap() - v1.first->getArray3fMap());
	normal = normal.normalized();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(normal, middle);

	Eigen::Vector3f zero_angle = (ball_center.getVector3fMap() - middle).normalized();


	double current_angle = 2 * M_PI;
	std::pair<TrianglePtr, int> output = std::make_pair( TrianglePtr(), -1 );


	vector<int> indices = get_neighbors(middle_point, ball_radius * 2);
	for (int i = 0; i < indices.size(); i++) {
		int index = indices[i];
		if (v0.second == index || v1.second == index || op.second == index) {
			continue;
		}

		Eigen::Vector3f point = cloud->at(index).getVector3fMap();
		if (plane.absDistance(point) <= ball_radius)
		{
			Eigen::Vector3f center;
			Eigen::Vector3i sequence;
			if (get_ball_center(v0.second, index, v1.second, center, sequence)){
				pcl::PointNormal new_ball_center = Util::make_pt_normal(center);
				std::vector<int> neighborhood = get_neighbors(new_ball_center, ball_radius);
				if (!is_empty(neighborhood, v0.second, index, v1.second, center)){
					continue;
				}

				// Check the face is pointing upwards
				Eigen::Vector3f vij = v1.first->getVector3fMap() - v0.first->getVector3fMap();
				Eigen::Vector3f vik = point - v0.first->getVector3fMap();
				Eigen::Vector3f face_normal = vik.cross(vij).normalized();
				if (!Util::is_oriented(face_normal, (Eigen::Vector3f) v0.first->getNormalVector3fMap(), (Eigen::Vector3f) v1.first->getNormalVector3fMap(), (Eigen::Vector3f) cloud->at(index).getNormalVector3fMap())){
					continue;
				}

				//calc angle
				Eigen::Vector3f new_vec = (center - middle).normalized();
				double cos_angle = zero_angle.dot(new_vec);
				if (fabs(cos_angle) > 1)
					cos_angle = sign<double>(cos_angle);
				double angle = acos(cos_angle);

				// find a min angle by pivoting the ball
				if (output.second == -1 || current_angle > angle){
					current_angle = angle;
					output = std::make_pair(TrianglePtr(new Triangle(v0, PointData(&cloud->points[index], index), v1)), index);
				}
			}
		}
	}

	return output.first;
}

void BPA::output_triangle(const PointData &a, const PointData &b, const PointData &c) {
	Vertices tmp1;
	tmp1.vertices.push_back(a.second);
	tmp1.vertices.push_back(b.second);
	tmp1.vertices.push_back(c.second);
	mesh->polygons.push_back(tmp1);
}

pair<Triangle, bool> BPA::find_seed_triangle() {
	const double neighborhood_size = 1.5;

	bool found = false;

	for (auto it = unused_index.begin(); it != unused_index.end() && !found; it++)
	{
		int index0 = *it;

		// Get the point's neighborhood
		std::vector<int> indices = get_neighbors(cloud->at(index0), ball_radius * neighborhood_size);
		if (indices.size() < 3)
			continue;

		// Look for a valid seed
		for (int j = 0; j < indices.size(); j++)
		{
			if (!found)
			{
				int index1 = indices[j];
				if (index1 == index0 || pt_used[index1] == true)
					continue;

				for (int k = j + 1; k < indices.size() && !found; k++)
				{
					int index2 = indices[k];

					if (index1 == index2 || index2 == index0 || pt_used[index2] == true)
						continue;

					Eigen::Vector3f center;
					Eigen::Vector3i sequence;
					if (!found && get_ball_center(index0, index1, index2, center, sequence))
					{
						pcl::PointNormal ball_center = Util::make_pt_normal(center);
						std::vector<int> neighborhood = get_neighbors(ball_center, ball_radius);
						if (!found && is_empty(neighborhood, index0, index1, index2, center))
						{
							{
								if (!found)
								{
									std::cout << "\tSeed found (" << sequence[0] << ", " << sequence[1] << ", " << sequence[2] << ")\n";
									Triangle tmp = Triangle(make_pair(&cloud->at((int)sequence[0]), sequence[0]),
										make_pair(&cloud->at((int)sequence[1]), sequence[1]),
										make_pair(&cloud->at((int)sequence[2]), sequence[2]),
										ball_center, ball_radius);

									set_used(index0);
									set_used(index1);
									set_used(index2);

									found = true;

									return make_pair(tmp, found);
								}
							}
						}
					}
				}
			}
		}
	}

	return make_pair(Triangle(), found);
}

bool BPA::get_ball_center(const int _index0, const int _index1, const int _index2, Eigen::Vector3f &_center, Eigen::Vector3i &_sequence) const
{
	bool status = false;

	Eigen::Vector3f p0 = cloud->at(_index0).getVector3fMap();
	Eigen::Vector3f p1 = cloud->at(_index1).getVector3fMap();
	Eigen::Vector3f p2 = cloud->at(_index2).getVector3fMap();
	_sequence = Eigen::Vector3i(_index0, _index1, _index2);

	Eigen::Vector3f v10 = p1 - p0;
	Eigen::Vector3f v20 = p2 - p0;
	Eigen::Vector3f normal = v10.cross(v20);

	// Calculate ball center only if points are not collinear
	if (normal.norm() > COMPARISON_EPSILON)
	{
		// Normalize to avoid precision errors while checking the orientation
		normal.normalize();
		if (!Util::is_oriented(normal, (Eigen::Vector3f) cloud->at(_index0).getNormalVector3fMap(), (Eigen::Vector3f) cloud->at(_index1).getNormalVector3fMap(), (Eigen::Vector3f) cloud->at(_index2).getNormalVector3fMap()))
		{
			// Wrong orientation, swap vertices to get a CCW oriented triangle so face's normal pointing upwards
			p0 = cloud->at(_index1).getVector3fMap();
			p1 = cloud->at(_index0).getVector3fMap();
			_sequence = Eigen::Vector3i(_index1, _index0, _index2);

			v10 = p1 - p0;
			v20 = p2 - p0;
			normal = v10.cross(v20);
			normal.normalize();
		}

		std::pair<Eigen::Vector3f, double> circle = get_circumscribed_circle(p0, p1, p2);
		double squared_up = ball_radius * ball_radius - circle.second * circle.second;
		if (squared_up > 0)
		{
			double up = sqrt(fabs(squared_up));
			_center = circle.first + up * normal;
			status = true;
		}
	}
	return status;
}

bool BPA::is_empty(const std::vector<int> &_data, const int _index0, const int _index1, const int _index2, const Eigen::Vector3f &_ballCenter) const
{
	if (_data.empty())
		return true;

	for (int i = 0; i < _data.size(); i++)
	{
		if (_data[i] == _index0 || _data[i] == _index1 || _data[i] == _index2)
			continue;

		Eigen::Vector3f dist = cloud->at(_data[i]).getVector3fMap() - _ballCenter;
		if ( ( ball_radius - dist.norm() ) < IN_BALL_THRESHOLD)
			continue;
		else 
			return false;
	}

	return true;
}

std::pair<Eigen::Vector3f, double> BPA::get_circumscribed_circle(const Eigen::Vector3f &_p0, const Eigen::Vector3f &_p1, const Eigen::Vector3f &_p2) const
{
	Eigen::Vector3f d10 = _p1 - _p0;
	Eigen::Vector3f d20 = _p2 - _p0;
	Eigen::Vector3f d01 = _p0 - _p1;
	Eigen::Vector3f d12 = _p1 - _p2;
	Eigen::Vector3f d21 = _p2 - _p1;
	Eigen::Vector3f d02 = _p0 - _p2;

	double norm01 = d01.norm();
	double norm12 = d12.norm();
	double norm02 = d02.norm();

	double norm01C12 = d01.cross(d12).norm();

	double alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	double beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	double gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

	Eigen::Vector3f circumscribedCircleCenter = alpha * _p0 + beta * _p1 + gamma * _p2;
	double circumscribedCircleRadius = (norm01 * norm12 * norm02) / (2 * norm01C12);

	return std::make_pair(circumscribedCircleCenter, circumscribedCircleRadius);
}

vector<int> BPA::get_neighbors(const PointNormal &middle_point, double search_R) {
	std::vector<int> indices;
	std::vector<float> distances;
	kdtree.radiusSearch(middle_point, search_R, indices, distances);

	return indices;
}