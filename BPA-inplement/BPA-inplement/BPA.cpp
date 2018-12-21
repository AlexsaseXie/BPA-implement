#include "BPA.h"

void BPA::do_bpa(pcl::PointCloud<pcl::PointNormal> &cloud, pcl::PolygonMesh &mesh) {
	init(cloud);

	while (true) {
		EdgePtr eij;

		while (eij = get_active_edge()) {
			PointData ek_data = ball_pivot();
			int ek_index = ek_data.second;
			PointNormal &ek = ek_data.first;

			if (ek_index >= 0 && (pt_used[ek_index] == 0 || F.on_front(ek_data))) {
				output_triangle(eij->vertices[0], eij->vertices[1], ek_data);

				F.join(*eij, ek_data);
				
				
				if (F.has(ek_data, eij->vertices[0])) {
					F.glue(eij->vertices[0], ek_data);
				}

				if (F.has(eij->vertices[1], ek_data)) {
					F.glue(ek_data, eij->vertices[1]);
				}
			}
			else {
				mark_as_boundary(eij);
			}
		}

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

PointData BPA::ball_pivot() {
	return PointData();
}

void BPA::output_triangle(const PointData &a, const PointData &b, const PointData &c) {
	
}

pair<Triangle, bool> BPA::find_seed_triangle() {
	return make_pair(Triangle(),true);
}
