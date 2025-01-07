#include <algorithms/refine.hh>
#include <types/PointCloud.hh>

#include <functions/fit_plane_thorugh_points.hh>
#include <functions/progress_bar.hh>

#include <functions/crop_plane_with_aabb.hh>
#include <cmath>


namespace linkml{


    std::vector<pcl::PointIndices> refine(
            PointCloud::Cloud::Ptr const cloud, 
            std::vector<pcl::PointIndices> const & clusters,
            float angle_threashhold,
            float distance_threshhold
            ){

        std::vector<Plane> planes;
        planes.resize(clusters.size());

        #pragma omp parallel for
        for (size_t i = 0; i < clusters.size(); ++i)
            planes[i] = fit_plane_thorugh_points(cloud, clusters[i].indices);

        
        std::vector<pcl::PointIndices> indecies;
        std::copy(clusters.begin(), clusters.end(), std::back_inserter(indecies));



        auto pbar = util::progress_bar(planes.size(), "Plane Refinement");
        for (int i = 0; i < (int)planes.size(); i++){

            std::vector<int> sel;

            // Check angle
            for (auto j = i+1; j < (int)planes.size(); j++){

                auto dot = CGAL::scalar_product(planes[i].orthogonal_vector(), planes[j].orthogonal_vector());
                if (dot < std::cos(angle_threashhold)){
                    sel.push_back(j);
                }
            }


            auto to_be_deleted = std::vector<int>();
            // Check overlap and merge
            for (auto & j: sel){

                auto A = planes[i];
                auto B = planes[j];


                auto A_idx =indecies[i];
                auto B_idx =indecies[j];

                auto n_point_of_B_in_A = std::accumulate(B_idx.indices.begin(), B_idx.indices.end(), 0, [&](int sum, int idx){
                    auto distance = std::sqrt(CGAL::squared_distance(A, cloud->points.at(idx).getPos()));

                    if (distance < distance_threshhold)
                        sum++;
                    return sum;
                });

                auto n_point_of_A_in_B = std::accumulate(A_idx.indices.begin(), A_idx.indices.end(), 0, [&](int sum, int idx){
                    auto distance = std::sqrt(CGAL::squared_distance(B, cloud->points.at(idx).getPos()));

                    if (distance < distance_threshhold)
                        sum++;
                    return sum;
                });

                auto Nt = std::min( A_idx.indices.size(),B_idx.indices.size())/5;

                // auto Nt = (long)tg::min( A_idx.indices.size(),B_idx.indices.size())/5;

                if (n_point_of_A_in_B > Nt and n_point_of_B_in_A > Nt){
                    to_be_deleted.push_back(j);
                    // Or mark them as empty
                    pcl::PointIndices merged;
                    std::copy(A_idx.indices.begin(), A_idx.indices.end(), std::back_inserter(merged.indices));
                    std::copy(B_idx.indices.begin(), B_idx.indices.end(), std::back_inserter(merged.indices));
                    planes[i] = fit_plane_thorugh_points(cloud, merged.indices);
                    indecies[i] = merged;
                }

            }

            // Erase merged itmes
            for (auto it = to_be_deleted.rbegin(); it != to_be_deleted.rend(); ++it){
                size_t idx = *it;
                auto p_it = std::next(planes.begin(), idx );
                auto i_it = std::next(indecies.begin(), idx ); 
                planes.erase(p_it);
                indecies.erase(i_it);
            }

            pbar.update(1+to_be_deleted.size());

        }
        pbar.stop();

        return indecies;
            

    }
}

