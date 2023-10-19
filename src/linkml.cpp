#include "linkml.h"
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <optional>
#include "../../../External Repos/nanoflann/include/nanoflann.hpp"
#include <typed-geometry/tg.hh>
//#include <nanoflan/nanoflan.hpp>


int random_integert_in_range(int min, int max)
{
    std::srand(std::time(NULL));
    return min + (rand() % (max - min + 1));
}

int get_random_index_not_in_register(const linkml::reg &reg)
{
    std::vector<int> options = std::vector<int>();

    for (size_t i = 0; i < reg.mask.size() ; i++)
        if (reg.mask.at(i) == 0)
            options.push_back(i);

    return options[random_integert_in_range(0,options.size()-1)];
}

void update_register(linkml::reg &reg){

    std::vector<int> compacted = std::vector<int>();

    for (size_t i = 0; i< reg.indecies.size(); i++)
        if ( reg.mask.at(i) == 1  )
            compacted.push_back(i);

    reg.indecies = compacted;
}



linkml::Plane fit_plane_thorugh_points(const linkml::point_cloud &cloud, const std::vector<int> &indecies){

    std::vector<tg::pos3> points = std::vector<tg::pos3>();
    std::vector<tg::pos3> centered = std::vector<tg::pos3>();

    for (size_t i = 0; i < indecies.size(); i++ )
        points.push_back(cloud.pts.at(indecies[i]));

    tg::pos3 com = tg::mean(points);
    for (size_t i = 0; i < points.size(); i++)
        centered.push_back(points.at(i) - (tg::vec3)com);

//    tg::covariance_matrix(centered);
//    tg::eigen_decomposition_symmetric();
//    tg::eigenvalues_symmetric();
//    tg::eigenvectors_symmetric();



    return linkml::Plane();



    /*

    com = np.mean(points, axis=0)  # Maybe choose a different center
    centered = points-com
    cov = np.cov(centered.T)

     eig = np.linalg.eigh(cov)
     n = eig.eigenvectors[:, eig.eigenvalues.argmin()]

     self[:3] = n
          self[3] = np.dot(-com, n)

     self._origin = None

     return self

         */

}



std::vector<int> filter_by_normal_angle(const linkml::point_cloud &cloud, const linkml::Plane plane, float cosalpha, std::vector<int> indecies_in ){

    std::vector<int> indecies_out = std::vector<int>();

    for( size_t i = 0; i< indecies_in.size(); i++)
    {
        int index = indecies_in.at(i);

        // Normalize Point Normal
        tg::vec3 unitised = tg::normalize(cloud.norm.at(index));

        // Dot product  (Cos angele)
        float cos = std::abs(
            unitised.x*plane.normal.x+
            unitised.y*plane.normal.y+
            unitised.z*plane.normal.z
            );

        if (cos > cosalpha)
            indecies_out.push_back(index);

    }

    return indecies_out;
}

std::vector<int> filter_by_normal_distance(const linkml::point_cloud &cloud, const std::vector<int> indecies_in, const linkml::Plane plane, float distance_thresh){

    std::vector<int> indecies_out = std::vector<int>();

    for (size_t i = 0; i < indecies_in.size(); i++)
    {
        float dist = tg::distance(plane, cloud.pts.at(indecies_in.at(i)));
//        float dist = point_distance_to_plane(cloud.pts.at(indecies_in.at(i)), plane);

        if (dist < distance_thresh){
            indecies_out.push_back(indecies_in.at(i));
        }
    }

    return indecies_out;
}

linkml::plane_fit_resutl fit_plane(
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float /* Distance */,linkml::point_cloud /* Data */> /* Distance */, linkml::point_cloud /*Data*/,3 /* Dim */, int /*Index*/> &tree,
    const linkml::point_cloud &cloud,
    const linkml::reg &processed_reg,
    const linkml::plane_fitting_parameters params,
    int initial_point_idx = -1
    ){

    // Get starting point index
    if (initial_point_idx == -1)
        initial_point_idx =  get_random_index_not_in_register(processed_reg);


    reg front_reg = reg(processed_reg.mask.size());
    reg plane_reg = reg(processed_reg.mask.size());

    // Set initila point in front
    front_reg.mask[initial_point_idx] = 1;
    front_reg.indecies[0] = initial_point_idx;


    // Set initila point in plane
    plane_reg.mask[initial_point_idx] = 1;
    plane_reg.indecies[0] = initial_point_idx;

    // Set plane
    linkml::Plane plane = linkml::Plane(
        cloud.norm.at(initial_point_idx).x,
        cloud.norm.at(initial_point_idx).y,
        cloud.norm.at(initial_point_idx).z,
        0,
        cloud.pts.at(initial_point_idx).x,
        cloud.pts.at(initial_point_idx).y,
        cloud.pts.at(initial_point_idx).z
    );

    // Keep growing the plane
    int front_loop = 0;
    while (true){

        // Stop if the are no more points in the front
        if (front_reg.indecies.size() == 0)
            break;

        // Clear the front register mask
        front_reg.mask.assign(front_reg.mask.size(), 0);

        // Loop all items in the current front
        for (size_t i = 0; i < front_reg.indecies.size(); i++){

            //KDTree radius search
            std::vector<nanoflann::ResultItem<int, float>> ret_matches;

            tg::pos3 pt = cloud.pts.at(front_reg.indecies.at(i));
            float query_pt[3] ={pt.x,pt.y,pt.z};

            int count = tree.radiusSearch( &query_pt[0], params.distance_threshhold, ret_matches );

            if (count == 0 or ret_matches.size() == 0)
                break;

            //Extract indecies
            std::vector<int> indecies = std::vector<int>();
            for (size_t i = 0; i < ret_matches.size(); i++)
                indecies.push_back(ret_matches.at(i).first);


            // Filter Angle
            indecies = filter_by_normal_angle(cloud, plane, params.cosalpha, indecies);
            if (indecies.size() == 0)
                break;

            // Filter Distance
            indecies = filter_by_normal_distance(cloud, indecies, plane, params.normal_distance_threshhold);
            if (indecies.size() == 0)
                break;

            //Assign values
            for (size_t i= 0; i< indecies.size(); i++)
                front_reg.mask[indecies.at(i)] = 1;
        }

        //Remove points that are already part of the current plane
        for (int i = 0; 0 < plane_reg.indecies.size(); i++)
            front_reg.mask[plane_reg.indecies.at(i)] = 0;

        //Remove point that have already been proccessed
        for (int i = 0; 0 < processed_reg.indecies.size(); i++)
            front_reg.mask[processed_reg.indecies.at(i)] = 0;

        update_register(front_reg);

        // Update Plane with furrent front
        for (int i = 0; 0 < front_reg.indecies.size(); i++)
            plane_reg.mask[front_reg.indecies.at(i)] = 1;


        // Update plane
        if (front_loop % 5 == 0){
            update_register(plane_reg);
            if (plane_reg.indecies.size() > 3)
                plane = fit_plane_thorugh_points(cloud, plane_reg.indecies);
        }

        front_loop++;
    }


    update_register(plane_reg);

    // Check if plane meets requered size
    if ((int)plane_reg.indecies.size() < params.plane_size_threshhold)
        return linkml::plane_fit_resutl(initial_point_idx);

    plane = fit_plane_thorugh_points(cloud, plane_reg.indecies);

    //Clear plane register
    return linkml::plane_fit_resutl(plane, plane_reg.indecies);


};


void fit_planes()
{

    // Launch a pool of threads looking for planes and manange the results they return.
    // Implement different stoping contidions
    //      All points serached
    //      60% > searched etc.
}
