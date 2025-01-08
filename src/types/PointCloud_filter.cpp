#include <types/PointCloud.hh>
#include <types/PointClouds.hh>

#include <vector>
#include <types/PointCloud.hh>
#include <functions/progress_bar.hh>


namespace ReUseX
{
    void PointCloud::filter(typename PointCloud::Cloud::PointType::LableT value){

        // Only keep highest confidence
        ///////////////////////////////////////////////////////////////////////////////
        size_t j = 0;
        for (size_t k = 0; k < (*this)->size(); k++){
            if ((*this)->at(k).confidence >= value){
                (*this)->at(j) = (*this)->at(k);
                j++;
            }
        }
        (*this)->resize(j);
        (*this)->width = j;
        (*this)->height = 1;
        (*this)->is_dense = false;


    }
} // namespace ReUseX
