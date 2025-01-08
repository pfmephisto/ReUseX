#include <types/PointCloud.hh>
#include <types/PointClouds.hh>

#include <vector>
#include <types/PointCloud.hh>
#include <functions/progress_bar.hh>


namespace ReUseX
{
    template <typename T>
    PointClouds<T> PointClouds<T>::filter(
        typename PointCloud::Cloud::PointType::LableT value
    ){

        // Only keep highest confidence
        ///////////////////////////////////////////////////////////////////////////////
        auto filter_bar = util::progress_bar(data.size(), "Filtering");
        #pragma omp parallel for
        for (size_t i = 0; i < data.size(); i++){
            if constexpr (std::is_same<T, std::string>::value){
                auto cloud = PointCloud::load(data[i]);
                cloud.filter(value);
                cloud.save(data[i]);  
            } else if constexpr (std::is_same<T, PointCloud>::value){
                data[i].filter(value);
            }
            filter_bar.update();
        }
        filter_bar.stop();

        return *this;

    }

    template PointCloudsInMemory  PointCloudsInMemory::filter( PointCloud::Cloud::PointType::LableT value);
    template PointCloudsOnDisk  PointCloudsOnDisk::filter( PointCloud::Cloud::PointType::LableT value);

} // namespace ReUseX
