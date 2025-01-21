#include "PointClouds.hh"
#include <types/Yolo.hh>
#include <types/Dataset.hh>

#include <functions/progress_bar.hh>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ml.hpp>

#include <string>


// #include <highfive/H5Easy.hpp>
// #include <highfive/highfive.hpp>
#include <pcl/filters/conditional_removal.h>

#include <fmt/core.h>
#include <fmt/color.h>

#include <iostream> 
#include <opencv2/opencv.hpp> 

// #include <xtensor/xtensor.hpp>
#include <xtensor/xarray.hpp>

#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>
#include <xtensor-io/xhighfive.hpp>



#include <optional>



static void draw_box(cv::Mat & img,  ReUseX::OutputParams const& param){
    cv::Scalar color_scalar = ReUseX::Color::from_index(param.id);
    std::string label = ReUseX::Yolov8Seg::GetClassName(param.id);
    cv::rectangle(img, param.box, color_scalar, 2);
    cv::putText(img, label, cv::Point(param.box.x, param.box.y), cv::FONT_HERSHEY_SIMPLEX, 1, color_scalar, 2);

    cv::Mat mask = param.boxMask;
    mask = ( mask > 0.5);
    mask.convertTo(mask, CV_8U);
    std::vector<cv::Mat> contours;
    cv::Mat hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat overlay = img.clone();
    cv::drawContours(overlay(param.box), contours, -1, color_scalar, cv::FILLED, cv::LINE_8, hierarchy,100);
    cv::addWeighted(overlay, 0.7, img, 0.3, 0, img);
}

static std::vector<std::string> split(const std::string& target, char c)
{
	std::string temp;
	std::stringstream stringstream { target };
	std::vector<std::string> result;

	while (std::getline(stringstream, temp, c)) {
		result.push_back(temp);
	}

	return result;
}

HighFive::DataType BoolMaskType = HighFive::AtomicType<uint8_t>();


static constexpr typename ReUseX::PointCloud::Cloud::PointType::LableT Invalid = std::numeric_limits<ReUseX::PointCloud::Cloud::PointType::LableT>::max();
static constexpr typename ReUseX::PointCloud::Cloud::PointType::LableT Deselected = std::numeric_limits<ReUseX::PointCloud::Cloud::PointType::LableT>::max()-1;


namespace ReUseX
{
    template<class T>
    PointClouds<T> PointClouds<T>::annotate(std::string yolo_path, std::optional<Dataset> & dataset){ 


        auto model = Yolov8Seg(yolo_path, true); // <- true for GPU
        std::chrono::nanoseconds duration = std::chrono::nanoseconds(0);
        size_t n_frames = data.size();



        // Checks
        assert(n_frames > 0);
        if constexpr (std::is_same<T, std::string>::value){
            std::cout << "Not checking if point cloud is dense for On disk point clouds" << std::endl;
        } else if constexpr (std::is_same<T, PointCloud>::value){
            for (size_t i = 0; i < n_frames; i++){
                if (!data.at(i)->is_dense)
                    throw std::runtime_error("Clouds must be dense");
            }
        }


        // Load images
        std::vector<cv::Mat> input_images;
        input_images.resize(n_frames);
        auto load_images_bar = util::progress_bar(n_frames,"Loading Images");
        if (dataset.has_value()){
            #pragma omp parallel for shared(data, input_images)
            for (size_t i = 0; i < n_frames; i++){
                pcl::PCLHeader header;
                if constexpr (std::is_same<T, std::string>::value)
                    header = PointCloud::load_header(data.at(i));
                else if constexpr (std::is_same<T, PointCloud>::value){
                    header = data.at(i)->header;
                }
                
                size_t index = std::atoi(header.frame_id.c_str());
                input_images[i] = dataset.value()[index].get<Field::COLOR>();
                load_images_bar.update();
            }
        } else {
            #pragma omp parallel for shared(data, input_images)
            for (size_t i = 0; i < n_frames; i++){
                PointCloud cloud;
                if constexpr (std::is_same<T, std::string>::value)
                    cloud = PointCloud::load(data.at(i));
                else if constexpr (std::is_same<T, PointCloud>::value){
                    cloud = data.at(i);
                } 
                input_images[i] = cloud.image();
                load_images_bar.update();
            }
        }
        duration += load_images_bar.stop();




        // Preprocessing
        std::vector<cv::Mat> blobs;
        blobs.resize(n_frames);
        std::vector<cv::Vec4d> params;
        params.resize(n_frames);
        auto preprocessing_bar = util::progress_bar(n_frames,"Preprocessing");
        #pragma omp parallel for shared(input_images, blobs, params)
        for (size_t i = 0; i < n_frames; i++){
            cv::rotate(input_images[i], input_images[i], cv::ROTATE_90_CLOCKWISE);
            //cv::resize(input_images[i], input_images[i], cv::Size(640,640));
            Yolov8Seg::Preprocess(input_images[i], blobs[i], params[i]);
            preprocessing_bar.update();
        }
        duration += preprocessing_bar.stop();


        // Inference
        // The neural network is not thread safe
        // _At least I am not able to make it thread safe_
        std::vector<std::vector<cv::Mat>> outputs;
        outputs.resize(n_frames);
        auto inference_bar = util::progress_bar(n_frames,"Running Inference");
        for (size_t i = 0; i < n_frames; i++){
            outputs[i] = model.Detect(blobs[i]);
            inference_bar.update();
        }
        duration +=  inference_bar.stop();
        blobs.clear(); // <- Free memory

        //cv::destroyAllWindows();


        auto inference_postprocessing_bar = util::progress_bar(n_frames,"Postprocessing");
        #pragma omp parallel for shared(data, outputs, params, input_images)
        for (size_t i = 0; i < n_frames; i++){

            auto results = Yolov8Seg::Postprocess(outputs[i], params[i], input_images[i].size());

            

            PointCloud cloud;
            if constexpr (std::is_same<T, std::string>::value){
                cloud = PointCloud::load(data.at(i));
            } else if constexpr (std::is_same<T, PointCloud>::value){
                cloud = data.at(i);
            } 


            // Reset the semantic field
            #pragma omp parallel for
            for (size_t i =0; i < cloud->size(); i++){
                cloud->at(i).semantic = 0;
            }


            cv::Size cloud_size = cv::Size(cloud->width, cloud->height);
            //cv::Size input_image_size_right_side_up = input_images[i].size();// cv::Size(input_image_size.height, input_image_size.width);
            cv::Size input_image_size = cv::Size(input_images[i].size().width, input_images[i].size().height);


            //auto img = cloud->image();
            //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

            for (OutputParams const& result_params: results ){
                OutputParams result_param_r = result_params
                    .Scale(input_image_size, cv::Size(cloud_size.height, cloud_size.width))
                    .Rotate<cv::ROTATE_90_COUNTERCLOCKWISE>(cv::Size(cloud_size.height, cloud_size.width));

                if (!result_param_r.valid)
                    continue;
                //draw_box(img, result_param_r);

                // Get the start and end of the box
                const auto row_start = result_param_r.box.y;
                const auto row_end = result_param_r.box.y + result_param_r.box.height;
                const auto col_start = result_param_r.box.x;
                const auto col_end = result_param_r.box.x + result_param_r.box.width;
                // Annotate the point cloud
                #pragma omp parallel for collapse(2) shared(cloud, result_param_r) firstprivate(row_start, row_end, col_start, col_end)
                for (int y = row_start; y < row_end; y++){
                    for (int x = col_start; x < col_end; x++){
                        if (result_param_r.boxMask.at<uchar>(y - row_start, x - col_start) > 0.5)
                            cloud->at(x, y).semantic = result_param_r.id;
                    }
                }
            }


            //cv::Mat img = cloud->image("semantic");
            //cv::addWeighted(img, 0.7, cloud->image("semantic"), 0.3, 0, img);
            //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
            //cv::imshow("Annotated Cloud", img);
            //cv::waitKey(0);

            if constexpr (std::is_same<T, std::string>::value)
                cloud.save(data.at(i));

            inference_postprocessing_bar.update();
        }
        duration += inference_postprocessing_bar.stop();




        blobs.clear();
        params.clear();
        outputs.clear();
        input_images.clear(); 

        // cv::destroyAllWindows();

        return *this;

       
    }


    template<class T>
    PointClouds<T> PointClouds<T>::annotate_from_hdf5(std::string hdf5_path){


        HighFive::File file(hdf5_path, HighFive::File::ReadOnly);

        HighFive::Group root = file.getGroup("/");

        // std::vector<std::string> frames = root.listObjectNames();

        // // Sort the frames
        // std::sort(frames.begin(), frames.end(), [ ](const std::string &a, const std::string &b) {
        //     auto a_num = std::stoi(a);
        //     auto b_num = std::stoi(b);
        //     return a_num < b_num;
        // });

        // Should check that the input frames match the point clouds

        // std::cout << "Number ov frames: " << frames.size() << std::endl;
        // std::cout << "Number ov clouds: " << data.size() << std::endl;

        // if (frames.size() != data.size())
        //     throw std::runtime_error("Number of frames and point clouds must match");


        for (size_t i = 0; i < data.size(); i++) {


            PointCloud cloud;
            if constexpr (std::is_same<T, std::string>::value){
                cloud = PointCloud::load(data.at(i));
            } else if constexpr (std::is_same<T, PointCloud>::value){
                cloud = data.at(i);
            }

            auto frame_id = cloud->header.frame_id;
            HighFive::Group frame;
            try{
                frame = root.getGroup(frame_id);
            } catch (const HighFive::GroupException & e){
                fmt::print(fg(fmt::color::red), "Frame {} not found in the HDF5 file", frame_id);
                continue;
            }


            // Set values for filtering
            #pragma omp parallel for shared(cloud)
            for (size_t i = 0; i < cloud->size(); i++){
                if(cloud->at(i).label != 2)
                    cloud->at(i).label = Invalid;
                else
                    cloud->at(i).label = Deselected;
            }


            xt::xarray<uint8_t,xt::layout_type::dynamic>  masks;
            std::vector<int> lables;
            try {
                masks = xt::load<xt::xarray<uint8_t,xt::layout_type::dynamic>>(file, frame_id+"/masks");
                lables = xt::load<std::vector<int>>(file, frame_id+"/names");
            }
            catch (const HighFive::DataSetException & e){
                fmt::print(fg(fmt::color::red), "Masks or names not found in frame {}", frame_id);
                continue;
            }
           

            #pragma omp parallel for shared(cloud, masks, lables)
            for (size_t i = 0; i < lables.size(); i++){

                auto img_size = cv::Size(masks.shape()[2],masks.shape()[1]);
                size_t offset = i * masks.shape()[1] * masks.shape()[2];

                cv::Mat masks_cv (img_size, CV_8U, masks.data()+offset, img_size.width);

                cv::rotate(masks_cv, masks_cv, cv::ROTATE_90_COUNTERCLOCKWISE);
                cv::resize(masks_cv, masks_cv, cv::Size(cloud->width,cloud->height));

                int label = lables[i];
                #pragma omp parallel for shared(cloud, masks_cv) firstprivate(label)
                for (size_t j = 0; j < masks_cv.total(); j++)
                    if (masks_cv.data[j] > 0.5 && cloud->at(j).label == Deselected )
                        cloud->at(j).label = label;
            }


            using PointT = PointCloud::Cloud::PointType;
            pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
            range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("label", pcl::ComparisonOps::GE, 0)));
            range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("label", pcl::ComparisonOps::LE, Deselected)));


            pcl::ConditionalRemoval<PointT> condrem;
            condrem.setCondition (range_cond);
            condrem.setInputCloud (cloud);
            condrem.setKeepOrganized (true);
            condrem.filter (*cloud);


            if constexpr (std::is_same<T, std::string>::value)
                cloud.save(data.at(i));
            
        }

        return *this;
    }

    template PointCloudsInMemory  PointCloudsInMemory::annotate(std::string ,std::optional<Dataset> & );
    template PointCloudsOnDisk  PointCloudsOnDisk::annotate(std::string ,std::optional<Dataset> & );

    template PointCloudsOnDisk  PointCloudsOnDisk::annotate_from_hdf5(std::string );
    template PointCloudsInMemory  PointCloudsInMemory::annotate_from_hdf5(std::string );

} // namespace ReUseX
