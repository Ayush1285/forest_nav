#include "genLocalMap.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

Eigen::MatrixXi cleanMapData(const std::vector<int8_t>& map_data, const uint32_t map_height, const uint32_t map_width){
  auto start0 = std::chrono::steady_clock::now();
  Eigen::MatrixXi clean_map_data(map_height,map_width);
  cv::Mat map_data_img;
  map_data_img = cv::Mat::zeros(map_width, map_height, CV_32F);

  for(size_t y = 0; y < map_height; y++) {
    for(size_t x = 0; x < map_width; x++) {
      map_data_img.at<float>(x,y) = map_data[y * map_width + x] <= 0 ? 0.0 : 1.0;
    }
  }
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
  dilate(map_data_img, map_data_img, element, cv::Point(-1, -1), 4); 
  //element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
  //erode(map_data_img, map_data_img, element, cv::Point(-1, -1), 2); 

  for(size_t y = 0; y < map_height; y++) {
    for(size_t x = 0; x < map_width; x++) {
      clean_map_data(y,x) = 100 * map_data_img.at<float>(x,y);
    }
  }
  auto end0 = std::chrono::steady_clock::now();
    std::cout << "Elapsed time in cleaning map: " << std::chrono::duration_cast<std::chrono::milliseconds>(end0 - start0).count()
        << " ms" << std::endl;
  return clean_map_data;
}

Eigen::MatrixXi cleanMapData(const Eigen::MatrixXi& map_data, const uint32_t map_height, const uint32_t map_width){
  //auto start0 = std::chrono::steady_clock::now();
  Eigen::MatrixXi clean_map_data(map_height,map_width);
  cv::Mat map_data_img;
  map_data_img = cv::Mat::zeros(map_width, map_height, CV_32F);

  for(size_t y = 0; y < map_height; y++) {
    for(size_t x = 0; x < map_width; x++) {
      map_data_img.at<float>(x,y) = map_data(y,x);
    }
  }
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  dilate(map_data_img, map_data_img, element, cv::Point(-1, -1), 4); 
  //element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
  //erode(map_data_img, map_data_img, element, cv::Point(-1, -1), 2); 

  for(size_t y = 0; y < map_height; y++) {
    for(size_t x = 0; x < map_width; x++) {
      clean_map_data(y,x) = 100 * map_data_img.at<float>(x,y);
    }
  }
  //auto end0 = std::chrono::steady_clock::now();
    //std::cout << "Elapsed time in cleaning map: " << std::chrono::duration_cast<std::chrono::milliseconds>(end0 - start0).count()
      //  << " ms" << std::endl;
  return clean_map_data;
}

GridPoint refineLocalGoal(const Eigen::MatrixXi& globalMap, const GridPoint localgoal_, const GridPoint globalgoal_){
  GridPoint localgoal_init = localgoal_;
  if(globalMap(localgoal_.y, localgoal_.x)==100){
    if(localgoal_.x==globalgoal_.x){
      while(globalMap(localgoal_init.y, localgoal_init.x)==100){
        localgoal_init.y = localgoal_init.y + (globalgoal_.y>localgoal_.x?1:-1)*4;
      }
      return localgoal_init;
    }
    else{
      double slopeangle = std::atan2(globalgoal_.y-localgoal_init.y,globalgoal_.x-localgoal_init.x);
      while(globalMap(localgoal_init.y, localgoal_init.x)==100){
        localgoal_init.y = localgoal_init.y + std::sin(slopeangle)*4;
        localgoal_init.x = localgoal_init.x + std::cos(slopeangle)*4;
      }
      return localgoal_init;
    }
  }
  else{
  return localgoal_;
  }
}

LocalMap genLocalMap(const Quad& quad, const GlobalMap& globalmap_){
  LocalMap localmap_;
  GridPoint currglobalpos_grid(std::floor((quad.position(1)-globalmap_.origin.position.y)/globalmap_.resolution),
                                                std::floor((quad.position(0)-globalmap_.origin.position.x)/globalmap_.resolution));
  //std::cout << "current position in global grid = " << currglobalpos_grid << std::endl;
  GridPoint globalgoal_grid(std::floor((globalmap_.goal(1)-globalmap_.origin.position.y)/globalmap_.resolution),
                                                std::floor((globalmap_.goal(0)-globalmap_.origin.position.x)/globalmap_.resolution));
  //std::cout << "desired goal in global grid = " << globalgoal_grid << std::endl;
  double slopeangle;
  GridPoint localgoal_grid;
  if(std::sqrt(std::pow((quad.position(0) - globalmap_.goal(0)),2) +
                                      std::pow((quad.position(1) - globalmap_.goal(1)),2)) > max_localgoal_dist){
      if(currglobalpos_grid.x==globalgoal_grid.x){
        localgoal_grid.y = currglobalpos_grid.y + (globalgoal_grid.y>currglobalpos_grid.y?1:-1)*
                        std::floor(3*localWindowSize/(4*globalmap_.resolution));
        localgoal_grid.x = currglobalpos_grid.x;
      }
      else{
        slopeangle = std::atan2(globalgoal_grid.y-currglobalpos_grid.y,globalgoal_grid.x-currglobalpos_grid.x);
        localgoal_grid.y = currglobalpos_grid.y + std::floor((3*localWindowSize*std::sin(slopeangle))/(4*globalmap_.resolution));
        localgoal_grid.x = currglobalpos_grid.x + std::floor((3*localWindowSize*std::cos(slopeangle))/(4*globalmap_.resolution));
      }
      localgoal_grid = refineLocalGoal(globalmap_.grid, localgoal_grid, globalgoal_grid);
  }
  else{
      localgoal_grid = globalgoal_grid;
      if(globalmap_.grid(localgoal_grid.y,localgoal_grid.x)==100){
        //while(globalmap_.grid(localgoal_grid(0), localgoal_grid(1))==100){
          if(currglobalpos_grid.x==globalgoal_grid.x){
              while(globalmap_.grid(localgoal_grid.y, localgoal_grid.x)==100){
                localgoal_grid.y = localgoal_grid.y + 5;
              }
          }
          else{
              slopeangle = std::atan2(globalgoal_grid.y-currglobalpos_grid.y,globalgoal_grid.x-currglobalpos_grid.x);
              while(globalmap_.grid(localgoal_grid.y, localgoal_grid.x)==100){
                localgoal_grid.y = localgoal_grid.y + std::sin(slopeangle)*4;
                localgoal_grid.x = localgoal_grid.x + std::cos(slopeangle)*4;
              }
            }
      }
  }
  
  const int diameter = std::floor(std::sqrt(std::pow((localgoal_grid.y - currglobalpos_grid.y),2) +
                                      std::pow((localgoal_grid.x - currglobalpos_grid.x),2)));

  localmap_.origin = {((localgoal_grid.y + currglobalpos_grid.y)/2) - (2*diameter/3),
                      ((localgoal_grid.x + currglobalpos_grid.x)/2) - (2*diameter/3)};

  localmap_.start.y = currglobalpos_grid.y - localmap_.origin.y;
  localmap_.start.x = currglobalpos_grid.x - localmap_.origin.x;
  localmap_.goal.y = localgoal_grid.y - localmap_.origin.y;
  localmap_.goal.x = localgoal_grid.x - localmap_.origin.x;
  localmap_.resolution = globalmap_.resolution;
  int windowsize = 4*diameter/3;
  if(windowsize < (std::floor(localWindowSize/globalmap_.resolution))){
    windowsize = std::floor(localWindowSize/globalmap_.resolution);
  }
  localmap_.grid = globalmap_.grid.block(localmap_.origin.y, localmap_.origin.x, windowsize, windowsize);
  localmap_.width = localmap_.grid.cols();
  localmap_.height = localmap_.grid.rows();
  localmap_.global_origin = globalmap_.origin;
  //std::cout << " localmap start = " << localmap_.start << std::endl;
  //std::cout << " localmap goal = " << localmap_.goal << std::endl;
  return localmap_ ;
}