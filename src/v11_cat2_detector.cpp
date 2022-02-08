#include "v11_cat2_detector/v11_cat2_detector.h"

#define FROM_VIDEO 0

const float Cat2Detector::MIN_CONTOUR_AREA = 100.0f;
const float Cat2Detector::MIN_FIELD_CONTOUR_AREA = 1600.0f;

// typedef std::vector<cv::Point3i> Vectors3;

Cat2Detector::Cat2Detector()
    :nh_(ros::this_node::getName()),
      it_(this->nh_),
      it_subs_(it_.subscribe("image_in", 1, &Cat2Detector::imageCallback, this)),
      it_pubs_(it_.advertise("image_out_cat2", 100)),
      cam_info_sub_(nh_.subscribe("camera_info_in", 100, &Cat2Detector::cameraInfoCallback, this)),
      cam_info_pub_(nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 100)),
      frame_mode_subs_(nh_.subscribe("frame_mode", 1, &Cat2Detector::frameModeCallback, this)),
      save_param_subs_(nh_.subscribe("save_param", 1, &Cat2Detector::saveParamCallback, this)),
      LUT_sub_(nh_.subscribe("LUT_data", 1, &Cat2Detector::lutCallback, this)),
      it_bf_sub_(it_.subscribe("square_ref", 1, &Cat2Detector::squareRefCallback, this)),
      square_pos_pub_(nh_.advertise<geometry_msgs::Point > ("square_pos", 100)),
      line_pos_pub_(nh_.advertise<geometry_msgs::Point > ("line_pos", 100)),
//vertikal line baru
      line_pos_ver_pub_(nh_.advertise<geometry_msgs::Point > ("line_pos_ver", 100)),
      update_params_pub_(nh_.advertise<std_msgs::Empty > ("update_params", 10)),
      it_inv_sg_pub_(it_.advertise("inv_segment_green", 10)),
      field_boundary_pub_(nh_.advertise<vision_utils::FieldBoundary > ("field_boundary", 10)),
      frame_mode_(0){

    nh_.param<std::string>("square_config_path", square_config_path,
                           ros::package::getPath("v11_cat2_detector") + "/config/saved_config.yaml");

    param_cb_ = boost::bind(&Cat2Detector::paramCallback, this, _1, _2);
    param_server_.setCallback(param_cb_);

    LUT_dir = ros::package::getPath("v11_cat2_detector") + "/config/tabel_warna.xml";

    loadParam();
}

void Cat2Detector::loadParam(){
    YAML::Node config_file;
    try{
        config_file = YAML::LoadFile(square_config_path.c_str());
    }catch(const std::exception &e){
        ROS_ERROR("[v11_cat2_detector] Unable to open config file: %s", e.what());
    }

    config_.min_thres = config_file["min_thres"].as<int>();
    config_.ratio1 = config_file["ratio1"].as<int>();
    config_.ratio2 = config_file["ratio2"].as<int>();

    cv::FileStorage fs(LUT_dir.c_str(),cv::FileStorage::READ);
    fs["Tabel_Warna"] >> LUT_data;
    fs.release();

    square_ref_ = cv::imread(ros::package::getPath("v11_cat2_detector") + "/config/square_ref.jpg");
    square_ref_ = cvtMulti(square_ref_);
    if(!square_ref_.empty()){
        cv::calcHist(&square_ref_, 1, hist_param_.channels, cv::Mat(), square_ref_hist_, 2, hist_param_.hist_size, hist_param_.ranges);
        cv::normalize(square_ref_hist_,square_ref_hist_, .0, 1.0, cv::NORM_MINMAX);
    }
}

void Cat2Detector::saveParam(){
    YAML::Emitter yaml_out;
    yaml_out << YAML::BeginMap;
    yaml_out << YAML::Key << "min_thres" << YAML::Value << config_.min_thres;
    yaml_out << YAML::Key << "ratio1" << YAML::Value << config_.ratio1;
    yaml_out << YAML::Key << "ratio2" << YAML::Value << config_.ratio2;
    yaml_out << YAML::EndMap;
    std::ofstream file_out(square_config_path.c_str());
    file_out << yaml_out.c_str();
    file_out.close();
    cv::FileStorage fs(LUT_dir.c_str(),cv::FileStorage::WRITE);
    fs << "Tabel_Warna" << LUT_data;
    fs.release();

    cv::imwrite(ros::package::getPath("v11_cat2_detector") + "/config/square_ref.jpg", square_ref_);
}

void Cat2Detector::frameModeCallback(const std_msgs::Int8::ConstPtr &_msg){
    frame_mode_ = _msg->data;
}

void Cat2Detector::saveParamCallback(const std_msgs::Empty::ConstPtr &_msg){
    (void)_msg;
    saveParam();
}

void Cat2Detector::squareRefCallback(const sensor_msgs::ImageConstPtr &_msg){
    cv_bf_ptr_sub_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
    square_ref_ = cv_bf_ptr_sub_->image;
    square_ref_ = cvtMulti(square_ref_);
    cv::calcHist(&square_ref_, 1, hist_param_.channels, cv::Mat(), square_ref_hist_, 2, hist_param_.hist_size, hist_param_.ranges);
    cv::normalize(square_ref_hist_, square_ref_hist_, .0, 1.0 , cv::NORM_MINMAX);
}

void Cat2Detector::lutCallback(const vision_utils::LUTConstPtr &_msg){
    for(size_t i = 0; i < _msg->color.size(); i++){
        int h = (int)_msg->color[i].x;
        int s = (int)_msg->color[i].y;
        LUT_data.at<uchar>(h,s) = (int) _msg->color_class.data;
    }
}

void Cat2Detector::imageCallback(const sensor_msgs::ImageConstPtr &_msg){

    try{
        img_encoding_ = Alfarobi::GRAY8Bit;
        if(_msg->encoding.compare(sensor_msgs::image_encodings::MONO8))
            img_encoding_ = Alfarobi::GRAY8Bit;
#if FROM_VIDEO == 0
        if(_msg->encoding.compare(sensor_msgs::image_encodings::BGR8))
            img_encoding_ = Alfarobi::BGR8Bit;
#else
        if(_msg->encoding.compare(sensor_msgs::image_encodings::RGB8))
            img_encoding_ = Alfarobi::BGR8Bit;
#endif
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("[v11_cat2_detector] cv bridge exception: %s",e.what());
        return;
    }

    cv_img_ptr_subs_ = cv_bridge::toCvCopy(_msg,_msg->encoding);
    this->stamp_ = _msg->header.stamp;
    this->frame_id_ = _msg->header.frame_id;
}

void Cat2Detector::cameraInfoCallback(const sensor_msgs::CameraInfo &_msg){
    //cam_info_msg_ = *_msg;
    
//    ROS_INFO("CHECK...");
}

void Cat2Detector::paramCallback(v11_cat2_detector::Cat2DetectorParamsConfig &_config, uint32_t level){
    (void)level;
    this->config_ = _config;
}

void Cat2Detector::publishImage(){
    cv_img_pubs_.image = out_img_.clone();

    //Stamp
    cv_img_pubs_.header.seq++;
    cv_img_pubs_.header.stamp = this->stamp_;
    cv_img_pubs_.header.frame_id = this->frame_id_;

    //microsoft lifecam brightness setting only work when the camera is capturing
    //setting first to zero brightness after first 2 frame then set to desired value
    //3 April 2019
    if(cv_img_pubs_.header.seq == 2){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }else if(cv_img_pubs_.header.seq == 4){
        std_msgs::Empty empty_msg;
        update_params_pub_.publish(empty_msg);
    }

    switch(img_encoding_){
        case Alfarobi::GRAY8Bit:cv_img_pubs_.encoding = sensor_msgs::image_encodings::MONO8;break;
        case Alfarobi::BGR8Bit:cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;break;
        default:cv_img_pubs_.encoding = sensor_msgs::image_encodings::RGB8;break;
    }

    it_pubs_.publish(cv_img_pubs_.toImageMsg());
    cam_info_pub_.publish(cam_info_msg_);
}

cv::Mat& Cat2Detector::setInputImage(){
    return in_img_;
}

void Cat2Detector::setOutputImage(const cv::Mat &_out_img){
    out_img_ = _out_img.clone();
}

cv::Mat Cat2Detector::segmentColor(cv::Mat &_segmented_green, cv::Mat &_inv_segmented_green, cv::Mat &_segmented_blue, cv::Mat &_segmented_white){

    cv::Mat blank = cv::Mat::zeros(Alfarobi::FRAME_HEIGHT, Alfarobi::FRAME_WIDTH, CV_8UC1);
    cv::Mat out_segment = cv::Mat::zeros(Alfarobi::FRAME_HEIGHT, Alfarobi::FRAME_WIDTH, CV_8UC3);

    cv::Mat segmented_green = blank.clone();
    cv::Mat segmented_blue = blank.clone();
    cv::Mat segmented_white = blank.clone();

    cv::cvtColor(in_img_,in_hsv_,CV_BGR2HSV);

    int num_cols = Alfarobi::FRAME_WIDTH;
    int num_rows = Alfarobi::FRAME_HEIGHT;

    // auto LUT_ptr = LUT_data.data;
    for(int i = 0; i < num_rows; i++){
        cv::Vec3b* in_hsv_ptr = in_hsv_.ptr<cv::Vec3b>(i);
        cv::Vec3b* out_segment_ptr = out_segment.ptr<cv::Vec3b>(i);
        uchar* sg_ptr = segmented_green.ptr<uchar>(i);
        uchar* sb_ptr = segmented_blue.ptr<uchar>(i);
        uchar* sw_ptr = segmented_white.ptr<uchar>(i); 
        for(int j = 0; j < num_cols; j++){
            uchar pres_class = LUT_data.at<uchar>(in_hsv_ptr[j][0], in_hsv_ptr[j][1]);
            if(pres_class == 1){
                sg_ptr[j] = 255;
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 200;
                out_segment_ptr[j][2] = 0;                
            }else if(pres_class == 2){
                sb_ptr[j] = 255;
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 0;
                out_segment_ptr[j][2] = 200;
            }else if(pres_class == 3){
                sw_ptr[j] = 255;
                out_segment_ptr[j][0] = 255;
                out_segment_ptr[j][1] = 255;
                out_segment_ptr[j][2] = 255;
            }
        }
    }

    cv::Mat inv_segmented_green;
    cv::bitwise_not(segmented_green,inv_segmented_green);

    _segmented_green = segmented_green.clone();
    _inv_segmented_green = inv_segmented_green.clone();
    _segmented_blue = segmented_blue.clone();
    _segmented_white = segmented_white.clone();

    return out_segment;
}

cv::Mat Cat2Detector::cvtMulti(const cv::Mat &_square_ref){
    cv::Mat yuv;
    cv::cvtColor(_square_ref,yuv,CV_BGR2YUV);
    return yuv.clone();
}

void Cat2Detector::filterContourData(std::vector<cv::Mat> &divided_roi, cv::Point top_left_pt,
                       std::vector<Points > &selected_data, cv::Mat *debug_mat, int sub_mode = 0){
    int num_roi_cols = divided_roi[0].cols;
    int num_roi_rows = divided_roi[0].rows;
    bool horizon_scan = (float)num_roi_rows/(float)num_roi_cols < .75f;
    cv::Point map_origin[4];
    map_origin[0].x = top_left_pt.x;
    map_origin[0].y = top_left_pt.y;
    map_origin[1].x = (sub_mode == 2)?top_left_pt.x:top_left_pt.x + divided_roi[0].cols;
    map_origin[1].y = (sub_mode == 2)?top_left_pt.y + divided_roi[0].rows:top_left_pt.y;
    map_origin[2].x = top_left_pt.x;
    map_origin[2].y = top_left_pt.y + divided_roi[0].rows;
    map_origin[3].x = top_left_pt.x + divided_roi[0].cols;
    map_origin[3].y = top_left_pt.y + divided_roi[0].rows;
    for(size_t idx = 0;idx < divided_roi.size() ; idx++){

        int scan_mode=idx;

        switch(idx){
        case 0:scan_mode = (sub_mode == 1)?0:(sub_mode == 2)?2:horizon_scan?0:2;break;
        case 1:scan_mode = (sub_mode == 1)?1:(sub_mode == 2)?3:horizon_scan?1:2;break;
        case 2:scan_mode = horizon_scan?0:3;break;
        case 3:scan_mode = horizon_scan?1:3;break;
        }

        switch(scan_mode){
        case 0:{
            for(int i=0;i<num_roi_rows;i++){
                for(int j=0;j<num_roi_cols;j++){
                    if(divided_roi[idx].at<uchar>(i,j) == 255){
                        if(j==0)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + j;
                        selected_point.y = map_origin[idx].y + i;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(i,j) = 255;
                        break;
                    }
                }
            }
        }break;
        case 1:{
            for(int i=0;i<num_roi_rows;i++){
                for(int j=num_roi_cols-1;j>=0;j--){
                    if(divided_roi[idx].at<uchar>(i,j) == 255){
                        if(j==num_roi_cols-1)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + j;
                        selected_point.y = map_origin[idx].y + i;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(i,j) = 255;
                        break;
                    }
                }
            }
        }break;
        case 2:{
            for(int i=0;i<num_roi_cols;i++){
                for(int j=0;j<num_roi_rows;j++){
                    if(divided_roi[idx].at<uchar>(j,i) == 255){
                        if(j==0)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + i;
                        selected_point.y = map_origin[idx].y + j;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(j,i) = 255;
                        break;
                    }
                }
            }
        }break;
        case 3:{
            for(int i=0;i<num_roi_cols;i++){
                for(int j=num_roi_rows-1;j>=0;j--){
                    if(divided_roi[idx].at<uchar>(j,i) == 255){
                        if(j==num_roi_rows-1)continue;
                        cv::Point selected_point;
                        selected_point.x = map_origin[idx].x + i;
                        selected_point.y = map_origin[idx].y + j;
                        selected_data[idx].push_back(selected_point);
                        debug_mat[idx].at<uchar>(j,i) = 255;
                        break;
                    }
                }
            }
        }break;

        }
    }
}


float Cat2Detector::checkTargetHistogram(cv::Mat _target_roi){

    if(square_ref_.empty()){
        ROS_ERROR("[v11_cat2_detector] Cat2 reference not found !!!");
        return -1;
    }
    _target_roi = cvtMulti(_target_roi);
    cv::MatND target_hist;
    cv::calcHist(&_target_roi, 1, hist_param_.channels, cv::Mat(), target_hist, 2, hist_param_.hist_size, hist_param_.ranges);
    cv::normalize(target_hist,target_hist, .0, 1.0, cv::NORM_MINMAX);

    return cv::compareHist(square_ref_hist_, target_hist, CV_COMP_KL_DIV);
}

void Cat2Detector::publishLocalizationUtils(const cv::Mat &_inv_segmented_green,
                                                vision_utils::FieldBoundary _field_boundary){
    cv_inv_sg_pub_.image = _inv_segmented_green.clone();

    cv_inv_sg_pub_.header.seq++;
    _field_boundary.header.seq++;

    cv_inv_sg_pub_.header.stamp = this->stamp_;
    _field_boundary.header.stamp = this->stamp_;

    cv_inv_sg_pub_.header.frame_id = this->frame_id_;
    _field_boundary.header.frame_id = this->frame_id_;

    cv_inv_sg_pub_.encoding = sensor_msgs::image_encodings::MONO8;


    it_inv_sg_pub_.publish(cv_inv_sg_pub_.toImageMsg());
    field_boundary_pub_.publish(_field_boundary);
}

std::pair<cv::Mat, vision_utils::FieldBoundary > Cat2Detector::getImageContours(const cv::Mat &_segmented_green){
    cv::Mat _field_contour = cv::Mat::zeros(_segmented_green.size(), CV_8UC1);
    vision_utils::FieldBoundary field_boundary;
    Points contour_points;
    std::vector<Points > contours;
    std::vector<cv::Vec4i > hierarchy;

    cv::findContours(_segmented_green, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) > MIN_FIELD_CONTOUR_AREA){
            contour_points.insert(contour_points.end(), contours[i].begin(), contours[i].end());
        }
    }

    if(contour_points.size()){
        std::vector<Points > contour(1);
        cv::convexHull(contour_points,contour[0]);
        cv::Rect field_bound = cv::boundingRect(contour[0]);
        drawContours(_field_contour, contour, 0, cv::Scalar(255), cv::FILLED);
        //[HW] Scan from dual direction
        for(int i=field_bound.tl().x;
            i<field_bound.br().x;i++){
            geometry_msgs::Vector3 temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().y-1;
            for(int j=field_bound.tl().y;
                j<field_bound.br().y;j++){
                if(_field_contour.at<uchar >(j,i) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(j,i) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary.bound1.push_back(temp);
        }

        for(int i=field_bound.tl().y;
            i<field_bound.br().y;i++){
            geometry_msgs::Vector3 temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().x-1;
            for(int j=field_bound.tl().x;
                j<field_bound.br().x;j++){
                if(_field_contour.at<uchar >(i,j) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(i,j) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary.bound2.push_back(temp);
        }
    }

    std::pair<cv::Mat, vision_utils::FieldBoundary > result;
    result.first = _field_contour;
    result.second = field_boundary;
    return result;
}

cv::Vec2d linearParameters(cv::Vec4i line){
    cv::Mat a = (cv::Mat_<double>(2, 2)) << (line[0], 1,line[2], 1);
    cv::Mat y = (cv::Mat_<double>(2, 1)) <<(line[1], line[3]);
    cv::Vec2d mc; solve(a, y, mc);
    return mc;
}

cv::Vec4i extendedLine(cv::Vec4i line, double d){
    // oriented left-t-right
    cv::Vec4d _line = line[2] - line[0] < 0 ? cv::Vec4d(line[2], line[3], line[0], line[1]) : cv::Vec4d(line[0], line[1], line[2], line[3]);
    double m = linearParameters(_line)[0];
    // solution of pythagorean theorem and m = yd/xd
    double xd = sqrt(d * d / (m * m + 1));
    double yd = xd * m;
    return cv::Vec4d(_line[0] - xd, _line[1] - yd , _line[2] + xd, _line[3] + yd);
}


std::vector<cv::Point2i> boundingRectangleContour(cv::Vec4i line, float d){
    // finds coordinates of perpendicular lines with length d in both line points
    // https://math.stackexchange.com/a/2043065/183923

    cv::Vec2f mc = linearParameters(line);
    float m = mc[0];
    float factor = sqrtf(
        (d * d) / (1 + (1 / (m * m)))
    );

    float x3, y3, x4, y4, x5, y5, x6, y6;
    // special case(vertical perpendicular line) when -1/m -> -infinity
    if(m == 0){
        x3 = line[0]; y3 = line[1] + d;
        x4 = line[0]; y4 = line[1] - d;
        x5 = line[2]; y5 = line[3] + d;
        x6 = line[2]; y6 = line[3] - d;
    } else {
        // slope of perpendicular lines
        float m_per = - 1/m;

        // y1 = m_per * x1 + c_per
        float c_per1 = line[1] - m_per * line[0];
        float c_per2 = line[3] - m_per * line[2];

        // coordinates of perpendicular lines
        x3 = line[0] + factor; y3 = m_per * x3 + c_per1;
        x4 = line[0] - factor; y4 = m_per * x4 + c_per1;
        x5 = line[2] + factor; y5 = m_per * x5 + c_per2;
        x6 = line[2] - factor; y6 = m_per * x6 + c_per2;
    }

    return std::vector<cv::Point2i> {
        cv::Point2i(x3, y3),
        cv::Point2i(x4, y4),
        cv::Point2i(x6, y6),
        cv::Point2i(x5, y5)
    };
}

bool extendedBoundingRectangleLineEquivalence(const cv::Vec4i& _l1, const cv::Vec4i& _l2, float extensionLengthFraction, float maxAngleDiff, float boundingRectangleThickness){

    cv::Vec4i l1(_l1), l2(_l2);
    // extend lines by percentage of line width
    float len1 = sqrtf((l1[2] - l1[0])*(l1[2] - l1[0]) + (l1[3] - l1[1])*(l1[3] - l1[1]));
    float len2 = sqrtf((l2[2] - l2[0])*(l2[2] - l2[0]) + (l2[3] - l2[1])*(l2[3] - l2[1]));
    cv::Vec4i el1 = extendedLine(l1, len1 * extensionLengthFraction);
    cv::Vec4i el2 = extendedLine(l2, len2 * extensionLengthFraction);

    // reject the lines that have wide difference in angles
    float a1 = atan(linearParameters(el1)[0]);
    float a2 = atan(linearParameters(el2)[0]);
    if(fabs(a1 - a2) > maxAngleDiff * M_PI / 180.0){
        return false;
    }

    // calculate window around extended line
    // at least one point needs to inside extended bounding rectangle of other line,
    std::vector<cv::Point2i> lineBoundingContour = boundingRectangleContour(el1, boundingRectangleThickness/2);
    return
        pointPolygonTest(lineBoundingContour, cv::Point(el2[0], el2[1]), false) == 1 ||
        pointPolygonTest(lineBoundingContour, cv::Point(el2[2], el2[3]), false) == 1;
}
void scanLinePoints(cv::Mat _invert_green, cv::Mat _segmented_white, const std::vector<std::vector<cv::Point3i>> &_field_boundary,
                             Points &_target_points, int _orientation){

    if(_orientation){
        transpose(_invert_green, _invert_green);
        transpose(_segmented_white, _segmented_white);
    }
    
    std::vector<cv::Point3i> boundary = (_orientation==0) ? _field_boundary[0] : _field_boundary[1];
    for(size_t i = 0; i < boundary.size();
        i += 5){
        cv::Point target(boundary[i].x,-1);
        int start = boundary[i].y;
        while(_invert_green.at<uchar>(start, target.x) > 0 && start < boundary[i].z)
            start++;                       

        for(int j=start;j<boundary[i].z;j++){         
            if(_invert_green.at<uchar>(j, target.x) > 0 && target.y == -1){
                target.y = j;
            }else if(_invert_green.at<uchar>(j, target.x) == 0 && target.y != -1){
                int diff = j-target.y;
                if(diff < 100){
                    target.y = (target.y + (j-1))/2;
                    if(_segmented_white.at<uchar>(target.y, target.x))
                        _target_points.emplace_back(_orientation==1 ? cv::Point(target.y, target.x) : target);
                }
                target.y = -1;
            }
        }
    }
//    cv::imshow("DEBUG",debug);
//    cv::waitKey(0);
}
void scanLinePoint(const cv::Mat &_invert_green, const cv::Mat &_segmented_white, const std::vector<std::vector<cv::Point3i>> &_field_boundary,
                             Points &_target_points){

    scanLinePoints(_invert_green, _segmented_white, _field_boundary, _target_points, 0); //horizontal
    scanLinePoints(_invert_green, _segmented_white, _field_boundary, _target_points, 1); //vertikal

}

std::pair<cv::Mat, std::vector<std::vector<cv::Point3i>>> getFieldImage(const cv::Mat &_segmented_green){
    cv::Mat _field_contour = cv::Mat::zeros(_segmented_green.size(), CV_8UC1);
    std::vector<std::vector<cv::Point3i>> field_boundary;
    std::vector<cv::Point3i> field_boundary_temp;
    Points contour_points;
    std::vector<Points > contours;
    std::vector<cv::Vec4i > hierarchy;

    cv::findContours(_segmented_green, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) > 1600.0f){
            contour_points.insert(contour_points.end(), contours[i].begin(), contours[i].end());
        }
    }

    if(contour_points.size()){
        std::vector<Points > contour(1);
        cv::convexHull(contour_points,contour[0]);
        cv::Rect field_bound = cv::boundingRect(contour[0]);
        drawContours(_field_contour, contour, 0, cv::Scalar(255), cv::FILLED);
        //[HW] Scan from dual direction
        for(int i=field_bound.tl().x;
            i<field_bound.br().x;i++){
            cv::Point3i temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().y-1;
            for(int j=field_bound.tl().y;
                j<field_bound.br().y;j++){
                if(_field_contour.at<uchar >(j,i) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(j,i) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary_temp.push_back(temp);
        }

        field_boundary.insert(field_boundary.begin(), field_boundary_temp);
        field_boundary_temp.clear();  

        for(int i=field_bound.tl().y;
            i<field_bound.br().y;i++){
            cv::Point3i temp;
            temp.x = i;
            temp.y = -1;
            temp.z = field_bound.br().x-1;
            for(int j=field_bound.tl().x;
                j<field_bound.br().x;j++){
                if(_field_contour.at<uchar >(i,j) > 0 &&
                        temp.y==-1){
                    temp.y = j;
                }else if(_field_contour.at<uchar >(i,j) == 0 &&
                         temp.y!=-1){
                    temp.z = j-1;
                    break;
                }
            }
            field_boundary_temp.push_back(temp);
        }
    }
    field_boundary.insert(field_boundary.end(), field_boundary_temp);
    // field_boundary_temp.clear();

    
    std::pair<cv::Mat, std::vector<std::vector<cv::Point3i>>> result;
    result.first = _field_contour;
    result.second = field_boundary;
    return result;
}


void Cat2Detector::process(){
    if(cv_img_ptr_subs_ == nullptr)return;
    static geometry_msgs::Point last_square_pos_;
    line_pos_.x = -1;
    line_pos_.y = -1;
    line_pos_.z = 0;
//vertikal line baru
    line_pos_ver_.x = -1;
    line_pos_ver_.y = -1;
    line_pos_ver_.z = 0;
    square_pos_.x = -1;
    square_pos_.y = -1;
    square_pos_.z = 0;
    geometry_msgs::Point _square_pos;
    geometry_msgs::Point _line_pos;
//vertikal line baru
    geometry_msgs::Point _line_pos_ver;
    setInputImage() = cv_img_ptr_subs_->image;

    cv::Mat output_view = in_img_.clone();
    cv::Mat vertikal = in_img_.clone();
    cv::Mat segmented_green, inv_segmented_green, segmented_blue, segmented_white;
    cv::Mat thresh_image = segmentColor(segmented_green, inv_segmented_green, segmented_blue, segmented_white);
    //Buat masklap dari segmented green
    cv::cvtColor(segmented_green,segmented_green,cv::COLOR_BGR2GRAY);
    cv::Mat masklap;
    cv::threshold(segmented_green, masklap,100, 255, cv::THRESH_BINARY);
    //Buat maskgar dari segmented white
    cv::cvtColor(segmented_white,segmented_white,cv::COLOR_BGR2GRAY);
    cv::Mat maskgar;
    cv::threshold(segmented_white, maskgar,100, 255, cv::THRESH_BINARY);
    
    cv::Mat field_contour;
    std::pair<cv::Mat, vision_utils::FieldBoundary > field_prop = getImageContours(segmented_green);
    field_contour = field_prop.first;
    publishLocalizationUtils(inv_segmented_green,field_prop.second);

    // cv::dilate(square_in_background, square_in_background, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    // cv::erode(square_in_background, square_in_background, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

    cv::Mat square_contour;
    std::pair<cv::Mat, vision_utils::FieldBoundary > square_prop = getImageContours(segmented_blue);
    square_contour = square_prop.first;

    std::vector<Points > contours;
    //Approx NONE to get the authentic contours
    cv::findContours(square_contour, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //for square
    cv::Mat ROI;
    cv::Rect square_roi;
    cv::Mat square;
    for(size_t i = 0; i < contours.size(); i++){
        float contour_area = cv::contourArea(contours[i]);
        // std::cout<<"contour_area : "<<contour_area<<std::endl; //max65000-69999
        if(contour_area > 68000){
            cv::Rect rect_rough_roi = cv::boundingRect(contours[i]);
//            cv::Point br_pt = rect_rough_roi.br();

            cv::Mat roi_hsv_ = cv::Mat(in_hsv_,rect_rough_roi);

            float histogram_score = checkTargetHistogram(roi_hsv_);
            // std::cout<<"Histogram : "<<histogram_score<<std::endl;

            if(histogram_score < (float)config_.min_thres / 10.0f)continue;

            cv::Mat frame_rough_roi(square_contour,rect_rough_roi);
        
            cv::Point tl_pt = rect_rough_roi.tl();




            float roi_ratio = (rect_rough_roi.width < rect_rough_roi.height)?(float)rect_rough_roi.width/(float)rect_rough_roi.height:
                                                                             (float)rect_rough_roi.height/(float)rect_rough_roi.width;
            // std::cout<<"Ratio 1 : "<<roi_ratio<<std::endl;
            // std::cout<<"Ratio 2 : "<<1.0f/roi_ratio<<std::endl;
            std::vector<cv::Mat> sub_frame;
            // if(roi_ratio >= (float)config_.ratio1/100.0f && 1.0f/roi_ratio <= (float)config_.ratio2/100.0f){ //0.30  kanan->3.16
                // if(contour_area < 271856){
                    Points outer_rect;
                    cv::convexHull(contours[i],outer_rect);
                    cv::Moments moment;
                    moment = cv::moments(outer_rect,true);
                    cv::Point square_com(moment.m10/moment.m00, moment.m01/moment.m00);
                          
                    _square_pos.x = int(moment.m10/moment.m00);
                    _square_pos.y = int(moment.m01/moment.m00);

                    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
                    std::vector<cv::Rect> boundRect( contours.size() );
                    std::vector<cv::Point2f> centers( contours.size() );

                    cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
                    boundRect[i] = cv::boundingRect( contours_poly[i] );

                    cv::Mat output_view_roi(output_view,boundRect[i]);
                    cv::Mat square_region(output_view_roi.size(),CV_8UC3,cv::Scalar(50,50,255));
                    square = square_region.clone();
                    cv::addWeighted(output_view_roi, .5, square_region, .5, .0,output_view_roi);

                    if(!contours.size()){
                        square_pos_.x = -1;
                        square_pos_.y = -1;
                        square_pos_.z = 0;
                    }else {
                        square_pos_.x = _square_pos.x;
                        square_pos_.y = _square_pos.y;
                        square_pos_.z = 0;
                    }

                    // std::cout<<"square_pos_.x : "<<square_pos_.x<<std::endl;
                    // std::cout<<"square_pos_.y : "<<square_pos_.y<<std::endl;
                // }
            // }
        }
    }
    //for line
    cv::Mat field_for_line = cv::Mat::zeros(thresh_image.size(), CV_8UC1);

    Points contour_points_for_line;

    std::vector<Points > contours_field;
    std::vector<cv::Vec4i > hierarchy_field;
    std::vector<Points > contours_square;
    std::vector<cv::Vec4i > hierarchy_square;

    cv::findContours(field_contour, contours_field, hierarchy_field, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for(size_t i = 0; i < contours_field.size(); i++){
        if(cv::contourArea(contours_field[i]) > MIN_FIELD_CONTOUR_AREA){
            contour_points_for_line.insert(contour_points_for_line.end(), contours_field[i].begin(), contours_field[i].end());
        }
    }

    cv::findContours(square_contour, contours_square, hierarchy_square, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for(size_t i = 0; i < contours_square.size(); i++){
        if(cv::contourArea(contours_square[i]) > MIN_FIELD_CONTOUR_AREA){
            contour_points_for_line.insert(contour_points_for_line.end(), contours_square[i].begin(), contours_square[i].end());
        }
    }

    if(contours_field.size() || contours_square.size()){
        std::vector<Points > contour(1);
        cv::convexHull(contour_points_for_line,contour[0]);
        cv::Rect field_bound = cv::boundingRect(contour[0]);
        drawContours(field_for_line, contour, 0, cv::Scalar(255), cv::FILLED);
    }

    cv::Mat line_contour;
    std::pair<cv::Mat, vision_utils::FieldBoundary > line_prop = getImageContours(segmented_white);
    line_contour = line_prop.first;
    cv::bitwise_and(line_contour,field_for_line,line_contour);
    //cv::imshow("line_contour",line_contour);

    cv::Mat _lines, image_thres;
    cv::cvtColor(line_contour, _lines, CV_GRAY2BGR);
    cv::bitwise_and(_lines, in_img_, _lines);
    cv::cvtColor(_lines, _lines, CV_BGR2RGB);
    cv::cvtColor(_lines, _lines, CV_BGR2GRAY);

// #### Horizontal Line
    // cv::Mat bw;
    // cv::adaptiveThreshold(_lines, bw, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, -2);

    // cv::Mat horizontal = bw.clone();

    // int horizontal_size = horizontal.cols / 7.5;
    // cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horizontal_size, 1));

    // cv::erode(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));
    // cv::dilate(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));

    // std::vector<std::vector<cv::Point> > contours_lines;
    // Points contour_pt_ine;
    // cv::findContours(horizontal,contours_lines, CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    // for(size_t i = 0; i < contours_lines.size(); i++){
    //     // std::cout<<"cv::contourArea(contours_lines[i]) : "<<cv::contourArea(contours_lines[i])<<std::endl;
    //     if(cv::contourArea(contours_lines[i]) > 150){
    //         contour_pt_ine.insert(contour_pt_ine.end(), contours_lines[i].begin(), contours_lines[i].end());
    //     }
    // }

    // cv::Mat horizontal_line = cv::Mat::zeros(thresh_image.size(), CV_8UC1);
    // if(contour_pt_ine.size()){
    //     std::vector<Points > contour(1);
    //     cv::convexHull(contour_pt_ine,contour[0]);
    //     cv::Rect field_bound = cv::boundingRect(contour[0]);

    //     drawContours(horizontal_line, contour, 0, cv::Scalar(255), cv::FILLED);
    // }

    // std::vector<Points > contours_lineH;
    // //for gui
    // //Approx NONE to get the authentic contours
    // cv::findContours(horizontal_line, contours_lineH, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // for(size_t i = 0; i < contours_lineH.size(); i++){

    //     Points outer_rect;
    //     cv::convexHull(contours_lineH[i],outer_rect);
    //     cv::Moments moment;
    //     moment = cv::moments(outer_rect,true);
    //     cv::Point line_com(moment.m10/moment.m00, moment.m01/moment.m00);
                
    //     _line_pos.x = int(moment.m10/moment.m00);
    //     _line_pos.y = int(moment.m01/moment.m00);

    //     cv::Point line_center;
    //     line_center = cv::Point(moment.m10/moment.m00,moment.m01/moment.m00);
    //     cv::circle(output_view, line_center, 1, cv::Scalar(255,0,0), 3, 8, 0 );

    //     std::vector<std::vector<cv::Point> > contours_poly( contours_lineH.size() );
    //     std::vector<cv::Rect> boundRect( contours_lineH.size() );
    //     std::vector<cv::Point2f> centers( contours_lineH.size() );

    //     cv::approxPolyDP( contours_lineH[i], contours_poly[i], 3, true );
    //     boundRect[i] = cv::boundingRect( contours_poly[i] );

    //     cv::Mat output_view_roi(output_view,boundRect[i]);
    //     cv::Mat square_region(output_view_roi.size(),CV_8UC3,cv::Scalar(255,50,50));
    //     square = square_region.clone();
    //     cv::addWeighted(output_view_roi, .5, square_region, .5, .0,output_view_roi);

    //     if(!contours_lineH.size()){
    //         line_pos_.x = -1;
    //         line_pos_.y = -1;
    //         line_pos_.z = 0;
    //     }else {
    //         line_pos_.x = _line_pos.x;
    //         line_pos_.y = _line_pos.y;
    //         line_pos_.z = 0;
    //     }

    //     std::cout<<"line_pos_.x : "<<line_pos_.x<<std::endl;
    //     std::cout<<"line_pos_.y : "<<line_pos_.y<<std::endl;

    // }  
    
// #### Vertical Line
    // //BEGINNING OF COMMENT
    // cv::threshold(_lines, image_thres, config_.min_thres, 255, cv::THRESH_BINARY);
    // //cv::bitwise_and(segmented_white,field_contour, image_thres);
    // //cv::imshow("segmented white",image_thres);

    // cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    // //cv::erode(image_thres, image_thres, element);
    // //cv::dilate(image_thres, image_thres, element,cv::Point(-1,-1));
    // cv::Mat skel(image_thres.size(), CV_8UC1, cv::Scalar(0));
    //         cv::Mat temp;
    //         cv::Mat eroded;


    //         bool done;
    //         do
    //         {
    //           cv::erode(image_thres, eroded, element);
    //           cv::dilate(eroded, temp, element); // temp = open(img)
    //           cv::subtract(image_thres, temp, temp);
    //           cv::bitwise_or(skel, temp, skel);
    //           eroded.copyTo(image_thres);

    //           done = (cv::countNonZero(image_thres) == 0);
    //         } while (!done);

    //         cv::Mat dot = in_img_.clone();
    //         std::vector<cv::Vec4i> lines; // will hold the results of the detection
    //             cv::HoughLinesP(skel, lines, 2, CV_PI/180, 80,0,80 ); // runs the actual detection
    //             // Draw the lines
    // //ENDING OF COMMENT
    
     //skeleton
    std::pair<cv::Mat, std::vector<std::vector<cv::Point3i>>> field = getFieldImage(masklap);
    field_contour = field.first;
    // cout<<field.second[1].size()<<endl;
    Points target_point;
    scanLinePoint(inv_segmented_green,maskgar,field.second,target_point);


    
    cv::Mat skel = cv::Mat::zeros(thresh_image.size(),CV_8UC1);
    cv::Mat ver_image = in_img_.clone();
    cv::Mat hori_image = in_img_.clone();

    // for(Points::const_iterator it=target_point.begin();
    // it != target_point.end(); it++){  
    //     cout<<*it<<endl;
    //     // skel.at<cv::Vec3b>(*it) = cv::Scalar(255,255,255);
    // }

    for (size_t i = 0;i < target_point.size(); i++){
        uchar & color = skel.at<uchar>(target_point[i].y,target_point[i].x);
        color=255;
        // color[1]=255;
        // color[2]=255;
    }

    //DIlate 2d point
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(5,5));
    // cv::morphologyEx(skel,skel,cv::MORPH_CLOSE,kernel,cv::Point(-1,-1),2);
    cv::dilate(skel,skel,kernel);
    
    //Hough Line dari skeleton
    std::vector<cv::Vec4i> lines;
    HoughLinesP(skel,lines,2, CV_PI/180,80,0,30);
    
    // remove small lines
    std::vector<cv::Vec4i> linesWithoutSmall;
    std::copy_if (lines.begin(), lines.end(), std::back_inserter(linesWithoutSmall), [](cv::Vec4f line){
        float length = sqrtf((line[2] - line[0]) * (line[2] - line[0])
                             + (line[3] - line[1]) * (line[3] - line[1]));
        return length > 30;
    });

    // std::cout << "Detected: " << linesWithoutSmall.size() << std::endl;

    // partition via our partitioning function
    std::vector<int> labels;
    int equilavenceClassesCount = cv::partition(linesWithoutSmall, labels, [](const cv::Vec4i l1, const cv::Vec4i l2){
        return extendedBoundingRectangleLineEquivalence(
            l1, l2,
            // line extension length - as fraction of original line width
            0.2,
            // maximum allowed angle difference for lines to be considered in same equivalence class
            10,
            // thickness of bounding rectangle around each line
            10);
    });

    // std::cout << "Equivalence classes: " << equilavenceClassesCount << std::endl;

    // grab a random colour for each equivalence class
    cv::RNG rng(215526);
    std::vector<cv::Scalar> colors(equilavenceClassesCount);
    for (int i = 0; i < equilavenceClassesCount; i++){
        colors[i] = cv::Scalar(rng.uniform(30,255), rng.uniform(30, 255), rng.uniform(30, 255));;
    }

    // draw original detected lines
    cv::Mat detectedLinesImg = cv::Mat::zeros(Alfarobi::FRAME_HEIGHT, Alfarobi::FRAME_WIDTH, CV_8UC3);
    for (int i = 0; i < linesWithoutSmall.size(); i++){
        cv::Vec4i& detectedLine = linesWithoutSmall[i];
        cv::line(detectedLinesImg,
             cv::Point(detectedLine[0], detectedLine[1]),
             cv::Point(detectedLine[2], detectedLine[3]), colors[labels[i]], 2);
    }

    // build point clouds out of each equivalence classes
    std::vector<std::vector<cv::Point2i>> pointClouds(equilavenceClassesCount);
    for (int i = 0; i < linesWithoutSmall.size(); i++){
        cv::Vec4i& detectedLine = linesWithoutSmall[i];
        pointClouds[labels[i]].push_back(cv::Point2i(detectedLine[0], detectedLine[1]));
        pointClouds[labels[i]].push_back(cv::Point2i(detectedLine[2], detectedLine[3]));
    }

    // fit line to each equivalence class point cloud
    std::vector<cv::Vec4i> reducedLines = accumulate(pointClouds.begin(), pointClouds.end(), std::vector<cv::Vec4i>{}, [](std::vector<cv::Vec4i> target, const std::vector<cv::Point2i>& _pointCloud){
        std::vector<cv::Point2i> pointCloud = _pointCloud;

        //lineParams: [vx,vy, x0,y0]: (normalized vector, point on our contour)
        // (x,y) = (x0,y0) + t*(vx,vy), t -> (-inf; inf)
        cv::Vec4f lineParams; cv::fitLine(pointCloud, lineParams, CV_DIST_L2, 0, 0.01, 0.01);

        // derive the bounding xs of point cloud
        decltype(pointCloud)::iterator minXP, maxXP;
        std::tie(minXP, maxXP) = std::minmax_element(pointCloud.begin(), pointCloud.end(), [](const cv::Point2i& p1, const cv::Point2i& p2){ return p1.x < p2.x; });

        // derive y coords of fitted line
        float m = lineParams[1] / lineParams[0];
        int y1 = ((minXP->x - lineParams[2]) * m) + lineParams[3];
        int y2 = ((maxXP->x - lineParams[2]) * m) + lineParams[3];

        target.push_back(cv::Vec4i(minXP->x, y1, maxXP->x, y2));
        return target;
    });

    std::vector<cv::Vec4i> garis_hor;
    std::vector<cv::Vec4i> garis_ver;
    // cv::Point p1, p2;
    cv::Mat frameclone = in_img_.clone();
    for(cv::Vec4i reduced: reducedLines){
        //cv::line(frameclone, cv::Point(reduced[0], reduced[1]), cv::Point(reduced[2], reduced[3]), cv::Scalar(255, 0, 0), 2);
        //double x = (reduced[2]+reduced[0])/2;
        //double y = (reduced[3]+reduced[1])/2;
        cv::Point p1, p2;
            p1=cv::Point(reduced[0], reduced[1]);
            p2=cv::Point(reduced[2], reduced[3]);

            double angle = (atan2 ( fabs(p2.y-p1.y)*1.0 , fabs(p2.x-p1.x)*1.0))*(180/CV_PI);
            double center_x = (reduced[2]+reduced[0])/2;
            double center_y = (reduced[3]+reduced[1])/2;
            
            /*
            std::cout <<"Point 1" <<p1 <<",  Point 2"<< p2
                              << "  angle :" << angle
                              <<'\n';
            */
             std::cout<<"center.x : "<<center_x<<std::endl;
            std::cout<<"center.y : "<<center_y<<std::endl;
            //horizontal lines
                if( angle < 45 && angle > 0)
                    {
                    // garis_hor.push_back(reduced);
                    cv::line(output_view, p1, p2, cv::Scalar(255,0,0), 3, cv::LINE_AA); //biru
                    cv::circle(output_view, cv::Point(center_x,center_y),6,cv::Scalar(0,255,0),-1,8,0);

                    //line( horizontal, p1, p2, Scalar(0,0,255), 3, CV_AA);
                    }
                else if(angle >= 45){
                    // garis_ver.push_back(reduced);
                    cv::line(output_view, p1, p2, cv::Scalar(0,0,255), 3, cv::LINE_AA); //merah
                    cv::circle(output_view, cv::Point(center_x,center_y),6,cv::Scalar(0,0,0),-1,8,0);

                }

        //circle(frameclone, Point(x,y),3,Scalar(255,0,0),-1,8,0);
    }

    //garis horizontal
    // for( size_t i = 0; i < garis_hor.size(); i++ ){
    //  cv::Vec4i garis_horizontal = garis_hor[i];
    //  cv::Point point1, point2;
    //     _line_pos.x = int((garis_horizontal[2]+garis_horizontal[0])/2);
    //     _line_pos.y = int((garis_horizontal[3]+garis_horizontal[1])/2);

    //     int center_x = (garis_horizontal[2]+garis_horizontal[0])/2;
    //     int center_y = (garis_horizontal[3]+garis_horizontal[1])/2;

    //      point1=cv::Point(garis_horizontal[0], garis_horizontal[1]);
    //      point2=cv::Point(garis_horizontal[2], garis_horizontal[3]);
    // double angle = (atan2 ( fabs(point2.y-point1.y)*1.0 , fabs(point2.x-point1.x)*1.0))*(180/CV_PI);

    
    // std::cout <<"selected Point 1" <<point1 <<", selected Point 2"<< point2
    //                           << " selected angle :" << angle
    //                           <<'\n';
    // std::cout << "center point (" << center_x << "," << center_y << ") \n";
    

    //      cv::line( output_view, point1, point2, cv::Scalar(255,0,0), 4, cv::LINE_AA);
    //      cv::circle(output_view, cv::Point(center_x,center_y),6,cv::Scalar(0,0,255),-1,8,0);



    //     // std::cout<<"line_pos_.x : "<<line_pos_.x<<std::endl;
    //     // std::cout<<"line_pos_.y : "<<line_pos_.y<<std::endl;
    //      if(i==0){
    //          break;
    //      }   
    // }
    //      if(!garis_hor.size()){
    //         line_pos_.x = -1;
    //         line_pos_.y = -1;
    //         line_pos_.z = 0;
    //     }else {
    //         line_pos_.x = _line_pos.x;
    //         line_pos_.y = _line_pos.y;
    //         line_pos_.z = 0;
    //     }

//     //garis vertikal
    for( size_t i = 0; i < garis_ver.size(); i++ ){
     cv::Vec4i garis_vertikal = garis_ver[i]; 
     cv::Point point1, point2;
//vertikal line baru
        _line_pos_ver.x = int((garis_vertikal[2]+garis_vertikal[0])/2);
        _line_pos_ver.y = int((garis_vertikal[3]+garis_vertikal[1])/2);

        double center_x = (garis_vertikal[2]+garis_vertikal[0])/2;
        double center_y = (garis_vertikal[3]+garis_vertikal[1])/2;

         point1=cv::Point(garis_vertikal[0], garis_vertikal[1]);
         point2=cv::Point(garis_vertikal[2], garis_vertikal[3]);
    double angle = (atan2 ( fabs(point2.y-point1.y)*1.0 , fabs(point2.x-point1.x)*1.0))*(180/CV_PI);
    
    /*
    std::cout <<"selected Point 1" <<point1 <<", selected Point 2"<< point2
                              << " selected angle :" << angle
                              <<'\n';
    std::cout << "center point (" << center_x << "," << center_y << ") \n";  
    */
         cv::line(vertikal, point1, point2, cv::Scalar(255,0,0), 4, cv::LINE_AA);
         cv::circle(vertikal, cv::Point(center_x,center_y),6,cv::Scalar(0,0,255),-1,8,0);
    
    
         if(i==0){
             break;
         }
    
    }
//vertikal line baru
            if(!garis_ver.size()){
            line_pos_ver_.x = -1;
            line_pos_ver_.y = -1;
            line_pos_ver_.z = 0;
        }else {
            line_pos_ver_.x = _line_pos_ver.x;
            line_pos_ver_.y = _line_pos_ver.y;
            line_pos_ver_.z = 0;
        }
//vertikal line baru
    line_pos_pub_.publish(line_pos_);
    line_pos_ver_pub_.publish(line_pos_ver_);
    square_pos_pub_.publish(square_pos_);
    // cv::imshow("Detected Lines", detectedLinesImg);
    //imshow("Reduced Lines", reducedLinesImg);
    cv::imshow("skeleton",skel);
    // cv::imshow("segmented white",segmented_white);
    //cv::imshow("hasil awal",frameclone);
    //cv::waitKey(27);
    // cv::imshow("horizontal",output_view);
    // cv::imshow("vertikal",vertikal);

    //for gui
    //Approx NONE to get the authentic contours

        // std::cout<<"line_pos_.x : "<<line_pos_.x<<std::endl;
        // std::cout<<"line_pos_.y : "<<line_pos_.y<<std::endl;

    

    // cv::imshow("horizontal_line", horizontal_line);
    // cv::imshow("field_contour", field_contour);
    // cv::imshow("square_contour", square_contour);
    cv::waitKey(1);


#if DEBUG == 1
    std::cout<<Success<<std::endl;
#endif

    //For purpose GUI only
    switch(frame_mode_){
    case 1:setOutputImage(in_hsv_); break;
    case 2:setOutputImage(thresh_image);break;
    case 3:
    
    setOutputImage(output_view);break;
    default:setOutputImage(in_img_);break;
    }
    publishImage();
}