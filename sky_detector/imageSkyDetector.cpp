/************************************************
* Author: MaybeShewill-CV
* File: imageSkyDetector.cpp
* Date: 18-6-28 上午10:28
************************************************/

#include "imageSkyDetector.h"

#include <fstream>
#include <chrono>

#include <glog/logging.h>

#include <file_system_processor.h>

namespace sky_detector {

/***
 * Copy class constructor
 * @param _SkyAreaDetector
 */
SkyAreaDetector::SkyAreaDetector(const SkyAreaDetector &_SkyAreaDetector) {
    this->f_thres_sky_max = _SkyAreaDetector.f_thres_sky_max;
    this->f_thres_sky_min = _SkyAreaDetector.f_thres_sky_min;
    this->f_thres_sky_search_step = _SkyAreaDetector.f_thres_sky_search_step;
}
/***
 * Copy class constructor
 * @param _SkyAreaDetector
 * @return
 */
SkyAreaDetector& SkyAreaDetector::operator=(const SkyAreaDetector &_SkyAreaDetector) {
    this->f_thres_sky_max = _SkyAreaDetector.f_thres_sky_max;
    this->f_thres_sky_min = _SkyAreaDetector.f_thres_sky_min;
    this->f_thres_sky_search_step = _SkyAreaDetector.f_thres_sky_search_step;

    return *this;
}

/***
 * Read image file
 * @param image_file_path
 * @return
 */
bool SkyAreaDetector::load_image(const std::string &image_file_path) {
    if (!file_processor::FileSystemProcessor::is_file_exist(image_file_path)) {
        LOG(ERROR) << "Image file: " << image_file_path << "does not exist" << std::endl;
        return false;
    }

    _src_img = cv::imread(image_file_path, cv::IMREAD_UNCHANGED);

    if (_src_img.channels() == 1)
        cv::cvtColor(_src_img, _src_img, CV_GRAY2BGR);

//    cv::imwrite("/home/hunterlew/tmp.png", _src_img);

    std::cout << "loading " << image_file_path << std::endl;
    std::cout << "image size: " << _src_img.size << std::endl;
    cv::resize(_src_img, _src_img, cv::Size(1240, 360));

    assert (_src_img.channels() == 3);

    if (_src_img.empty() || !_src_img.data) {
        LOG(ERROR) << "Image file: " << image_file_path << "read failed" << std::endl;
        return false;
    }

    return true;
}

/***
 * Extract image sky area
 * @param bgrimg
 * @param skybinaryimg
 * @param horizonline
 * @return
 */
bool SkyAreaDetector::extract_sky(const cv::Mat &src_image, cv::Mat &sky_mask) {

    int image_height = src_image.size[0];
    int image_width = src_image.size[1];

    std::vector<int> sky_border_optimal = extract_border_optimal(src_image);

//    std::vector<int> border_diff(static_cast<int>(sky_border_optimal.size() - 1), 0);

//    if (!has_sky_region(sky_border_optimal, border_diff, image_height / 30, image_height / 10, 5)) {
//#ifdef DEBUG
//        LOG(INFO) << "没有提取到天空区域" << std::endl;
//#endif
//        LOG(INFO) << "find no sky" << std::endl;
//        return false;
//    }

#ifdef DEBUG
    cv::Mat sky_image;
    display_sky_region(src_image, optimized_border, sky_image);
    cv::imwrite("sky.jpg", sky_image);
    cv::imshow("sky image without refine", sky_image);
    cv::waitKey();
#endif

//    // hard to find threshold, so always refine the sky border
//    /*if (has_partial_sky_region(sky_border_optimal, border_diff, image_width / 3)) */{
//        LOG(INFO) << "find partial sky and refine it" << std::endl;
//        std::vector<int> border_new = refine_border(sky_border_optimal, src_image);
//        sky_mask = make_sky_mask(src_image, border_new);
//#ifdef DEBUG
//        display_sky_region(src_image, optimized_border, sky_image);
//        cv::imshow("sky image with refine", sky_image);
//        cv::waitKey();
//#endif
//        return true;
//    }

    check_sky_border_by_gray_value(src_image, sky_border_optimal);

    sky_mask = make_sky_mask(src_image, sky_border_optimal);

    return true;
}

void SkyAreaDetector::check_sky_border_by_gray_value(const cv::Mat &src_image,
                                                     std::vector<int> &sky_border_optimal) {
    int image_height = src_image.size[0];
    int image_width = src_image.size[1];

    assert (image_width == sky_border_optimal.size());

    cv::Mat gray_image;
    if (_src_img.channels() != 1)
        cv::cvtColor(_src_img, gray_image, CV_BGR2GRAY);

    for (int i=0; i<image_width; ++i)
    {
        int border = sky_border_optimal[i];

        for (int j=0; j<border; ++j)
        {
            uchar val = gray_image.at<uchar>(j, i);
            if (val < 128)
            {
                sky_border_optimal[i] = -1;
                break;
            }
        }

        if (border != -1
            && (i>1 && sky_border_optimal[i-1] == -1)
            && (i<image_width-1 && sky_border_optimal[i+1] == -1))
        {
            sky_border_optimal[i] = -1;
        }
    }

    // remove those with short width
    int p = -1;
    int q = -1;
    for (int i=0; i<image_width; ++i)
    {
        int border = sky_border_optimal[i];

        if (border != -1)  // find left valid border [
        {
            p = i;

            if (i == image_width - 1)
                q = image_width;

            for (int j=i+1; j<image_width; ++j)
            {
                int border_tmp = sky_border_optimal[j];

                if (border_tmp == -1 || j == image_width - 1)  // find right valid border )
                {
                    q = j;
                    if (j == image_width - 1)
                        q = image_width;

                    break;
                }
            }

            assert (q > p);

            if (q > p)
                printf("find valid len: %d\n", q-p);

            if (q > p && q - p < f_thres_sky_width)
            {
                for (int j=p; j<q; ++j)
                    sky_border_optimal[j] = -1;
            }

            i = q;
        }
    }
}

/***
 * Detect Image Sky Area Interface
 * @param image_file_dir
 * @param check_output_dir
 */
void SkyAreaDetector::detect(const std::string &image_file_path, const std::string &output_path) {

    LOG(INFO) << "Start detecting images: " << image_file_path;

    // Load image
    load_image(image_file_path);

    // Extract image sky area
    cv::Mat sky_mask;

    auto start_t = std::chrono::high_resolution_clock::now();

    if (extract_sky(_src_img, sky_mask)) {

        // 制作掩码输出
        _src_img.setTo(cv::Scalar(0, 0, 255), sky_mask);

        cv::imwrite(output_path, _src_img);

        auto end_t = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> cost_time = end_t - start_t;

        LOG(INFO) << "---- " << image_file_path << " ---- "
                  << cost_time.count() << "s" << std::endl;
    } else {

        cv::imwrite(output_path, _src_img);

        LOG(INFO) << "---- " << image_file_path << " ---- "
                  << "Null s" << std::endl;
    }
}

void SkyAreaDetector::batch_detect(const std::string &image_dir, const std::string &output_dir) {

    // Get image information
    std::vector<std::string> image_file_list;
    file_processor::FileSystemProcessor::get_directory_files(image_dir,
            image_file_list,
            ".png",
            file_processor::FileSystemProcessor::
            SEARCH_OPTION_T::ALLDIRECTORIES);

    LOG(INFO) << "Start batch extraction of sky regions";
    LOG(INFO) << "--- Image: --- Time(s): ---";

    for (auto &image_file : image_file_list) {

        auto start_t = std::chrono::high_resolution_clock::now();

        auto image_file_name = file_processor::FileSystemProcessor::get_file_name(image_file);
        auto output_path = file_processor::FileSystemProcessor::combine_path(output_dir, image_file_name);

        // Load image
        load_image(image_file);

        // Extract sky area
        cv::Mat sky_mask;

        if (extract_sky(_src_img, sky_mask)) {

            // 制作掩码输出
            _src_img.setTo(cv::Scalar(0, 0, 255), sky_mask);

            cv::imwrite(output_path, _src_img);

            auto end_t = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> cost_time = end_t - start_t;

            LOG(INFO) << "---- " << image_file_name << " ---- "
                      << cost_time.count() << "s" << std::endl;
        } else {

            cv::imwrite(output_path, _src_img);

            LOG(INFO) << "---- " << image_file_name << " ---- "
                      << "Null s" << std::endl;
        }
    }

    LOG(INFO) << "Batch extraction completed";
}

/***
 * Extract image gradient information
 * @param src_image
 * @param gradient_image
 */
void SkyAreaDetector::extract_image_gradient(const cv::Mat &src_image, cv::Mat &gradient_image) {
    // Convert grayscale image
    cv::Mat gray_image;
    cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
    // Sobel operator to extract image gradient information
    cv::Mat x_gradient;
    cv::Sobel(gray_image, x_gradient, CV_64F, 1, 0);
    cv::Mat y_gradient;
    cv::Sobel(gray_image, y_gradient, CV_64F, 0, 1);
    // Computational gradient infographic
    cv::Mat gradient;
    cv::pow(x_gradient, 2, x_gradient);
    cv::pow(y_gradient, 2, y_gradient);
    cv::add(x_gradient, y_gradient, gradient);
    cv::sqrt(gradient, gradient);

    gradient_image = gradient;

}

/***
 * Calculate the sky boundary
 * @param src_image
 * @return
 */
std::vector<int> SkyAreaDetector::extract_border_optimal(const cv::Mat &src_image) {
    // Extract gradient infographics
    cv::Mat gradient_info_map;
    extract_image_gradient(src_image, gradient_info_map);

//    cv::imwrite("/home/hunterlew/grad.png", gradient_info_map);

    int n = static_cast<int>(std::floor((f_thres_sky_max - f_thres_sky_min)
                                        / f_thres_sky_search_step)) + 1;

    int image_height = gradient_info_map.size[0];
    int image_width = gradient_info_map.size[1];

    std::vector<int> border_opt(image_width, image_height - 1);
    std::vector<int> b_tmp(image_width, image_height - 1);

    double jn_max = 0.0;

    double step = (std::floor((f_thres_sky_max - f_thres_sky_min) / n) - 1);

    for (int k = 1; k < n + 1; ++k) {
        double t = f_thres_sky_min + step * (k - 1);

        extract_border(b_tmp, gradient_info_map, t, src_image);
        double jn = calculate_sky_energy(b_tmp, src_image);

//        printf("%d: %.20lf\n", k, jn);

        if (std::isinf(jn)) {
            LOG(INFO) << "Jn is -inf" << std::endl;
        }

        if (jn > jn_max) {
            jn_max = jn;
            border_opt = b_tmp;
        }
    }

    return border_opt;
}

/***
 * Calculate the sky boundary
 * @param gradient_info_map
 * @param thresh
 * @return
 */
void SkyAreaDetector::extract_border(std::vector<int> &border,
                                     const cv::Mat &gradient_info_map,
                                     double thresh,
                                     const cv::Mat &src_image) {
    int image_height = gradient_info_map.size[0];
    int image_width = gradient_info_map.size[1];

    assert (image_width == border.size());

#pragma omp parallel for
    for (int col = 0; col < image_width; ++col) {
        int row_index = -1;
        for (int row = 0; row < image_height; ++row) {
            row_index = row;

            if (gradient_info_map.at<double>(row, col) > thresh) {

                // for gray image
                if (src_image.at<cv::Vec3b>(row, col)[0] == src_image.at<cv::Vec3b>(row, col)[1]
                    && src_image.at<cv::Vec3b>(row, col)[0] == src_image.at<cv::Vec3b>(row, col)[2]) {

                    double grad_y = (2*src_image.at<cv::Vec3b>(row+1, col)[0]
                            + src_image.at<cv::Vec3b>(row+1, col+1)[0]
                            + src_image.at<cv::Vec3b>(row+1, col-1)[0])
                            - (2*src_image.at<cv::Vec3b>(row-1, col)[0]
                            + src_image.at<cv::Vec3b>(row-1, col+1)[0]
                            + src_image.at<cv::Vec3b>(row-1, col-1)[0]);

                    if (grad_y > 0)  // down white, up black
                        border[col] = -1;
                    else
                        border[col] = row;

                    break;
                }
                else
                {
                    border[col] = row;
                    break;
                }
            }

            if (row_index >= image_height / 2)
            {
                border[col] = -1;
                break;
            }
        }

        if (row_index >= image_height / 2)
            border[col] = -1;

        if (row_index <= 5) {
            border[col] = -1;
        }
    }

//    for (int h : border)
//        std::cout << h << ", ";
//    std::cout << std::endl;
}

/***
 * Improve sky boundaries
 * @param border
 * @param src_image
 * @return
 */
std::vector<int> SkyAreaDetector::refine_border(const std::vector<int> &border,
        const cv::Mat &src_image) {

    int image_height = src_image.size[0];
    int image_width = src_image.size[1];

    // Make a sky image mask and a ground image mask
    cv::Mat sky_mask = make_sky_mask(src_image, border, 1);
    cv::Mat ground_mask = make_sky_mask(src_image, border, 0);

    // Deduction of sky images and ground images
    cv::Mat sky_image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    cv::Mat ground_image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    src_image.copyTo(sky_image, sky_mask);
    src_image.copyTo(ground_image, ground_mask);

    // Compute sky and ground image covariance matrices
    int ground_non_zeros_nums = cv::countNonZero(ground_mask);
    int sky_non_zeros_nums = cv::countNonZero(sky_mask);

    cv::Mat ground_image_non_zero = cv::Mat::zeros(ground_non_zeros_nums, 3, CV_8UC1);
    cv::Mat sky_image_non_zero = cv::Mat::zeros(sky_non_zeros_nums, 3, CV_8UC1);

    int row_index = 0;
    for (int col = 0; col < ground_image.cols; ++col) {
        for (int row = 0; row < ground_image.rows; ++row) {
            if (ground_image.at<cv::Vec3b>(row, col)[0] == 0 &&
                    ground_image.at<cv::Vec3b>(row, col)[1] == 0 &&
                    ground_image.at<cv::Vec3b>(row, col)[2] == 0) {
                continue;
            } else {
                cv::Vec3b intensity = ground_image.at<cv::Vec3b>(row, col);
                ground_image_non_zero.at<uchar>(row_index, 0) = intensity[0];
                ground_image_non_zero.at<uchar>(row_index, 1) = intensity[1];
                ground_image_non_zero.at<uchar>(row_index, 2) = intensity[2];
                row_index++;
            }
        }
    }

    row_index = 0;
    for (int col = 0; col < sky_image.cols; ++col) {
        for (int row = 0; row < sky_image.rows; ++row) {
            if (sky_image.at<cv::Vec3b>(row, col)[0] == 0 &&
                    sky_image.at<cv::Vec3b>(row, col)[1] == 0 &&
                    sky_image.at<cv::Vec3b>(row, col)[2] == 0) {
                continue;
            } else {
                cv::Vec3b intensity = sky_image.at<cv::Vec3b>(row, col);
                sky_image_non_zero.at<uchar>(row_index, 0) = intensity[0];
                sky_image_non_zero.at<uchar>(row_index, 1) = intensity[1];
                sky_image_non_zero.at<uchar>(row_index, 2) = intensity[2];
                row_index++;
            }
        }
    }
    */

    static std::vector<cv::Vec3b> sky_pixels(image_width*image_height);
    static std::vector<cv::Vec3b> ground_pixels(image_width*image_height);

    int sky_non_zeros_nums = 0;
    int ground_non_zeros_nums = 0;

    for (int row = 0; row < image_height; ++row) {
        const cv::Vec3b *ptr_src = src_image.ptr<cv::Vec3b>(row);

        for (int col = 0; col < image_width; ++col) {

            const cv::Vec3b &p = ptr_src[col];
            if (p[0] == 0 && p[1] == 0 && p[2] == 0)
                continue;

            if (row <= border[col]) {
                sky_pixels[sky_non_zeros_nums] = p;
                ++sky_non_zeros_nums;
            }
            else {
                ground_pixels[ground_non_zeros_nums] = p;
                ++ground_non_zeros_nums;
            }
        }
    }

    static cv::Mat sky_image_non_zero, ground_image_non_zero;
    sky_image_non_zero.create(sky_non_zeros_nums, 3, CV_8UC1);
    ground_image_non_zero.create(ground_non_zeros_nums, 3, CV_8UC1);

    #pragma omp parallel for
    for (int i=0; i<sky_non_zeros_nums; ++i)
    {
        const cv::Vec3b &p = sky_pixels[i];
        uchar *ptr_src = sky_image_non_zero.ptr<uchar>(i);
        ptr_src[0] = p[0];
        ptr_src[1] = p[1];
        ptr_src[2] = p[2];
    }

    #pragma omp parallel for
    for (int i=0; i<ground_non_zeros_nums; ++i)
    {
        const cv::Vec3b &p = ground_pixels[i];
        uchar *ptr_src = ground_image_non_zero.ptr<uchar>(i);
        ptr_src[0] = p[0];
        ptr_src[1] = p[1];
        ptr_src[2] = p[2];
    }

    // k-means clustering adjusts sky region boundaries
    cv::Mat sky_image_float;
    sky_image_non_zero.convertTo(sky_image_float, CV_32FC1);
    cv::Mat labels;

    /*cv::kmeans(sky_image_float, 2, labels,
               cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
               10, cv::KMEANS_RANDOM_CENTERS);*/

    cv::kmeans(sky_image_float, 2, labels,
                cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 10, 1.0),
                10, cv::KMEANS_RANDOM_CENTERS);

    int label_1_nums = cv::countNonZero(labels);
    int label_0_nums = labels.rows - label_1_nums;

    cv::Mat sky_label_1_image = cv::Mat::zeros(label_1_nums, 3, CV_8UC1);
    cv::Mat sky_label_0_image = cv::Mat::zeros(label_0_nums, 3, CV_8UC1);

    int row_index = 0;
    for (int row = 0; row < labels.rows; ++row) {
        if (labels.at<float>(row, 0) == 0.0) {
            sky_label_0_image.at<uchar>(row_index, 0) = sky_image_non_zero.at<uchar>(row, 0);
            sky_label_0_image.at<uchar>(row_index, 1) = sky_image_non_zero.at<uchar>(row, 1);
            sky_label_0_image.at<uchar>(row_index, 2) = sky_image_non_zero.at<uchar>(row, 2);
            row_index++;
        }
    }
    row_index = 0;
    for (int row = 0; row < labels.rows; ++row) {
        if (labels.at<float>(row, 0) == 1.0) {
            sky_label_1_image.at<uchar>(row_index, 0) = sky_image_non_zero.at<uchar>(row, 0);
            sky_label_1_image.at<uchar>(row_index, 1) = sky_image_non_zero.at<uchar>(row, 1);
            sky_label_1_image.at<uchar>(row_index, 2) = sky_image_non_zero.at<uchar>(row, 2);
            row_index++;
        }
    }

    cv::Mat sky_covar_1;
    cv::Mat sky_mean_1;
    cv::calcCovarMatrix(sky_label_1_image, sky_covar_1,
                        sky_mean_1, cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::Mat ic_s1;
    cv::invert(sky_covar_1, ic_s1, cv::DECOMP_SVD);

    cv::Mat sky_covar_0;
    cv::Mat sky_mean_0;
    cv::calcCovarMatrix(sky_label_0_image, sky_covar_0,
                        sky_mean_0, cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::Mat ic_s0;
    cv::invert(sky_covar_0, ic_s0, cv::DECOMP_SVD);

    cv::Mat ground_covar;
    cv::Mat ground_mean;
    cv::calcCovarMatrix(ground_image_non_zero, ground_covar,
                        ground_mean, cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::Mat ic_g;
    cv::invert(ground_covar, ic_g, cv::DECOMP_SVD);

    cv::Mat sky_mean;
    cv::Mat sky_covar;
    cv::Mat ic_s;
    if (cv::Mahalanobis(sky_mean_0, ground_mean, ic_s0) > cv::Mahalanobis(sky_mean_1, ground_mean, ic_s1)) {
        sky_mean = sky_mean_0;
        sky_covar = sky_covar_0;
        ic_s = ic_s0;
    } else {
        sky_mean = sky_mean_1;
        sky_covar = sky_covar_1;
        ic_s = ic_s1;
    }

    std::vector<int> border_new(border.size(), 0);
    for (size_t col = 0; col < border.size(); ++col) {
        double cnt = 0.0;
        for (int row = 0; row < border[col]; ++row) {
            // Calculate the Mahalanobis distance of each pixel in the original sky area and each point in the corrected sky area
            cv::Mat ori_pix;
            src_image.row(row).col(static_cast<int>(col)).convertTo(ori_pix, sky_mean.type());
            ori_pix = ori_pix.reshape(1, 1);
            double distance_s = cv::Mahalanobis(ori_pix,
                                                sky_mean, ic_s);
            double distance_g = cv::Mahalanobis(ori_pix,
                                                ground_mean, ic_g);

            if (distance_s < distance_g) {
                cnt++;
            }
        }
        if (cnt < (border[col] / 2)) {
            border_new[col] = 0;
        } else {
            border_new[col] = border[col];
        }
    }

    return border_new;
}

/***
 * Calculate the sky image energy function
 * @param border
 * @param src_image
 * @return
 */
double SkyAreaDetector::calculate_sky_energy(const std::vector<int> &border,
        const cv::Mat &src_image) {

    int image_height = src_image.size[0];
    int image_width = src_image.size[1];

/*
    // Make a sky image mask and a ground image mask
    cv::Mat sky_mask = make_sky_mask(src_image, border, 1);
    cv::Mat ground_mask = make_sky_mask(src_image, border, 0);

    // Deduction of sky images and ground images
    cv::Mat sky_image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    cv::Mat ground_image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    src_image.copyTo(sky_image, sky_mask);
    src_image.copyTo(ground_image, ground_mask);

    // Compute sky and ground image covariance matrices
    int ground_non_zeros_nums = cv::countNonZero(ground_mask);
    int sky_non_zeros_nums = cv::countNonZero(sky_mask);

    if (ground_non_zeros_nums == 0 || sky_non_zeros_nums == 0) {
        return std::numeric_limits<double>::min();
    }

    cv::Mat ground_image_non_zero = cv::Mat::zeros(ground_non_zeros_nums, 3, CV_8UC1);
    cv::Mat sky_image_non_zero = cv::Mat::zeros(sky_non_zeros_nums, 3, CV_8UC1);

    assert (ground_image.cols == image_width && ground_image.rows == image_height);
    assert (ground_image.cols == sky_image.cols && ground_image.rows == sky_image.rows);

    int row_index = 0;
    int row_index_beta = 0;
    for (int col = 0; col < image_width; ++col) {
        for (int row = 0; row < image_height; ++row) {
            if (ground_image.at<cv::Vec3b>(row, col)[0] != 0 ||
                    ground_image.at<cv::Vec3b>(row, col)[1] != 0 ||
                    ground_image.at<cv::Vec3b>(row, col)[2] != 0) {

                cv::Vec3b intensity = ground_image.at<cv::Vec3b>(row, col);
                ground_image_non_zero.at<uchar>(row_index, 0) = intensity[0];
                ground_image_non_zero.at<uchar>(row_index, 1) = intensity[1];
                ground_image_non_zero.at<uchar>(row_index, 2) = intensity[2];
                row_index++;
            }

            if (sky_image.at<cv::Vec3b>(row, col)[0] != 0 ||
                    sky_image.at<cv::Vec3b>(row, col)[1] != 0 ||
                    sky_image.at<cv::Vec3b>(row, col)[2] != 0) {

                cv::Vec3b intensity = sky_image.at<cv::Vec3b>(row, col);
                sky_image_non_zero.at<uchar>(row_index_beta, 0) = intensity[0];
                sky_image_non_zero.at<uchar>(row_index_beta, 1) = intensity[1];
                sky_image_non_zero.at<uchar>(row_index_beta, 2) = intensity[2];
                row_index_beta++;
            }
        }
    }
    */

    static std::vector<cv::Vec3b> sky_pixels(image_width*image_height);
    static std::vector<cv::Vec3b> ground_pixels(image_width*image_height);

    int sky_non_zeros_nums = 0;
    int ground_non_zeros_nums = 0;

    for (int row = 0; row < image_height; ++row) {
        const cv::Vec3b *ptr_src = src_image.ptr<cv::Vec3b>(row);

        for (int col = 0; col < image_width; ++col) {

            const cv::Vec3b &p = ptr_src[col];
            if (p[0] == 0 && p[1] == 0 && p[2] == 0)
                continue;

            if (row < border[col]) {
                sky_pixels[sky_non_zeros_nums] = p;
                ++sky_non_zeros_nums;
            }
            else {
                ground_pixels[ground_non_zeros_nums] = p;
                ++ground_non_zeros_nums;
            }
        }
    }

    if (ground_non_zeros_nums == 0 || sky_non_zeros_nums == 0) {
        return std::numeric_limits<double>::min();
    }

    static cv::Mat sky_image_non_zero, ground_image_non_zero;
    sky_image_non_zero.create(sky_non_zeros_nums, 3, CV_8UC1);
    ground_image_non_zero.create(ground_non_zeros_nums, 3, CV_8UC1);

    #pragma omp parallel for
    for (int i=0; i<sky_non_zeros_nums; ++i)
    {
        const cv::Vec3b &p = sky_pixels[i];
        uchar *ptr_src = sky_image_non_zero.ptr<uchar>(i);
        ptr_src[0] = p[0];
        ptr_src[1] = p[1];
        ptr_src[2] = p[2];
    }

    #pragma omp parallel for
    for (int i=0; i<ground_non_zeros_nums; ++i)
    {
        const cv::Vec3b &p = ground_pixels[i];
        uchar *ptr_src = ground_image_non_zero.ptr<uchar>(i);
        ptr_src[0] = p[0];
        ptr_src[1] = p[1];
        ptr_src[2] = p[2];
    }

    static cv::Mat ground_covar;
    static cv::Mat ground_mean;
    static cv::Mat ground_eig_vec;
    static cv::Mat ground_eig_val;

    cv::calcCovarMatrix(ground_image_non_zero, ground_covar,
                        ground_mean, cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::eigen(ground_covar, ground_eig_val, ground_eig_vec);

    static cv::Mat sky_covar;
    static cv::Mat sky_mean;
    static cv::Mat sky_eig_vec;
    static cv::Mat sky_eig_val;

    cv::calcCovarMatrix(sky_image_non_zero, sky_covar,
                        sky_mean, cv::COVAR_ROWS | cv::COVAR_SCALE | cv::COVAR_NORMAL);
    cv::eigen(sky_covar, sky_eig_val, sky_eig_vec);

    int para = 2; // 论文原始参数
    double ground_det = fabs(cv::determinant(ground_covar));
    double sky_det = fabs(cv::determinant(sky_covar));
    double ground_eig_det = fabs(ground_eig_val.at<double>(0,0));
    double sky_eig_det = fabs(sky_eig_val.at<double>(0.0));

//    printf("%lf, %lf, %lf, %lf\n", ground_det, sky_det, ground_eig_det, sky_eig_det);

    return 1 / ((para * sky_det + ground_det) + (para * sky_eig_det + ground_eig_det));

}

/***
 * Determine if an image contains a sky area
 * @param border
 * @param thresh_1
 * @param thresh_2
 * @param thresh_3
 * @return
 */
bool SkyAreaDetector::has_sky_region(const std::vector<int> &border,
                                     std::vector<int> &border_diff,
                                     double thresh_1, double thresh_2,
                                     double thresh_3) {
    double border_mean = 0.0;
    for (size_t i = 0; i < border.size(); ++i) {
        border_mean += border[i];
    }
    border_mean /= border.size();

    // If the average skyline is too small to consider no sky area
    if (border_mean < thresh_1) {
        printf("border mean too height\n");
        return false;
    }

    assert (border.size() - 1 == border_diff.size());
    for (auto i = static_cast<int>(border.size() - 1); i >= 0; --i) {
        border_diff[i] = std::abs(border[i + 1] - border[i]);
    }

    double border_diff_mean = 0.0;
    for (auto &diff_val : border_diff) {
        border_diff_mean += diff_val;
    }
    border_diff_mean /= border_diff.size();

//    printf("%lf, %lf; thresh1 %lf, thresh2 %lf, thresh3 %lf\n", border_mean, border_diff_mean, thresh_1, thresh_2, thresh_3);

    return !(border_mean < thresh_1 || (border_diff_mean > thresh_3 && border_mean < thresh_2));
}

/***
 * Determine whether there is a part of the image that is a sky area
 * @param border
 * @param thresh_1
 * @return
 */
bool SkyAreaDetector::has_partial_sky_region(const std::vector<int> &border,
                                             const std::vector<int> &border_diff,
                                             double thresh_4) {
    assert (border.size() - 1 == border_diff.size());

    for (size_t i = 0; i < border_diff.size(); ++i) {
        if (border_diff[i] > thresh_4) {
            return true;
        }
    }

    return false;
}

/***
 * Fusion map of sky area and original image
 * @param src_image
 * @param border
 * @param sky_image
 */
void SkyAreaDetector::display_sky_region(const cv::Mat &src_image,
        const std::vector<int> &border,
        cv::Mat &sky_image) {

    int image_height = src_image.size[0];
    int image_width = src_image.size[1];
    // Make a sky map mask
    cv::Mat sky_mask = make_sky_mask(src_image, border, 1);

    // Blend of sky and original image
    cv::Mat sky_image_full = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    sky_image_full.setTo(cv::Scalar(0, 0, 255), sky_mask);
    cv::addWeighted(src_image, 1, sky_image_full, 1, 0, sky_image);
}

/***
 * Make a sky mask image
 * @param src_image
 * @param border
 * @param type: 1: 天空 0: 地面
 * @return
 */
cv::Mat SkyAreaDetector::make_sky_mask(const cv::Mat &src_image,
                                       const std::vector<int> &border,
                                       int type) {
    int image_height = src_image.size[0];
    int image_width = src_image.size[1];

    cv::Mat mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);

    if (type == 1) {
        for (int row = 0; row < image_height; ++row) {
            uchar *p = mask.ptr<uchar>(row);
            for (int col = 0; col < image_width; ++col) {
                if (row <= border[col]) {
                    p[col] = 255;
                }
            }
        }
    } else if (type == 0) {
        for (int row = 0; row < image_height; ++row) {
            uchar *p = mask.ptr<uchar>(row);
            for (int col = 0; col < image_width; ++col) {
                if (row > border[col]) {
                    p[col] = 255;
                }
            }
        }
    } else {
        assert(type == 0 || type == 1);
    }

    return mask;
}

}
