/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>


#include "calc/ResultTrajectory.h"
#include "utils/Loader.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif


int main(int argc, char **argv) {

    // Create ros node
    ros::init(argc, argv, "error_dataset");

    // Ensure we have a path
    if(argc < 4) {
        ROS_ERROR("ERROR: Please specify a file to convert");
        ROS_ERROR("ERROR: rosrun ov_eval error_dataset <align_mode> <file_gt.txt> <folder_algorithms>");
        std::exit(EXIT_FAILURE);
    }

    // Load it!
    boost::filesystem::path path_gt(argv[2]);
    std::vector<double> times;
    std::vector<Eigen::Matrix<double,7,1>> poses;
    std::vector<Eigen::Matrix3d> cov_ori, cov_pos;
    ov_eval::Loader::load_data(argv[2], times, poses, cov_ori, cov_pos);
    // Print its length and stats
    double length = ov_eval::Loader::get_total_length(poses);
    ROS_INFO("[COMP]: %d poses in %s => length of %.2f meters",(int)times.size(),path_gt.stem().string().c_str(),length);


    // Get the algorithms we will process
    // Also create empty statistic objects for each of our datasets
    std::string path_algos(argv[3]);
    std::vector<boost::filesystem::path> path_algorithms;
    for(const auto& entry : boost::filesystem::directory_iterator(path_algos)) {
        if(boost::filesystem::is_directory(entry)) {
            path_algorithms.push_back(entry.path());

        }
    }
    std::sort(path_algorithms.begin(), path_algorithms.end());


    //===============================================================================
    //===============================================================================
    //===============================================================================


    // Relative pose error segment lengths
    //std::vector<double> segments = {8.0, 16.0, 24.0, 32.0, 40.0};
    std::vector<double> segments = {7.0, 14.0, 21.0, 28.0, 35.0};


    //===============================================================================
    //===============================================================================
    //===============================================================================


    // Loop through each algorithm type
    for(size_t i=0; i<path_algorithms.size(); i++) {

        // Debug print
        ROS_INFO("======================================");
        ROS_INFO("[COMP]: processing %s algorithm", path_algorithms.at(i).stem().c_str());

        // Get the list of datasets this algorithm records
        std::map<std::string,boost::filesystem::path> path_algo_datasets;
        for(auto& entry : boost::filesystem::directory_iterator(path_algorithms.at(i))) {
            if(boost::filesystem::is_directory(entry)) {
                path_algo_datasets.insert({entry.path().stem().string(),entry.path()});
            }
        }

        // Check if we have runs for our dataset
        if(path_algo_datasets.find(path_gt.stem().string())==path_algo_datasets.end()) {
            ROS_ERROR("[COMP]: %s dataset does not have any runs for %s!!!!!",path_algorithms.at(i).stem().c_str(),path_gt.stem().c_str());
            continue;
        }

        // Errors for this specific dataset (i.e. our averages over the total runs)
        ov_eval::Statistics ate_dataset_ori, ate_dataset_pos;
        std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> rpe_dataset;
        for(const auto& len : segments) {
            rpe_dataset.insert({len,{ov_eval::Statistics(),ov_eval::Statistics()}});
        }
        std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> rmse_dataset;
        std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> nees_dataset;

        // Loop though the different runs for this dataset
        size_t total_runs = 0;
        for(auto& entry : boost::filesystem::directory_iterator(path_algo_datasets.at(path_gt.stem().string()))) {
            if(entry.path().extension() == ".txt") {

                // Our paths
                std::string path_gttxt = path_gt.string();
                std::string path_esttxt = entry.path().string();

                // Create our trajectory object
                ov_eval::ResultTrajectory traj(path_esttxt, path_gttxt, argv[1]);

                // Calculate ATE error for this dataset
                ov_eval::Statistics error_ori, error_pos;
                traj.calculate_ate(error_ori, error_pos);
                ate_dataset_ori.values.push_back(error_ori.rmse);
                ate_dataset_pos.values.push_back(error_pos.rmse);
                for(size_t j=0; j<error_ori.values.size(); j++) {
                    rmse_dataset[error_ori.timestamps.at(j)].first.values.push_back(error_ori.values.at(j));
                    rmse_dataset[error_pos.timestamps.at(j)].second.values.push_back(error_pos.values.at(j));
                    assert(error_ori.timestamps.at(j)==error_pos.timestamps.at(j));
                }

                // NEES error for this dataset
                ov_eval::Statistics nees_ori, nees_pos;
                traj.calculate_nees(nees_ori, nees_pos);
                for(size_t j=0; j<nees_ori.values.size(); j++) {
                    nees_dataset[nees_ori.timestamps.at(j)].first.values.push_back(nees_ori.values.at(j));
                    nees_dataset[nees_ori.timestamps.at(j)].second.values.push_back(nees_pos.values.at(j));
                    assert(nees_ori.timestamps.at(j)==nees_pos.timestamps.at(j));
                }


                // Calculate RPE error for this dataset
                std::map<double,std::pair<ov_eval::Statistics,ov_eval::Statistics>> error_rpe;
                traj.calculate_rpe(segments, error_rpe);
                for(const auto& elm : error_rpe) {
                    rpe_dataset.at(elm.first).first.values.insert(rpe_dataset.at(elm.first).first.values.end(),elm.second.first.values.begin(),elm.second.first.values.end());
                    rpe_dataset.at(elm.first).first.timestamps.insert(rpe_dataset.at(elm.first).first.timestamps.end(),elm.second.first.timestamps.begin(),elm.second.first.timestamps.end());
                    rpe_dataset.at(elm.first).second.values.insert(rpe_dataset.at(elm.first).second.values.end(),elm.second.second.values.begin(),elm.second.second.values.end());
                    rpe_dataset.at(elm.first).second.timestamps.insert(rpe_dataset.at(elm.first).second.timestamps.end(),elm.second.second.timestamps.begin(),elm.second.second.timestamps.end());
                }

                // Count number of runs
                total_runs++;

            }
        }

        // Check if we have runs
        if(total_runs < 1) {
            ROS_ERROR("\tERROR: No runs found for %s, is the folder structure right??", path_algorithms.at(i).stem().c_str());
            continue;
        }

        // Compute our mean ATE score
        ate_dataset_ori.calculate();
        ate_dataset_pos.calculate();

        // Print stats for this specific dataset
        ROS_INFO("\tATE: mean_ori = %.3f | mean_pos = %.3f",ate_dataset_ori.mean,ate_dataset_pos.mean);
        ROS_INFO("\tATE: std_ori  = %.5f | std_pos  = %.5f",ate_dataset_ori.std,ate_dataset_pos.std);
        for(auto &seg : rpe_dataset) {
            seg.second.first.calculate();
            seg.second.second.calculate();
            ROS_INFO("\tRPE: seg %d - mean_ori = %.3f | mean_pos = %.3f (%d samples)",(int)seg.first,seg.second.first.mean,seg.second.second.mean,(int)seg.second.second.values.size());
            //ROS_INFO("RPE: seg %d - std_ori  = %.3f | std_pos  = %.3f",(int)seg.first,seg.second.first.std,seg.second.second.std);
        }

        // RMSE: Convert into the right format (only use times where all runs have an error)
        ov_eval::Statistics rmse_ori, rmse_pos;
        for(auto &elm : rmse_dataset) {
            if(elm.second.first.values.size()==total_runs) {
                elm.second.first.calculate();
                elm.second.second.calculate();
                rmse_ori.timestamps.push_back(elm.first);
                rmse_ori.values.push_back(elm.second.first.rmse);
                rmse_pos.timestamps.push_back(elm.first);
                rmse_pos.values.push_back(elm.second.second.rmse);
            }
        }
        rmse_ori.calculate();
        rmse_pos.calculate();
        ROS_INFO("\tRMSE: mean_ori = %.3f | mean_pos = %.3f",rmse_ori.mean,rmse_pos.mean);

        // NEES: Convert into the right format (only use times where all runs have an error)
        ov_eval::Statistics nees_ori, nees_pos;
        for(auto &elm : nees_dataset) {
            if(elm.second.first.values.size()==total_runs) {
                elm.second.first.calculate();
                elm.second.second.calculate();
                nees_ori.timestamps.push_back(elm.first);
                nees_ori.values.push_back(elm.second.first.mean);
                nees_pos.timestamps.push_back(elm.first);
                nees_pos.values.push_back(elm.second.second.mean);
            }
        }
        nees_ori.calculate();
        nees_pos.calculate();
        ROS_INFO("\tNEES: mean_ori = %.3f | mean_pos = %.3f",nees_ori.mean,nees_pos.mean);


#ifdef HAVE_PYTHONLIBS

        //=====================================================
        // RMSE plot at each timestep
        matplotlibcpp::figure_size(1000, 600);

        // Zero our time arrays
        double starttime1 = (rmse_ori.timestamps.empty())? 0 : rmse_ori.timestamps.at(0);
        double endtime1 = (rmse_ori.timestamps.empty())? 0 : rmse_ori.timestamps.at(rmse_ori.timestamps.size()-1);
        for(size_t j=0; j<rmse_ori.timestamps.size(); j++) {
            rmse_ori.timestamps.at(j) -= starttime1;
            rmse_pos.timestamps.at(j) -= starttime1;
        }

        // Update the title and axis labels
        matplotlibcpp::subplot(2,1,1);
        matplotlibcpp::title("Root Mean Squared Error - "+path_algorithms.at(i).stem().string());
        matplotlibcpp::ylabel("Error Orientation (deg)");
        matplotlibcpp::plot(rmse_ori.timestamps, rmse_ori.values);
        matplotlibcpp::xlim(0.0,endtime1-starttime1);
        matplotlibcpp::subplot(2,1,2);
        matplotlibcpp::ylabel("Error Position (m)");
        matplotlibcpp::xlabel("dataset time (s)");
        matplotlibcpp::plot(rmse_pos.timestamps, rmse_pos.values);
        matplotlibcpp::xlim(0.0,endtime1-starttime1);

        // Display to the user
        matplotlibcpp::tight_layout();
        matplotlibcpp::show(false);

        //=====================================================

        if(!nees_ori.values.empty() && !nees_pos.values.empty()) {
            // NEES plot at each timestep
            matplotlibcpp::figure_size(1000, 600);

            // Zero our time arrays
            double starttime2 = (nees_ori.timestamps.empty())? 0 : nees_ori.timestamps.at(0);
            double endtime2 = (nees_ori.timestamps.empty())? 0 : nees_ori.timestamps.at(nees_ori.timestamps.size()-1);
            for(size_t j=0; j<nees_ori.timestamps.size(); j++) {
                nees_ori.timestamps.at(j) -= starttime2;
                nees_pos.timestamps.at(j) -= starttime2;
            }

            // Update the title and axis labels
            matplotlibcpp::subplot(2,1,1);
            matplotlibcpp::title("Normalized Estimation Error Squared - "+path_algorithms.at(i).stem().string());
            matplotlibcpp::ylabel("NEES Orientation");
            matplotlibcpp::plot(nees_ori.timestamps, nees_ori.values);
            matplotlibcpp::xlim(0.0,endtime2-starttime2);
            matplotlibcpp::subplot(2,1,2);
            matplotlibcpp::ylabel("NEES Position");
            matplotlibcpp::xlabel("dataset time (s)");
            matplotlibcpp::plot(nees_pos.timestamps, nees_pos.values);
            matplotlibcpp::xlim(0.0,endtime2-starttime2);

            // Display to the user
            matplotlibcpp::tight_layout();
            matplotlibcpp::show(false);
        }

#endif


    }

    // Final line for our printed stats
    ROS_INFO("============================================");


#ifdef HAVE_PYTHONLIBS

    // Wait till the user kills this node
    matplotlibcpp::show(true);
    //ros::spin();

#endif

    // Done!
    return EXIT_SUCCESS;

}


