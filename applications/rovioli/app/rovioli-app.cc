#include <memory>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <ros/ros.h>
#include <sensors/imu.h>
#include <sensors/sensor-factory.h>
#include <signal.h>
#include <vi-map/vi-map-serialization.h>

#include "rovioli/rovioli-node.h"

DEFINE_string(
        vio_localization_map_folder, "",
        "Path to a localization summary map or a full VI-map used for "
                "localization.");
DEFINE_string(
        ncamera_calibration, "ncamera.yaml",
        "Path to the camera calibration yaml.");
// TODO(schneith): Unify these two noise definitions.
/*
 * DEFINE_string(
        imu_parameters_rovio, "imu-rovio.yaml",
        "Path to the imu configuration yaml "
                "for ROVIO.");
DEFINE_string(
        imu_parameters_maplab, "imu-maplab.yaml",
        "Path to the imu configuration yaml for MAPLAB.");
        */
DEFINE_string(
        save_map_folder, "", "Save map to folder; if empty nothing is saved.");
DEFINE_bool(
        overwrite_existing_map, false,
        "If set to true, an existing map will be overwritten on save. Otherwise, a "
                "number will be appended to save_map_folder to obtain an available "
                "folder.");
DEFINE_bool(
        optimize_map_to_localization_map, false,
        "Optimize and process the map into a localization map before "
                "saving it.");

DECLARE_bool(map_builder_save_image_as_resources);

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);//Looks for flags in argv and parses them.  Rearranges argv to put
  // flags first, or removes them entirely if remove_flags is true.
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "rovioli");
  ros::NodeHandle nh;

  // Optionally load localization map.
  std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map;
  if (!FLAGS_vio_localization_map_folder.empty()) {
    localization_map.reset(new summary_map::LocalizationSummaryMap);
    if (!localization_map->loadFromFolder(FLAGS_vio_localization_map_folder)) {
      LOG(WARNING) << "Could not load a localization summary map from "
                   << FLAGS_vio_localization_map_folder
                   << ". Will try to load it as a full VI map.";//这一步在实际跑的时候是没有读进来的，是要读入到vi_map中去的
      vi_map::VIMap vi_map;
      CHECK(
              vi_map::serialization::loadMapFromFolder(
                      FLAGS_vio_localization_map_folder, &vi_map))//vi-map-serialization.cc
                                                                          << "Loading a VI map failed. Either provide a valid localization map "
                                                                          << "or leave the map folder flag empty.";

      localization_map.reset(new summary_map::LocalizationSummaryMap);
      summary_map::createLocalizationSummaryMapForWellConstrainedLandmarks(//在localization-summary-map-creation.cc文件中，将vi-map转为localization-map？
              vi_map, localization_map.get());
      // Make sure the localization map is not empty.
      CHECK_GT(localization_map->GLandmarkPosition().cols(), 0);
    }
  }

  // Load camera calibration and imu parameters.
  aslam::NCamera::Ptr camera_system =
          aslam::NCamera::loadFromYaml(FLAGS_ncamera_calibration);
  CHECK(camera_system) << "Could not load the camera calibration from: \'"
                       << FLAGS_ncamera_calibration << "\'";

  //vi_map::Imu::UniquePtr maplab_imu_sensor =
  //        vi_map::createFromYaml<vi_map::Imu>(FLAGS_imu_parameters_maplab);
 // CHECK(maplab_imu_sensor)
 // << "Could not load IMU parameters for MAPLAB from: \'"
 // << FLAGS_imu_parameters_maplab << "\'";
  //CHECK(maplab_imu_sensor->getImuSigmas().isValid());

  //vi_map::ImuSigmas rovio_imu_sigmas;
 // CHECK(rovio_imu_sigmas.loadFromYaml(FLAGS_imu_parameters_rovio))
 // << "Could not load IMU parameters for ROVIO from: \'"
 // << FLAGS_imu_parameters_rovio << "\'";
//  CHECK(rovio_imu_sigmas.isValid());

  // Construct the application.
  //对于一些只订阅一个话题的简单节点来说，我们使用ros::spin()进入接收循环，每当有订阅的话题发布时，进入回调函数接收和处理消息数据。
  // 但是更多的时候，一个节点往往要接收和处理不同来源的数据，并且这些数据的产生频率也各不相同，
  // 当我们在一个回调函数里耗费太多时间时，会导致其他回调函数被阻塞，导致数据丢失。这种场合需要给一个节点开辟多个线程，保证数据流的畅通。
  //AsyncSpinner比MultiThreadedSpinner更优，它有start() 和stop() 函数，并且在销毁的时候会自动停止。
  //给个如下的例子：
  //ros::AsyncSpinner spinner(4); // Use 4 threads
  //spinner.start();
  //ros::waitForShutdown();
  ros::AsyncSpinner ros_spinner(common::getNumHardwareThreads());
  std::unique_ptr<message_flow::MessageFlow> flow(
          message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
                  common::getNumHardwareThreads()));

  if (FLAGS_map_builder_save_image_as_resources &&
      FLAGS_save_map_folder.empty()) {
    LOG(FATAL) << "If you would like to save the resources, "
               << "please also set a map folder with: --save_map_folder";
  }

  // If a map will be saved (i.e., if the save map folder is not empty), append
  // a number to the name until a name is found that is free.
  std::string save_map_folder = FLAGS_save_map_folder;
  if (!FLAGS_save_map_folder.empty()) {
    size_t counter = 0u;
    while (common::fileExists(save_map_folder) ||
           (!FLAGS_overwrite_existing_map &&
            common::pathExists(save_map_folder))) {
      save_map_folder = FLAGS_save_map_folder + "_" + std::to_string(counter++);
    }
  }
  //RovioliNode的形参为相机内参，imu内参，保存地图路径，定位地图，数据流
 /* rovioli::RovioliNode rovio_localization_node(
          camera_system, std::move(maplab_imu_sensor), rovio_imu_sigmas,
          save_map_folder, localization_map.get(), flow.get());
*/
    rovioli::RovioliNode rovio_localization_node(
            camera_system, save_map_folder, localization_map.get(), flow.get());

  // Start the pipeline. The ROS spinner will handle SIGINT for us and abort
  // the application on CTRL+C.
  ros_spinner.start();
  rovio_localization_node.start();

  std::atomic<bool>& end_of_days_signal_received =
          rovio_localization_node.isDataSourceExhausted();
  while (ros::ok() && !end_of_days_signal_received.load()) {
    VLOG_EVERY_N(1, 10) << "\n" << flow->printDeliveryQueueStatistics();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  rovio_localization_node.shutdown();
  flow->shutdown();
  flow->waitUntilIdle();

  if (!save_map_folder.empty()) {
    rovio_localization_node.saveMapAndOptionallyOptimize(
            save_map_folder, FLAGS_overwrite_existing_map,
            FLAGS_optimize_map_to_localization_map);
  }
  return 0;
}
