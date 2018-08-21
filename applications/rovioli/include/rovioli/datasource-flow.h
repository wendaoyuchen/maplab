#ifndef ROVIOLI_DATASOURCE_FLOW_H_
#define ROVIOLI_DATASOURCE_FLOW_H_
#include <memory>
#include <iostream>

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/rostopic-settings.h>
#include <vio-common/vio-types.h>

#include "rovioli/datasource-factory.h"
#include "rovioli/flow-topics.h"

namespace rovioli {

class DataSourceFlow {
 public:
  explicit DataSourceFlow(
      const aslam::NCamera& camera_system, const vi_map::Imu& imu_sensor) {
    datasource_.reset(
            createAndConfigureDataSourcefromGFlags(camera_system, imu_sensor));
    CHECK(datasource_);
  }
/*只读入图像，不用imu数据（change:1）*/
  explicit DataSourceFlow(
          const aslam::NCamera& camera_system) {
    datasource_.reset(
            createAndConfigureDataSourcefromGFlags(camera_system));
    CHECK(datasource_);
  }

  ~DataSourceFlow() {
    shutdown();
  }

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    datasource_->registerImageCallback(
            flow->registerPublisher<message_flow_topics::IMAGE_MEASUREMENTS>());//函数原型：void register##SENSOR_NAME##Callback（datasource.h）
    datasource_->registerImuCallback(
            flow->registerPublisher<message_flow_topics::IMU_MEASUREMENTS>());
  }

  /*只读入图像，不用imu数据（change:5）*/
  void attachToMessageFlow(message_flow::MessageFlow* flow, bool Isimu) {
    CHECK_NOTNULL(flow);
    datasource_->registerImageCallback(
        flow->registerPublisher<message_flow_topics::IMAGE_MEASUREMENTS>());//函数原型：void register##SENSOR_NAME##Callback（datasource.h）
    if(Isimu){
        std::cerr << "datasource-flow.h :50 "<< "datasource accept IMU messages"<<std::endl;
        datasource_->registerImuCallback(
                flow->registerPublisher<message_flow_topics::IMU_MEASUREMENTS>());
    }

  }

  void startStreaming() {
    datasource_->startStreaming();
  }

  void shutdown() {
    datasource_->shutdown();
  }

  void registerEndOfDataCallback(const std::function<void()>& cb) {
    CHECK(cb);
    datasource_->registerEndOfDataCallback(cb);
  }

 private:
  std::unique_ptr<DataSource> datasource_;
};

}  // namespace rovioli

#endif  // ROVIOLI_DATASOURCE_FLOW_H_
