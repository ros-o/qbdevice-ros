#include <qb_device_hardware_interface/qb_device_hardware_interface.h>

using namespace qb_device_hardware_interface;

qbDeviceHW::qbDeviceHW() :
    private_node_handler_(ros::NodeHandle("~")) {

}

qbDeviceHW::~qbDeviceHW() {

}

void qbDeviceHW::read(const ros::Time& time, const ros::Duration& period) {
  //TODO: read actuator state from hardware...
}

void qbDeviceHW::write(const ros::Time& time, const ros::Duration& period) {
  //TODO: send actuator command to hardware...
}