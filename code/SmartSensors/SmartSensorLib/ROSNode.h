#ifndef ROSNode_h
#define ROSNode_h
// ROS2 Arduino
class ROSPub : public ros2::Node
{
  public:
    ROSPub(SmartSensor* s) : Node()
    {
      ros2::Publisher<std_msgs::Float32>* publisher_ = this->createPublisher<std_msgs::Float32>(s->getTopicName());
      this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishTemp, NULL, publisher_);
    }
};


void publishTemp(std_msgs::Float32* msg, void* arg) {
  (void)(arg);

  static int cnt = 0;
  msg->data = s.getT

}
#endif // ROSNode_h
