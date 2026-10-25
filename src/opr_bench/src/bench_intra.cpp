// Publisher and subscriber in one process with intra-process communication: the baseline for
// what composing nodes into one container buys.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <class_loader/class_loader.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  class_loader::ClassLoader loader("libopr_bench_nodes.so");
  auto make = [&](const char* cls, std::vector<rclcpp::Parameter> params) {
    auto f = loader.createInstance<rclcpp_components::NodeFactory>(cls);
    rclcpp::NodeOptions o;
    o.use_intra_process_comms(true);
    o.parameter_overrides(params);
    o.arguments({"--ros-args"});
    return f->create_node_instance(o);
  };
  const std::string out = argc > 1 ? argv[1] : "";
  const int width = argc > 2 ? std::atoi(argv[2]) : 1920;
  const int height = argc > 3 ? std::atoi(argv[3]) : 1080;
  const int bpp = argc > 4 ? std::atoi(argv[4]) : 2;
  const int frames = argc > 5 ? std::atoi(argv[5]) : 300;
  auto sub = make("rclcpp_components::NodeFactoryTemplate<opr::bench::BenchSubscriber>",
                  {{"mode", "raw"}, {"output", out + ".sub.json"}});
  auto pub = make("rclcpp_components::NodeFactoryTemplate<opr::bench::BenchPublisher>",
                  {{"width", width}, {"height", height}, {"bytes_per_pixel", bpp}, {"frames", frames},
                   {"output", out + ".pub.json"}});
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(sub.get_node_base_interface());
  ex.add_node(pub.get_node_base_interface());
  ex.spin();
  return 0;
}
