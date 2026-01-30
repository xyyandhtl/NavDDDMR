#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <string>
#include <vector>

using livox_ros_driver2::msg::CustomMsg;

class BagConverter : public rclcpp::Node
{
public:
    BagConverter() : Node("bag_converter")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("input_bag", "");
        this->declare_parameter<std::string>("output_bag", "");
        this->declare_parameter<std::vector<std::string>>("livox_topics",
            std::vector<std::string>{"/livox/avia/lidar", "/livox/mid360/lidar"});
        this->declare_parameter<std::string>("output_frame_override", "");
        this->declare_parameter<bool>("verbose", true);

        // Get parameters
        input_bag_ = this->get_parameter("input_bag").as_string();
        output_bag_ = this->get_parameter("output_bag").as_string();
        livox_topics_ = this->get_parameter("livox_topics").as_string_array();
        output_frame_override_ = this->get_parameter("output_frame_override").as_string();
        verbose_ = this->get_parameter("verbose").as_bool();

        if (input_bag_.empty() || output_bag_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Input bag and output bag parameters are required!");
            throw std::runtime_error("Missing required parameters");
        }

        RCLCPP_INFO(this->get_logger(), "Input bag: %s", input_bag_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output bag: %s", output_bag_.c_str());

        convert_bag();
    }

private:
    void convert_bag()
    {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = input_bag_;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::Reader reader;
        reader.open(storage_options);

        rosbag2_cpp::Writer writer;
        rosbag2_storage::StorageOptions out_storage;
        out_storage.uri = output_bag_;
        out_storage.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        writer.open(out_storage, converter_options);

        // --- topic metadata ---
        auto topics = reader.get_all_topics_and_types();
        std::unordered_map<std::string, std::string> topic_type_map;
        std::unordered_set<std::string> livox_topic_set(livox_topics_.begin(), livox_topics_.end());

        for (const auto & t : topics) {
            topic_type_map[t.name] = t.type;

            // Check if this is a livox topic that needs type conversion
            if (livox_topic_set.count(t.name) && t.type == "livox_ros_driver2/msg/CustomMsg") {
                // Create a new topic info with the converted type
                rosbag2_storage::TopicMetadata converted_topic = t;
                converted_topic.type = "sensor_msgs/msg/PointCloud2";
                converted_topic.serialization_format = "cdr";  // Or whatever format is appropriate
                writer.create_topic(converted_topic);
            } else {
                // Create topic with original metadata
                writer.create_topic(t);
            }
        }

        rclcpp::Serialization<CustomMsg> livox_serializer;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;

        while (reader.has_next())
        {
            auto bag_msg = reader.read_next();
            const std::string & topic = bag_msg->topic_name;
            const std::string & type  = topic_type_map[topic];

            rclcpp::Time stamp(bag_msg->time_stamp);

            // Check if this is a livox topic that needs conversion
            bool is_livox_topic = false;
            for (const auto& livox_topic : livox_topics_) {
                if (topic == livox_topic && type == "livox_ros_driver2/msg/CustomMsg") {
                    is_livox_topic = true;
                    break;
                }
            }

            if (is_livox_topic)
            {
                // Deserialize
                CustomMsg livox_msg;
                rclcpp::SerializedMessage serialized_in(*bag_msg->serialized_data);
                livox_serializer.deserialize_message(&serialized_in, &livox_msg);

                // Convert to PointCloud2
                auto cloud = livoxCustomToPointCloud2(livox_msg);

                // Apply frame override if specified
                if (!output_frame_override_.empty()) {
                    cloud.header.frame_id = output_frame_override_;
                }

                // Serialize as shared_ptr
                auto out_serialized = std::make_shared<rclcpp::SerializedMessage>();
                pc2_serializer.serialize_message(&cloud, out_serialized.get());

                // Write
                writer.write(
                    out_serialized,
                    topic,
                    "sensor_msgs/msg/PointCloud2",
                    stamp
                );

                if (verbose_) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Converted message from %s to PointCloud2", topic.c_str());
                }
            }
            else
            {
                // Passthrough unknown topics
                auto passthrough = std::make_shared<rclcpp::SerializedMessage>(*bag_msg->serialized_data);

                writer.write(
                    passthrough,
                    topic,
                    type,
                    stamp
                );
            }
        }

        RCLCPP_INFO(this->get_logger(), "Bag conversion completed successfully!");
    }

    static sensor_msgs::msg::PointCloud2
    livoxCustomToPointCloud2(const CustomMsg & msg)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg.header;

        cloud.height = 1;
        cloud.width = msg.point_num;
        cloud.is_dense = false;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
        );
        modifier.resize(msg.point_num);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");

        for (uint32_t i = 0; i < msg.point_num; ++i)
        {
            const auto & p = msg.points[i];
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            *iter_i = static_cast<float>(p.reflectivity);

            ++iter_x; ++iter_y; ++iter_z; ++iter_i;
        }

        return cloud;
    }

    std::string input_bag_;
    std::string output_bag_;
    std::vector<std::string> livox_topics_;
    std::string output_frame_override_;
    bool verbose_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto converter = std::make_shared<BagConverter>();
        rclcpp::spin(converter);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("bag_converter"), "Error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
