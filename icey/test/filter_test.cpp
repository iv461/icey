#include "node_fixture.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <unordered_set>
using namespace std::chrono_literals;
using ExampleService = std_srvs::srv::SetBool;

TEST_F(NodeTest, SyncWithReferenceTest) {

}

TEST_F(NodeTest, SyncApproxTimeTest) {

    auto images = node_->icey().create_stream<icey::Stream< sensor_msgs::msg::Image::SharedPtr >>();
    auto point_clouds = node_->icey().create_stream<icey::Stream< sensor_msgs::msg::PointCloud2::SharedPtr >>();
    
    size_t num_message_sets_received = 0;
    auto synched = node_->icey().synchronize_approx_time(images, point_clouds);
        synched.then([&](sensor_msgs::msg::Image::SharedPtr img, sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {
            num_message_sets_received++;
        });
    
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.stamp = icey::rclcpp_from_chrono(icey::Time(10000s));

    auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    point_cloud->header.stamp = icey::rclcpp_from_chrono(icey::Time(10001s));
    
    //spin(1s);
}

TEST_F(NodeTest, SynchronizeWithTransformTest) {
    
}

TEST_F(NodeTest, AnyTest) {
    auto s1 = node_->icey().create_timer(43ms).
        then([](size_t ticks) -> std::optional<std::string> {
            if(ticks >= 3) return {};
            return "s1_" + std::to_string(ticks+1);
        });

    auto s2 = node_->icey().create_timer(53ms)
        .then([](size_t ticks) -> std::optional<std::string> {
            if(ticks >= 3) return {};
            return "s2_" + std::to_string(ticks+1);
        });

    auto s3 = node_->icey().create_timer(67ms)
        .then([](size_t ticks) -> std::optional<std::string> {
            if(ticks >= 3) return {};
            return "s3_" + std::to_string(ticks+1);
        });
    
    std::unordered_multiset<std::string> received_values;
    icey::any(s1, s2, s3).
        then([&](std::string val) {
            received_values.emplace(val);
        });
    
    spin(250ms);
    std::unordered_multiset<std::string> target_values{"s1_1", "s1_2", "s1_3", "s2_1", "s2_2", "s2_3", 
                "s3_1", "s3_2", "s3_3"};

    EXPECT_EQ(received_values, target_values);
}

TEST_F(NodeTest, UnpackTest) {
   auto timer = node_->icey().create_timer(50ms);

   auto [double_stream, tf_stream2, string_stream] = 
        timer
        .then([&](size_t ticks) {
            auto val = std::make_tuple(double(ticks), geometry_msgs::msg::TransformStamped{}, std::string{});
            if(ticks >= 3) {
                timer.impl()->timer->cancel();
                return std::optional<decltype(val)>{};
            }
            return std::optional(val);
        }).unpack();
    
    size_t double_stream_num_called = 0;
    size_t tf_stream2_num_called = 0;
    size_t string_stream_num_called = 0;

    double_stream
        .then([&](double) { 
            double_stream_num_called++;
         });
    
    tf_stream2
        .then([&](geometry_msgs::msg::TransformStamped) { 
            tf_stream2_num_called++;
        });
    
    string_stream
        .then([&](std::string) {             
            string_stream_num_called++;
        });

    spin(250ms);

    EXPECT_EQ(double_stream_num_called, 3);
    EXPECT_EQ(tf_stream2_num_called, 3);
    EXPECT_EQ(string_stream_num_called, 3);
    
}

TEST_F(NodeTest, FilterTest) {
   auto timer = node_->icey().create_timer(50ms);
    
   size_t num_values_received = 0;
   timer
     .filter([&](size_t ticks) { return !(ticks % 3); })
     .then([&](size_t ticks) {
         if(ticks > 10) {
             timer.impl()->timer->cancel();
             return;
        }
        ASSERT_FALSE(ticks % 3);
        num_values_received++;
    });

    spin(1100ms);
    /// 0, 3, 6, 9, four times
    EXPECT_EQ(num_values_received, 4);
}

TEST_F(NodeTest, BufferTest) {
    auto timer = node_->icey().create_timer(50ms);
    
    std::vector<std::vector<size_t>> received_values;
     timer
     .then([&](size_t ticks) -> std::optional<size_t> {
        if(ticks >= 15) {
            timer.impl()->timer->cancel();
            return {};
       }
       return ticks;
    })
     .buffer(5)
     .then([&](std::shared_ptr<std::vector<size_t>> value) {
        received_values.push_back(*value);
    });

    std::vector<std::vector<size_t>> target_values;
    for(auto i :{0, 1, 2}) {
        target_values.emplace_back();
        for(auto j = 0; j < 5; j++) {
            target_values.back().push_back(i * 5 + j);
        }
    }

    spin(1100ms);

    EXPECT_EQ(received_values, target_values);
    
}