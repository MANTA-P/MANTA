#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <string>
#include <dave_interfaces/msg/dvl.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// --- 전역 변수 (클래스 멤버 변수 대체) ---
rclcpp::Node::SharedPtr g_node = nullptr;
std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> g_thruster_pubs;

// --- 퍼블리시 전용 함수 (제어 명령 송신) ---
// thruster_index: 0~5 (모터 1~6 매칭), thrust_value: 추력 값
void send_thruster_cmd(int thruster_index, double thrust_value) {
    if (thruster_index >= 0 && thruster_index < 6) {
        auto msg = std_msgs::msg::Float64();
        msg.data = thrust_value;
        g_thruster_pubs[thruster_index]->publish(msg);
        
        // [출력 확인용] 주석 해제 시 모터 제어 명령 송신 확인
        // RCLCPP_INFO(g_node->get_logger(), "모터 %d로 추력 %f 전송 완료", thruster_index + 1, thrust_value);
    }
}

// --- 콜백 함수 (센서 데이터 수신) ---
void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    // [입력 확인용] 수압 데이터 수신 확인
    //RCLCPP_INFO(g_node->get_logger(), "수압 데이터: %f", msg->fluid_pressure);
    
    // TODO: 수압 데이터를 이용한 제어 로직 추가
}

void depth_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // [입력 확인용] 수심 데이터 수신 확인 (보통 Point의 z값에 수심이 들어갑니다)
    RCLCPP_INFO(g_node->get_logger(), "수심 데이터: %.3f m", msg->point.z);
    
    // TODO: 수심 데이터를 이용한 제어 로직 추가
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // [입력 확인용] IMU 데이터(자세 4축, 각속도 3축, 선형 가속도 3축) 수신 확인
    // RCLCPP_INFO(g_node->get_logger(), "자세 [X: %.3f, Y: %.3f, Z: %.3f, W: %.3f] | 각속도 [X: %.3f, Y: %.3f, Z: %.3f] | 가속도 [X: %.3f, Y: %.3f, Z: %.3f]",
    //             msg->orientation.x,
    //             msg->orientation.y,
    //             msg->orientation.z,
    //             msg->orientation.w,
    //             msg->angular_velocity.x,
    //             msg->angular_velocity.y,
    //             msg->angular_velocity.z,
    //             msg->linear_acceleration.x,
    //             msg->linear_acceleration.y,
    //             msg->linear_acceleration.z);
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // [입력 확인용] 오도메트리(위치 3축 및 자세 4축 쿼터니언) 수신 확인
    // RCLCPP_INFO(g_node->get_logger(), "위치 [X: %.3f, Y: %.3f, Z: %.3f] | 자세 [X: %.3f, Y: %.3f, Z: %.3f, W: %.3f]", 
    //             msg->pose.pose.position.x, 
    //             msg->pose.pose.position.y, 
    //             msg->pose.pose.position.z,
    //             msg->pose.pose.orientation.x,
    //             msg->pose.pose.orientation.y,
    //             msg->pose.pose.orientation.z,
    //             msg->pose.pose.orientation.w);

    // [출력 테스트용] 오도메트리를 받을 때마다 모터1에 1.5 추력 전송 테스트
    // send_thruster_cmd(0, 1.5);
}

void dvl_callback(const dave_interfaces::msg::DVL::SharedPtr msg) {
    // [입력 확인용] DVL 선형 속도(Linear Velocity) 수신 확인
    // RCLCPP_INFO(g_node->get_logger(), "DVL 속도 [X: %.3f, Y: %.3f, Z: %.3f]",
    //             msg->velocity.twist.linear.x,
    //             msg->velocity.twist.linear.y,
    //             msg->velocity.twist.linear.z);
    
    // // TODO: DVL 속도 데이터를 이용한 제어 로직 추가
}

// --- 메인 함수 ---
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // 1. 노드 생성 (이름은 기존과 동일하게 유지하여 CMake 수정 방지)
    g_node = rclcpp::Node::make_shared("uart_bridge_node");

    // 2. 송신부 (Publisher) 초기화: 모터 1~6 추력 명령
    for (int i = 1; i <= 6; ++i) {
        std::string topic = "/model/bluerov2/joint/thruster" + std::to_string(i) + "_joint/cmd_thrust";
        //g_thruster_pubs.push_back(g_node->create_publisher<std_msgs::msg::Float64>(topic, 10));
    }

    // 3. 수신부 (Subscriber) 초기화 및 콜백 함수 연결
    auto pressure_sub = g_node->create_subscription<sensor_msgs::msg::FluidPressure>(
        "/model/bluerov2/pressure", 10, pressure_callback);

    auto depth_sub = g_node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/model/bluerov2/Pressure_depth", 10, depth_callback);

    auto imu_sub = g_node->create_subscription<sensor_msgs::msg::Imu>(
        "/model/bluerov2/imu", 10, imu_callback);

    auto odom_sub = g_node->create_subscription<nav_msgs::msg::Odometry>(
        "/model/bluerov2/odometry", 10, odom_callback);

    auto dvl_sub = g_node->create_subscription<dave_interfaces::msg::DVL>(
        "/dvl/velocity", 10, dvl_callback);
        

    RCLCPP_INFO(g_node->get_logger(), "SITL 통신용 브릿지 노드 시작 완료");

    // 4. 노드 실행 (콜백 대기)
    rclcpp::spin(g_node);
    
    rclcpp::shutdown();
    return 0;
}