
/**
 * @file   nmpc_bluerov2_2-D_main.cpp
 * @author Mohit Mehindratta / Hakim Amer
 * @date   Jan 2023
 *
 */

#include <nmpc_bluerov2_2D_main.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#define SAMPLE_TIME 20ms

using namespace std;
using namespace Eigen;
using namespace rclcpp;
using std::placeholders::_1;


void NMPCBlueROV2Node::ref_position_cb(const geometry_msgs::msg::Vector3::ConstSharedPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void NMPCBlueROV2Node::ref_velocity_cb(const geometry_msgs::msg::Vector3::ConstSharedPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void NMPCBlueROV2Node::ref_yaw_cb(const std_msgs::msg::Float64::ConstSharedPtr& msg)
{
    ref_yaw_rad = msg->data;
}

void NMPCBlueROV2Node::pos_cb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    current_vel_rate = {msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z,
                        msg->twist.twist.angular.x,
                        msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z};

    current_pos_att = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw};
}

void NMPCBlueROV2Node::vel_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
{
    current_vel_body = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}

void NMPCBlueROV2Node::orientation_cb(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& msg)
{   
    
    angles = {msg->vector.x*(M_PI/180),
                   msg->vector.y*(M_PI/180),
                   msg->vector.z*(M_PI/180)};   
    angles_d ={msg->vector.x,
                   msg->vector.y,
                   msg->vector.z}; 
}

void NMPCBlueROV2Node::dist_Fx_predInit_cb(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    dist_Fx.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fx.predInit && dist_Fx.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fx estimates! \n";
        dist_Fx.print_predInit = 0;
    }
}

void NMPCBlueROV2Node::dist_Fy_predInit_cb(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    dist_Fy.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fy.predInit && dist_Fy.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fy estimates! \n";
        dist_Fy.print_predInit = 0;
    }
}

void NMPCBlueROV2Node::dist_Fz_predInit_cb(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    dist_Fz.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fz.predInit && dist_Fz.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fz estimates! \n";
        dist_Fz.print_predInit = 0;
    }
}

void NMPCBlueROV2Node::dist_Fx_data_cb(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
{
    if (use_dist_estimates && dist_Fx.predInit)
    {
        dist_Fx.data.clear();
        dist_Fx.data.insert(dist_Fx.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fx.data = dist_Fx.data_zeros;
}

void NMPCBlueROV2Node::dist_Fy_data_cb(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
{
    if (use_dist_estimates && dist_Fy.predInit)
    {
        dist_Fy.data.clear();
        dist_Fy.data.insert(dist_Fy.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fy.data = dist_Fy.data_zeros;
}

void NMPCBlueROV2Node::dist_Fz_data_cb(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
{
    if (use_dist_estimates && dist_Fz.predInit)
    {
        dist_Fz.data.clear();
        dist_Fz.data.insert(dist_Fz.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fz.data = dist_Fz.data_zeros;
}

void NMPCBlueROV2Node::publish_wrench(struct NMPC_PC::command_struct& commandstruct)
{

    geometry_msgs::msg::Wrench nmpc_wrench_msg;
    
    nmpc_wrench_msg.force.x =    commandstruct.control_wrench_vec[0];
    nmpc_wrench_msg.force.y =    commandstruct.control_wrench_vec[1];
    nmpc_wrench_msg.force.z =    commandstruct.control_wrench_vec[2];

    nmpc_wrench_msg.torque.x =    0.0;
    nmpc_wrench_msg.torque.y =    0.0;
    nmpc_wrench_msg.torque.z =   commandstruct.control_wrench_vec[3];

    nmpc_cmd_wrench_pub->publish(nmpc_wrench_msg);
    

    std_msgs::msg::Float64 exe_time_msg;
    exe_time_msg.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub->publish(exe_time_msg);

    std_msgs::msg::Float64 kkt_tol_msg;
    kkt_tol_msg.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub->publish(kkt_tol_msg);

    std_msgs::msg::Float64 obj_val_msg;
    obj_val_msg.data = commandstruct.obj_val;
    nmpc_cmd_obj_pub->publish(obj_val_msg);
}

void NMPCBlueROV2Node::publish_pred_tarjectory(struct NMPC_PC::acado_struct& traj_struct)
{
      // Create an instance of the Float32MultiArray message type
    std_msgs::msg::Float64MultiArray pred_traj_msg;

    // Resize the data array based on the size of nmpc_pc->nmpc_struct.x
    pred_traj_msg.data.resize(NMPC_NX * (NMPC_N + 1));


    for (int i = 0; i < NMPC_NX * (NMPC_N + 1); ++i)
    {
       // pred_traj_msg.data[i] = traj_struct.x[i];
        pred_traj_msg.data[i] =  nmpc_pc->nmpc_struct.x[0+9];
    }
   

    nmpc_pred_traj_pub->publish(pred_traj_msg);
  
    // a = nmpc_pc->nmpc_struct.x[0+9] <<endl;
}

NMPCBlueROV2Node::NMPCBlueROV2Node() : Node("bluerov2_nmpc_node")
{
    // ---------------
    // Main loop timer
    // ---------------
     main_loop_timer = create_wall_timer(SAMPLE_TIME, std::bind(&NMPCBlueROV2Node::main_loop, this));

    mocap_topic_part = "/predefined/mocap_topic_part";
    dist_Fx_predInit_topic = "/predefined/dist_Fx_predInit_topic";
    dist_Fy_predInit_topic = "/predefined/dist_Fy_predInit_topic";
    dist_Fz_predInit_topic = "/predefined/dist_Fz_predInit_topic";
    dist_Fx_data_topic = "/predefined/dist_Fx_data_topic";
    dist_Fy_data_topic = "/predefined/dist_Fy_data_topic";
    dist_Fz_data_topic = "/predefined/dist_Fz_data_topic";

    // -----------
    // Subscribers
    // -----------
    ref_position_sub = create_subscription<geometry_msgs::msg::Vector3>("ref_trajectory/position", 1, std::bind(&NMPCBlueROV2Node::ref_position_cb, this, _1));
    ref_velocity_sub = create_subscription<geometry_msgs::msg::Vector3>("ref_trajectory/velocity", 1, std::bind(&NMPCBlueROV2Node::ref_velocity_cb, this, _1));
    ref_yaw_sub = create_subscription<std_msgs::msg::Float64>("ref_trajectory/yaw", 1, std::bind(&NMPCBlueROV2Node::ref_yaw_cb, this, _1));
    pos_sub = create_subscription<nav_msgs::msg::Odometry>("/mobula/rov/odometry", 1, std::bind(&NMPCBlueROV2Node::pos_cb, this, _1));
    dist_Fx_predInit_sub = create_subscription<std_msgs::msg::Bool>(dist_Fx_predInit_topic, 1, std::bind(&NMPCBlueROV2Node::dist_Fx_predInit_cb, this, _1));
    dist_Fy_predInit_sub = create_subscription<std_msgs::msg::Bool>(dist_Fy_predInit_topic, 1, std::bind(&NMPCBlueROV2Node::dist_Fy_predInit_cb, this, _1));
    dist_Fz_predInit_sub = create_subscription<std_msgs::msg::Bool>(dist_Fz_predInit_topic, 1, std::bind(&NMPCBlueROV2Node::dist_Fz_predInit_cb, this, _1));
    dist_Fx_data_sub = create_subscription<std_msgs::msg::Float64MultiArray>(dist_Fx_data_topic, 1, std::bind(&NMPCBlueROV2Node::dist_Fx_data_cb, this, _1));
    dist_Fy_data_sub = create_subscription<std_msgs::msg::Float64MultiArray>(dist_Fy_data_topic, 1, std::bind(&NMPCBlueROV2Node::dist_Fy_data_cb, this, _1));
    dist_Fz_data_sub = create_subscription<std_msgs::msg::Float64MultiArray>(dist_Fz_data_topic, 1, std::bind(&NMPCBlueROV2Node::dist_Fz_data_cb, this, _1));
    orientation_sub = create_subscription<geometry_msgs::msg::Vector3Stamped>("/mobula/rov/orientation", 1, std::bind(&NMPCBlueROV2Node::orientation_cb, this, _1));

    // ----------
    // Publishers
    // ----------
    nmpc_cmd_wrench_pub = create_publisher<geometry_msgs::msg::Wrench>("/mobula/rov/wrench", 1);
    nmpc_cmd_exeTime_pub = create_publisher<std_msgs::msg::Float64>("outer_nmpc_cmd/exeTime", 1);
    nmpc_cmd_kkt_pub = create_publisher<std_msgs::msg::Float64>("outer_nmpc_cmd/kkt", 1);
    nmpc_cmd_obj_pub = create_publisher<std_msgs::msg::Float64>("outer_nmpc_cmd/obj", 1);

    nmpc_pred_traj_pub = create_publisher<std_msgs::msg::Float64MultiArray>("nmpc_predicted_trajectory", 1); 
    
    s_sdot_pub = create_publisher<std_msgs::msg::Float64MultiArray>("outer_nmpc_cmd/s_sdot", 1);

    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);
    
    // Roslaunch parameters
      
    declare_parameter("verbose", rclcpp::PARAMETER_BOOL);
    nmpc_struct.verbose = get_parameter("verbose").as_bool();
    declare_parameter("yaw_control", rclcpp::PARAMETER_BOOL);
    nmpc_struct.yaw_control = get_parameter("yaw_control").as_bool();
    declare_parameter("online_ref_yaw", rclcpp::PARAMETER_BOOL);
    online_ref_yaw = get_parameter("online_ref_yaw").as_bool();
    declare_parameter("use_dist_estimates", rclcpp::PARAMETER_BOOL);
    use_dist_estimates = get_parameter("use_dist_estimates").as_bool();


    declare_parameter("W_Wn_factor", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W_Wn_factor = get_parameter("W_Wn_factor").as_double();
    
    int u_idx = 0;
    declare_parameter("F_x_ref", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.U_ref(u_idx++) = get_parameter("F_x_ref").as_double();
    declare_parameter("F_y_ref", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.U_ref(u_idx++) = get_parameter("F_y_ref").as_double();
    declare_parameter("F_z_ref", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.U_ref(u_idx++) = get_parameter("F_z_ref").as_double();
    declare_parameter("M_z_ref", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.U_ref(u_idx++) = get_parameter("M_z_ref").as_double();

    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    declare_parameter("W_x", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_x").as_double();
    declare_parameter("W_y", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_y").as_double();
    declare_parameter("W_z", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_z").as_double();
    declare_parameter("W_u", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_u").as_double();
    declare_parameter("W_v", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_v").as_double();
    declare_parameter("W_w", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_w").as_double();
    declare_parameter("W_psi", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_psi").as_double();
    declare_parameter("W_r", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_r").as_double();
    declare_parameter("W_Fx", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_Fx").as_double();
    declare_parameter("W_Fy", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_Fy").as_double();
    declare_parameter("W_Fz", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_Fz").as_double();
    declare_parameter("W_Mz", rclcpp::PARAMETER_DOUBLE);
    nmpc_struct.W(w_idx++) = get_parameter("W_Mz").as_double();
    assert(w_idx == NMPC_NY);

    nmpc_struct.sample_time = std::chrono::duration<double>(SAMPLE_TIME).count();

    nmpc_pc = new NMPC_PC(nmpc_struct);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_vel_body.resize(6);
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    ref_traj_type = 0;
    ref_position << 0, 0, 0;
    ref_velocity << 0, 0, 0;

    angles = { 0,0,0};

    if (!nmpc_pc->return_control_init_value())
    {
        nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);
        if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
        {
            RCLCPP_INFO(get_logger(), "NMPC: initialized correctly\n");
        }
    }
}

void NMPCBlueROV2Node::main_loop()
{
    current_states = {
        current_pos_att.at(0),
        current_pos_att.at(1),
        current_pos_att.at(2),
        current_vel_rate.at(0),
        current_vel_rate.at(1),
        current_vel_rate.at(2),
        angles.at(2),
        current_vel_rate.at(5)
    };

    ref_trajectory = {
        ref_position[0],   //x
        ref_position[1],   //y
        ref_position[2],   //z
        ref_velocity[0],   //u
        ref_velocity[1],   //v
        ref_velocity[2],   //w
        ref_yaw_rad,
        0.0
    };                   

    std::cout << "current_states = ";
    for (int idx = 0; idx < current_states.size(); idx++)
    {
        std::cout << current_states[idx] << ",";
    }
    std::cout << "\n";

    std::cout << "ref_trajectory = ";
    for (int idx = 0; idx < ref_trajectory.size(); idx++)
    {
        std::cout << ref_trajectory[idx] << ",";
    }
    std::cout << "\n";

            
    std::cout << "ref  yaw = "<< ref_yaw_rad << std::endl ;
    std::cout << "current yaw = "<< angles.at(2) << std::endl ;

    double F_d_max = 50;
    
    for (size_t i = 0; i < dist_Fx.data.size(); ++i) {
        dist_Fx.data[i] = std::min(std::max(dist_Fx.data[i], -F_d_max), F_d_max);
        dist_Fy.data[i] = std::min(std::max(dist_Fy.data[i], -F_d_max), F_d_max);
        dist_Fz.data[i] = std::min(std::max(dist_Fz.data[i], -F_d_max), F_d_max);
    }
                
    online_data.distFx = dist_Fx.data;
    online_data.distFy = dist_Fy.data;
    online_data.distFz = dist_Fx.data_zeros;

    std::cout << "\033[1;31m" << "online_data = " << online_data.distFx[0] << " (sec)" << "\033[0m" << std::endl;
    std::cout << "\033[1;31m" << "online_data = " << online_data.distFy[0] << " (sec)" << "\033[0m" << std::endl;
    std::cout << "\033[1;31m" << "online_data = " << online_data.distFz[0] << " (sec)" << "\033[0m" << std::endl;

    nmpc_pc->nmpc_core(nmpc_struct,
                    nmpc_pc->nmpc_struct,
                    nmpc_pc->nmpc_cmd_struct,
                    ref_trajectory,
                    online_data,
                    current_states);

    if (nmpc_pc->acado_feedbackStep_fb != 0 ||
        std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
        std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Controller ERROR at time = " << get_clock()->now().seconds() << " (sec)");
        rclcpp::shutdown();
    }

    std::cout << "predicted dist x "<< online_data.distFx[0]<<endl;

    publish_pred_tarjectory(nmpc_pc->nmpc_struct);
    publish_wrench(nmpc_pc->nmpc_cmd_struct);

    print_flag_offboard = 1;
    print_flag_arm = 1;
    print_flag_altctl = 1;
}

int main(int argc, char * argv[])
{ 
    // Use a single-threaded executor - Node is not thread-safe
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NMPCBlueROV2Node>());
    rclcpp::shutdown();
    return 0;
}
