#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <flor_ocs_msgs/OCSTemplateAdd.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateUpdate.h>
#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <vigir_object_template_msgs/GetTemplateStateAndTypeInfo.h>
#include <vigir_object_template_msgs/GetGraspInfo.h>
#include <vigir_object_template_msgs/GetInstantiatedGraspInfo.h>
#include <vigir_object_template_msgs/SetAttachedObjectTemplate.h>
#include <vigir_object_template_msgs/DetachObjectTemplate.h>
#include <vigir_object_template_msgs/Affordance.h>
#include <vigir_object_template_msgs/Usability.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>

#include <tinyxml.h>

namespace ocs_template
{

    struct VigirObjectTemplate
    {
        uint16_t                                                     id;
        uint16_t                                                     type;
        std::string                                                  name;
        float                                                        mass;
        geometry_msgs::Point                                         com;
        geometry_msgs::Point                                         b_max;
        geometry_msgs::Point                                         b_min;
        shape_msgs::Mesh                                             mesh;
        std::string                                                  path;
        std::map<unsigned int,moveit_msgs::Grasp>                    grasps;
        std::map<unsigned int,vigir_object_template_msgs::StandPose> stand_poses;
        std::map<unsigned int,vigir_object_template_msgs::Usability> usabilities;
        std::map<unsigned int,vigir_object_template_msgs::Affordance>affordances;

        VigirObjectTemplate() : id(0),
                                type(0),
                                mass(0.0)
        {
            //com   = geometry_msgs::Point(0.0,0.0,0.0);
        }
    };

    class TemplateNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addTemplateCb(const flor_ocs_msgs::OCSTemplateAdd::ConstPtr& msg);
        void removeTemplateCb(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg);
        void updateTemplateCb(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& msg);
        void graspRequestCb(const flor_grasp_msgs::GraspSelection::ConstPtr& msg);
        void graspStateFeedbackCb(const flor_grasp_msgs::GraspState::ConstPtr& msg);
        void templateMatchFeedbackCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg);
        void publishTemplateList();
        std::vector< std::vector <std::string> > readCSVFile(std::string& file_name);
        void loadObjectTemplateDatabaseXML(std::string& file_name);
        void loadGraspDatabaseXML(std::string& file_name, std::string hand_side);
        void loadStandPosesDatabaseXML(std::string& file_name);
        int  worldPoseTransform(const geometry_msgs::PoseStamped& template_pose, const geometry_msgs::Pose &input_pose, geometry_msgs::PoseStamped &target_pose);
        int  poseTransform(geometry_msgs::Pose& first_pose, geometry_msgs::Pose& second_pose);
        int  staticTransform(geometry_msgs::Pose& palm_pose, tf::Transform gp_T_hand);
        void gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans);
        void timerCallback(const ros::TimerEvent& event);
        bool templateInfoSrv(vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Request& req,
                             vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Response& res);

        bool graspInfoSrv(vigir_object_template_msgs::GetGraspInfo::Request& req,
                          vigir_object_template_msgs::GetGraspInfo::Response& res);

        bool instantiatedGraspInfoSrv(vigir_object_template_msgs::GetInstantiatedGraspInfo::Request& req,
                                      vigir_object_template_msgs::GetInstantiatedGraspInfo::Response& res);

        bool attachObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                     vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res);

        bool stitchObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                     vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res);

        bool detachObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                     vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res);

        //Planning Scene
        void addCollisionObject(int type, int index, std::string mesh_name, geometry_msgs::Pose pose);
        void moveCollisionObject(int template_id, geometry_msgs::Pose pose);
        void removeCollisionObject(int template_id);

      protected:
        ros::Subscriber template_update_sub_;
        ros::Subscriber template_add_sub_;
        ros::Subscriber template_remove_sub_;
        ros::Subscriber grasp_request_sub_;
        ros::Subscriber grasp_state_feedback_sub_;
        ros::Subscriber template_match_feedback_sub_;
        ros::Publisher  template_list_pub_;
        ros::Publisher  grasp_selected_pub_;
        ros::Publisher  grasp_selected_state_pub_;
        ros::Publisher  co_pub_;
        ros::Publisher  aco_pub_;

        ros::ServiceServer template_info_server_;
        ros::ServiceServer grasp_info_server_;
        ros::ServiceServer inst_grasp_info_server_;
        ros::ServiceServer attach_object_server_;
        ros::ServiceServer stitch_object_server_;
        ros::ServiceServer detach_object_server_;

        ros::Timer image_publish_timer_;

      private:
        std::vector<unsigned char>                 template_id_list_;
        std::vector<unsigned char>                 template_type_list_;
        std::vector<std::string>                   template_name_list_;
        std::vector<geometry_msgs::PoseStamped>    template_pose_list_;
        std::vector<unsigned char>                 template_status_list_; //0-normal, 1-attached
        unsigned char                              id_counter_;
        // Filename of the grasping library
        std::string                                r_grasps_filename_;
        std::string                                l_grasps_filename_;
        // Filename of the object template library
        std::string                                ot_filename_;
        // Filename of the stand pose library
        std::string                                stand_filename_;
        tf::Transform                              gp_T_lhand_;
        tf::Transform                              gp_T_rhand_;
        std::map<unsigned int,VigirObjectTemplate> object_template_map_;

        robot_model_loader::RobotModelLoaderPtr    robot_model_loader_;
        robot_model::RobotModelPtr                 robot_model_;
        std::vector<std::string>                   hand_joint_names_;
        std::vector<std::string>                   hand_link_names_;

        ros::Timer timer;

        tf::TransformBroadcaster tfb_;
    };
}
