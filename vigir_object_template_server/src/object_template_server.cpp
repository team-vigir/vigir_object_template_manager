/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// Authors: Alberto Romay and Felipe Bacim
#include "object_template_server.h"

namespace object_template_server
{
void ObjectTemplateServer::onInit()
{
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle nh_out(nh, "template");
    ros::NodeHandle nhp("~");

    if (!nh.getParam("manipulators", manipulators_))
        ROS_ERROR(" Did not find manipulators parameter");
    else
        ROS_ASSERT(manipulators_.getType() == XmlRpc::XmlRpcValue::TypeStruct);


    //Template Services
    template_info_server_      = nh_out.advertiseService("/template_info", &ObjectTemplateServer::templateInfoSrv, this);
    grasp_info_server_         = nh_out.advertiseService("/grasp_info", &ObjectTemplateServer::graspInfoSrv, this);

    ROS_INFO(" Start reading database files");

    //LOAD OBJECT TEMPLATE LIBRARY
    if (!nhp.getParam("/object_templates/ot_library", this->ot_filename_))
        ROS_ERROR(" Did not find Object Template Library parameter /object_templates/ot_library");
    else
        ObjectTemplateServer::loadObjectTemplateDatabaseXML(this->ot_filename_);

    //LOADING ROBOT MODEL FOR JOINT NAMES
    if (!nh.hasParam("/robot_description"))
        ROS_ERROR(" Did not find robot_description parameter! Not loading grasp library nor robot library");
    else{
        robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
        robot_model_ = robot_model_loader_->getModel();

        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = manipulators_.begin(); it != manipulators_.end(); ++it) {
            std::string temp_file;
            if (!nhp.getParam("/grasp_library/"+std::string(it->first), temp_file))
                ROS_WARN(" Did not find Right Grasp Library parameter /object_templates/%s, not using right grasps", std::string(it->first).c_str());
            else
                ObjectTemplateServer::loadGraspDatabaseXML(temp_file, std::string(it->first));
        }

        // Load the robot specific ghost_poses database
        if (!nhp.getParam("/object_templates/stand_poses_library", this->stand_filename_))
            ROS_WARN(" Did not find Stand Poses parameter /object_templates/stand_poses_library, not using stand poses for robot");
        else
            ObjectTemplateServer::loadStandPosesDatabaseXML(this->stand_filename_);
    }
}

void ObjectTemplateServer::loadGraspDatabaseXML(std::string& file_name, std::string gripper)
{
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = manipulators_.begin(); it != manipulators_.end(); ++it) {
        if (std::string(it->first) == gripper){
            palm_link_  = std::string(manipulators_[it->first]["palm_link"]);
            wrist_link_ = std::string(manipulators_[it->first]["wrist_link"]);
            gripper_group_ = std::string(manipulators_[it->first]["gripper_group"]);
        }
    }

    if(robot_model_->hasJointModelGroup(gripper_group_))
    {
        hand_joint_names_.clear();
        hand_joint_names_ = robot_model_->getJointModelGroup(gripper_group_)->getActiveJointModelNames();
    }
    else
    {
        ROS_WARN("NO JOINTS FOUND FOR %s GRIPPER in %s",gripper.c_str(), gripper_group_.c_str());
    }

    ROS_INFO("%s %s hand model gotten, #actuated joints: %ld ",gripper.c_str(), robot_model_->getName().c_str(),hand_joint_names_.size() );

    for(int i = 0; i < hand_joint_names_.size(); i++)
        ROS_INFO("Joint %d: %s",i,hand_joint_names_[i].c_str());

    //Creating XML document from parameter server string of grasp library
    TiXmlDocument doc;
    doc.Parse((const char*)file_name.c_str(), 0, TIXML_ENCODING_UTF8);
    if (doc.ErrorId() != 0)
    {
        ROS_ERROR("Could not read file for %s hand",gripper.c_str());
        return;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);

    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem)
    {
        ROS_ERROR("File for %s hand read but empty", gripper.c_str());
        return;
    }

    // save this for later
    hRoot=TiXmlHandle(pElem);
    ROS_INFO("Reading %s for %s hand", pElem->Value(), gripper.c_str());

    TiXmlElement* pGrasps=hRoot.FirstChild( "grasps" ).Element();
    for( pGrasps; pGrasps; pGrasps=pGrasps->NextSiblingElement("grasps")) //Iterates thorugh all template types
    {
        const char *pName=pGrasps->Attribute("template_name");
        int template_type;
        pGrasps->QueryIntAttribute("template_type", &template_type);
        if (pName) ROS_DEBUG("Reading Grasps for %s type: %d",pName, template_type);

        TiXmlElement* pGrasp=pGrasps->FirstChildElement( "grasp" );
        for( pGrasp; pGrasp; pGrasp=pGrasp->NextSiblingElement( "grasp" ) )   //Iterates thorugh all grasp IDs for this particular template type
        {
            moveit_msgs::Grasp grasp;
            float x,y,z,qx,qy,qz,qw;
            grasp.grasp_pose.header.frame_id = wrist_link_;

            const char *pID=pGrasp->Attribute("id");
            if (pID) ROS_DEBUG("Found Grasp id: %s",pID);
            grasp.id = std::string(pID);

            TiXmlElement* pPose=pGrasp->FirstChildElement( "final_pose" );       //Gets final grasp pose
            if(!pPose)
            {
                ROS_DEBUG("Grasp ID: %s does not contain an final pose, setting identity",pID);
                grasp.grasp_pose.pose.position.x    = 0.0;
                grasp.grasp_pose.pose.position.y    = 0.0;
                grasp.grasp_pose.pose.position.z    = 0.0;
                grasp.grasp_pose.pose.orientation.x = 0.0;
                grasp.grasp_pose.pose.orientation.y = 0.0;
                grasp.grasp_pose.pose.orientation.z = 0.0;
                grasp.grasp_pose.pose.orientation.w = 1.0;
            }
            else
            {
                pPose->QueryFloatAttribute("x",  &x);
                pPose->QueryFloatAttribute("y",  &y);
                pPose->QueryFloatAttribute("z",  &z);
                pPose->QueryFloatAttribute("qx", &qx);
                pPose->QueryFloatAttribute("qy", &qy);
                pPose->QueryFloatAttribute("qz", &qz);
                pPose->QueryFloatAttribute("qw", &qw);

                grasp.grasp_pose.pose.position.x    = x;
                grasp.grasp_pose.pose.position.y    = y;
                grasp.grasp_pose.pose.position.z    = z;
                grasp.grasp_pose.pose.orientation.x = qx;
                grasp.grasp_pose.pose.orientation.y = qy;
                grasp.grasp_pose.pose.orientation.z = qz;
                grasp.grasp_pose.pose.orientation.w = qw;

                grasp.grasp_pose.header.frame_id    = wrist_link_;
            }

            ROS_DEBUG_STREAM("Added grasp information id: " << grasp.id << " pose: " << std::endl << grasp.grasp_pose.pose);

            TiXmlElement* pApproachingVector=pGrasp->FirstChildElement( "approaching_vector" );       //Gets approaching vector
            if(!pApproachingVector)
            {
                ROS_DEBUG("Grasp ID: %s does not contain an approaching vector, setting default values",pID);
                grasp.pre_grasp_approach.direction.vector.x = 0.00;
                grasp.pre_grasp_approach.direction.vector.y = 1.00;
                grasp.pre_grasp_approach.direction.vector.z = 0.00;
                grasp.pre_grasp_approach.desired_distance   = 0.20;
                grasp.pre_grasp_approach.min_distance       = 0.05;
            }
            else
            {
                grasp.pre_grasp_approach.direction.header.frame_id = wrist_link_;

                pApproachingVector->QueryFloatAttribute("x", &x);
                pApproachingVector->QueryFloatAttribute("y", &y);
                pApproachingVector->QueryFloatAttribute("z", &z);

                grasp.pre_grasp_approach.direction.vector.x = x;
                grasp.pre_grasp_approach.direction.vector.y = y;
                grasp.pre_grasp_approach.direction.vector.z = z;

                pApproachingVector->QueryFloatAttribute("desired", &grasp.pre_grasp_approach.desired_distance);
                pApproachingVector->QueryFloatAttribute("minimal", &grasp.pre_grasp_approach.min_distance);
            }

            ROS_DEBUG_STREAM("Added aproaching vector information id: " << grasp.id << " pose: " << std::endl << grasp.pre_grasp_approach);

            grasp.pre_grasp_posture.points.resize(1);
            grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

            TiXmlElement* pPrePosture=pGrasp->FirstChildElement( "pre_grasp_posture" );
            if(!pPrePosture)
            {
                ROS_DEBUG("Grasp ID: %s does not contain a pregrasp posture, setting all %d joints to zeros",pID, (int)hand_joint_names_.size());
                if(hand_joint_names_.size() > 0)
                {
                    grasp.pre_grasp_posture.joint_names.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.pre_grasp_posture.joint_names[j] = hand_joint_names_.at(j);
                    grasp.pre_grasp_posture.points[0].positions.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.pre_grasp_posture.points[0].positions[j] = 0.0;  //Setting default joint values to zeros
                }
                else
                {
                    ROS_DEBUG("Grasp ID: %s does not contain a pregrasp posture and URDF shows no %s hand joints",pID, gripper.c_str());
                }
            }
            else
            {
                TiXmlElement* pFinger=pPrePosture->FirstChildElement( "finger" );       //Gets pre finger joints
                if(!pFinger)
                {
                    ROS_DEBUG("Grasp ID: %s does not contain any finger, setting joints to zeros",pID);
                }
                else
                {
                    for( pFinger; pFinger; pFinger=pFinger->NextSiblingElement("finger"))   //Iterates thorugh all fingers for this particular pre posture
                    {
                        TiXmlElement* pJoint=pFinger->FirstChildElement( "joint" );       //Gets approaching vector
                        if(!pJoint)
                        {
                            ROS_DEBUG("Grasp ID: %s does not contain joints for finger %s",pID, pFinger->Attribute("idx"));
                        }
                        else
                        {
                            for( pJoint; pJoint; pJoint=pJoint->NextSiblingElement("joint"))   //Iterates thorugh all fingers for this particular pre posture
                            {
                                const char *pJointName=pJoint->Attribute("name");
                                if (pJointName)
                                    grasp.pre_grasp_posture.joint_names.push_back(pJointName);
                                else
                                    ROS_DEBUG("Found joint without name for finger %s",pFinger->Attribute("idx"));
                                pJoint->QueryFloatAttribute("value", &x);
                                grasp.pre_grasp_posture.points[0].positions.push_back(x);
                            }
                        }
                    }

                    ROS_DEBUG_STREAM("Added pre_grasp_posture information id: " << grasp.id << " pose: " << std::endl << grasp.pre_grasp_posture);
                }
            }

            grasp.grasp_posture.points.resize(1);
            grasp.grasp_posture.points[0].time_from_start = ros::Duration(0.5);

            TiXmlElement* pPosture=pGrasp->FirstChildElement( "grasp_posture" );
            if(!pPosture)
            {
                ROS_DEBUG("Grasp ID: %s does not contain a grasp posture, setting all %d joints to zeros",pID, (int)hand_joint_names_.size());
                if(hand_joint_names_.size() > 0)
                {
                    grasp.grasp_posture.joint_names.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.grasp_posture.joint_names[j] = hand_joint_names_.at(j);
                    grasp.grasp_posture.points[0].positions.resize(hand_joint_names_.size());
                    for(int j=0; j<hand_joint_names_.size();j++)
                        grasp.grasp_posture.points[0].positions[j] = 0.0;  //Setting default joint values to zeros
                }
                else
                {
                    ROS_DEBUG("Grasp ID: %s does not contain a grasp posture and URDF shows no %s hand joints",pID, gripper.c_str());
                }
            }
            else
            {
                TiXmlElement* pFinger=pPosture->FirstChildElement( "finger" );       //Gets final finger joints
                if(!pFinger)
                {
                    ROS_DEBUG("Grasp ID: %s does not contain any finger, setting joints to zeros",pID);
                }
                else
                {
                    for( pFinger; pFinger; pFinger=pFinger->NextSiblingElement("finger"))   //Iterates thorugh all fingers for this particular posture
                    {
                        TiXmlElement* pJoint=pFinger->FirstChildElement( "joint" );       //Gets approaching vector
                        if(!pJoint)
                        {
                            ROS_DEBUG("Grasp ID: %s does not contain joints for finger %s",pID, pFinger->Attribute("idx"));
                        }
                        else
                        {
                            for( pJoint; pJoint; pJoint=pJoint->NextSiblingElement("joint"))   //Iterates thorugh all fingers for this particular posture
                            {
                                const char *pJointName=pJoint->Attribute("name");
                                if (pJointName)
                                    grasp.grasp_posture.joint_names.push_back(pJointName);
                                else
                                    ROS_DEBUG("Found joint without name for finger %s",pFinger->Attribute("idx"));
                                pJoint->QueryFloatAttribute("value", &x);
                                grasp.grasp_posture.points[0].positions.push_back(x);
                            }
                        }
                    }

                    ROS_DEBUG_STREAM("Added grasp_posture information id: " << grasp.id << " pose: " << std::endl << grasp.grasp_posture);
                }
            }

            //RETREAT VECTOR (FIXING TO LIFT 10cm AFTER GRASPING)
            //ROS_DEBUG("Staring retreat vector idx: %d",idx);
            grasp.post_grasp_retreat.direction.header.frame_id = "/world";
            grasp.post_grasp_retreat.direction.vector.z        = 1.0;
            grasp.post_grasp_retreat.min_distance              = 0.05;
            grasp.post_grasp_retreat.desired_distance          = 0.1;

            boost::recursive_mutex::scoped_lock lock(object_template_map_mutex_);

            if(object_template_map_.find(template_type) != object_template_map_.end())   //Template Type exists
                object_template_map_[template_type].grasps.insert(std::pair<unsigned int,moveit_msgs::Grasp>(std::atoi(grasp.id.c_str()),grasp));
        }
    }

    boost::recursive_mutex::scoped_lock lock(object_template_map_mutex_);

    for (std::map<unsigned int,VigirObjectTemplate>::iterator it=object_template_map_.begin(); it!=object_template_map_.end(); ++it)
        for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it2=it->second.grasps.begin(); it2!=it->second.grasps.end(); ++it2)
            ROS_INFO("OT Map, inside ot: %d -> Grasp id %s ", it->second.type, it2->second.id.c_str());

    ROS_INFO("%s Grasp Database loaded",gripper.c_str());

}

void ObjectTemplateServer::loadStandPosesDatabaseXML(std::string& file_name){
    /*
     * Need to fill object_template_map_[type].stand_poses with the poses read from file
     *
     * ORDER IN FILE SHOULD MATCH ORDER IN READING
     * template type,
     * stand pose id,
     * stand pose relative to template (x,y,z,qw,qx,qy,qz),
    */
    ROS_INFO("Loading Stand Poses...");

    //Creating XML document from parameter server string of template library
    TiXmlDocument doc;
    doc.Parse((const char*)file_name.c_str(), 0, TIXML_ENCODING_UTF8);
    if (doc.ErrorId() != 0){
        ROS_ERROR("Could not read stand poses library file ");
        return;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);

    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem){
        ROS_ERROR("File for stand poses library empty");
        return;
    }

    // save this for later
    hRoot=TiXmlHandle(pElem);
    ROS_DEBUG("Reading %s", pElem->Value());



    TiXmlElement* pTemplate=hRoot.FirstChild( "template" ).Element();
    for( pTemplate; pTemplate; pTemplate=pTemplate->NextSiblingElement("template")) //Iterates thorugh all template types
    {
        int template_type;
        const char *pName=pTemplate->Attribute("template_name");
        pTemplate->QueryIntAttribute("template_type", &template_type);
        if (pName) ROS_DEBUG("Reading %s type: %d",pName, template_type);

        TiXmlElement* pStandPose=pTemplate->FirstChildElement( "standpose" );
        for( pStandPose; pStandPose; pStandPose=pStandPose->NextSiblingElement("standpose" )) //Iterates thorugh all template types
        {
            vigir_object_template_msgs::StandPose current_pose;
            TiXmlElement* pPose=pStandPose->FirstChildElement("pose");
            if(!pPose)
            {
                ROS_ERROR("Template ID: %d does not contain a  stand pose, skipping template",template_type);
                continue;
            }
            else
            {
                double qx,qy,qz,qw;
                std::string xyz = pPose->Attribute("xyz");
                std::istringstream iss(xyz);
                std::string word;
                std::vector<std::string> tokens;
                while ( iss >> word ) tokens.push_back( word );

                pPose->QueryDoubleAttribute("qx",&qx);
                pPose->QueryDoubleAttribute("qy",&qy);
                pPose->QueryDoubleAttribute("qz",&qz);
                pPose->QueryDoubleAttribute("qw",&qw);

                current_pose.id                      = std::atoi(pStandPose->Attribute("id"));
                current_pose.pose.header.frame_id    = "/world";
                current_pose.pose.header.stamp       = ros::Time::now();
                current_pose.pose.pose.position.x    = std::atof(tokens[0].c_str());
                current_pose.pose.pose.position.y    = std::atof(tokens[1].c_str());
                current_pose.pose.pose.position.z    = std::atof(tokens[2].c_str());
                current_pose.pose.pose.orientation.x = qx;
                current_pose.pose.pose.orientation.y = qy;
                current_pose.pose.pose.orientation.z = qz;
                current_pose.pose.pose.orientation.w = qw;

                boost::recursive_mutex::scoped_lock lock(object_template_map_mutex_);

                if(object_template_map_.find(template_type) != object_template_map_.end())   //Template Type exists
                    object_template_map_[template_type].stand_poses.insert(std::pair<unsigned int,vigir_object_template_msgs::StandPose>(current_pose.id, current_pose));
            }
        }
    }

    boost::recursive_mutex::scoped_lock lock(object_template_map_mutex_);

    for (std::map<unsigned int,VigirObjectTemplate>::iterator it=object_template_map_.begin(); it!=object_template_map_.end(); ++it)
        for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it2=it->second.stand_poses.begin(); it2!=it->second.stand_poses.end(); ++it2)
            ROS_INFO("OT Map, inside ot: %d -> Stand pose id %d ", it->second.type, it2->second.id);

    ROS_INFO("Stand Poses Database loaded");
}

void ObjectTemplateServer::loadObjectTemplateDatabaseXML(std::string& file_name)
{
    //Creating XML document from parameter server string of template library
    TiXmlDocument doc;
    doc.Parse((const char*)file_name.c_str(), 0, TIXML_ENCODING_UTF8);
    if (doc.ErrorId() != 0){
        ROS_ERROR("Could not read object library file ");
        return;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);

    pElem=hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!pElem){
        ROS_ERROR("File for template library empty");
        return;
    }

    // save this for later
    hRoot=TiXmlHandle(pElem);
    ROS_DEBUG("Reading %s", pElem->Value());

    TiXmlElement* pTemplate=hRoot.FirstChild( "template" ).Element();
    int count=0;
    for( pTemplate; pTemplate; pTemplate=pTemplate->NextSiblingElement( "template"), count++) //Iterates thorugh all template types
    {
        //Getting type and name
        VigirObjectTemplate object_template;
        const char *pName=pTemplate->Attribute("template_name");
        const char *pGroup=pTemplate->Attribute("group");
        int template_type;
        pTemplate->QueryIntAttribute("template_type", &template_type);
        if (pName) ROS_DEBUG("Reading %s type: %d",pName, template_type);

        object_template.name = pName;
        object_template.type = template_type;
        object_template.id   = count;
        object_template.path = std::string(pGroup) + "/" + std::string(pName);

//        //Getting Mesh Path
//        TiXmlElement* pMesh=pTemplate->FirstChildElement( "visual" )->FirstChildElement("geometry")->FirstChildElement("mesh");
//        if(!pMesh){
//            ROS_ERROR("Template ID: %d does not contain a mesh path, skipping template",template_type);
//            continue;
//        }else{
//            object_template.path = pMesh->Attribute("filename");
//        }


        //Getting Mass
        TiXmlElement* pMass=pTemplate->FirstChildElement( "inertial" )->FirstChildElement("mass");
        if(!pMass){
            ROS_DEBUG("Template ID: %d does not contain a mass value, setting to 0",template_type);
            object_template.mass = 0;
        }else{
            object_template.mass = std::atof(pMass->Attribute("value"));
        }

        //Getting CoM
        TiXmlElement* pCoM=pTemplate->FirstChildElement( "inertial" )->FirstChildElement("origin");
        if(!pCoM){
            ROS_DEBUG("Template ID: %d does not contain a mass value, setting to 0",template_type);
            object_template.com.x = object_template.com.y = object_template.com.z = 0;
        }else{
            geometry_msgs::Point com;
            std::string xyz = pCoM->Attribute("xyz");
            std::istringstream iss(xyz);
            std::string word;
            std::vector<std::string> tokens;
            while ( iss >> word ) tokens.push_back( word );

            com.x = std::atof(tokens[0].c_str());
            com.y = std::atof(tokens[1].c_str());
            com.z = std::atof(tokens[2].c_str());

            object_template.com = com;
        }

        //Getting Inertia
        TiXmlElement* pInertia=pTemplate->FirstChildElement( "inertial" )->FirstChildElement("inertia");
        if(!pInertia){
            ROS_DEBUG("Template ID: %d does not contain an inertia tensor",template_type);
        }else{
            //object_template.inertia.ixx = std::atof(pInertia->Attribute("ixx"));
            //object_template.inertia.ixy = std::atof(pInertia->Attribute("ixy"));
            //object_template.inertia.ixz = std::atof(pInertia->Attribute("ixz"));
            //object_template.inertia.iyy = std::atof(pInertia->Attribute("iyy"));
            //object_template.inertia.iyz = std::atof(pInertia->Attribute("iyz"));
            //object_template.inertia.izz = std::atof(pInertia->Attribute("izz"));
        }

        //Getting bounding box
        TiXmlElement* pBoundingBox=pTemplate->FirstChildElement( "visual" )->FirstChildElement("geometry")->FirstChildElement("boundingbox");
        if(!pBoundingBox){
            ROS_DEBUG("Template ID: %d does not contain a  bounding box, setting to zeros",template_type);
            object_template.b_max.x = object_template.b_max.y = object_template.b_max.z = 0;
            object_template.b_min.x = object_template.b_min.y = object_template.b_min.z = 0;
        }else{
            geometry_msgs::Point b_max;
            geometry_msgs::Point b_min;
            std::string xyz = pBoundingBox->Attribute("min");
            std::istringstream iss(xyz);
            std::string word;
            std::vector<std::string> tokens;
            while ( iss >> word ) tokens.push_back( word );

            b_min.x    = std::atof(tokens[0].c_str());
            b_min.y    = std::atof(tokens[1].c_str());
            b_min.z    = std::atof(tokens[2].c_str());

            xyz = pBoundingBox->Attribute("max");
            std::istringstream iss2(xyz);
            tokens.clear();
            while ( iss2 >> word ) tokens.push_back( word );

            b_max.x    = std::atof(tokens[0].c_str());
            b_max.y    = std::atof(tokens[1].c_str());
            b_max.z    = std::atof(tokens[2].c_str());

            object_template.b_max = b_max;
            object_template.b_min = b_min;
        }

        //Getting usabilities
        TiXmlElement* pUsability=pTemplate->FirstChildElement( "usability" );
        for( pUsability; pUsability; pUsability=pUsability->NextSiblingElement("usability")) //Iterates thorugh all usabilities
        {
            vigir_object_template_msgs::Usability usability;
            TiXmlElement* pPose=pUsability->FirstChildElement("pose");
            if(!pPose){
                ROS_ERROR("Template ID: %d does not contain a  pose in usability id: %s, skipping template",template_type,pUsability->Attribute("id"));
                continue;
            }else{
                double qx,qy,qz,qw;
                std::string xyz = pPose->Attribute("xyz");
                std::istringstream iss(xyz);
                std::string word;
                std::vector<std::string> tokens;
                while ( iss >> word ) tokens.push_back( word );

                pPose->QueryDoubleAttribute("qx",&qx);
                pPose->QueryDoubleAttribute("qy",&qy);
                pPose->QueryDoubleAttribute("qz",&qz);
                pPose->QueryDoubleAttribute("qw",&qw);

                usability.id                      = std::atoi(pUsability->Attribute("id"));
                usability.pose.header.frame_id    = "/world";
                usability.pose.header.stamp       = ros::Time::now();
                usability.pose.pose.position.x    = std::atof(tokens[0].c_str());
                usability.pose.pose.position.y    = std::atof(tokens[1].c_str());
                usability.pose.pose.position.z    = std::atof(tokens[2].c_str());
                usability.pose.pose.orientation.x = qx;
                usability.pose.pose.orientation.y = qy;
                usability.pose.pose.orientation.z = qz;
                usability.pose.pose.orientation.w = qw;

                if(pUsability->Attribute("name"))
                    usability.name = pUsability->Attribute("name");
                else{
                    ROS_DEBUG("Usability ID: %d has no name attribute, setting to usability_%d", usability.id, usability.id);
                    usability.name = "usability_" + boost::to_string(pUsability->Attribute("id"));
                }

                object_template.usabilities.insert(std::pair<unsigned int,vigir_object_template_msgs::Usability>(usability.id, usability));
            }
        }

        //Getting Affordances
        TiXmlElement* pAffordance=pTemplate->FirstChildElement( "affordance" );
        unsigned int aff_idx = 100;
        for( pAffordance; pAffordance; pAffordance=pAffordance->NextSiblingElement("affordance")) //Iterates thorugh all affordances
        {
            vigir_object_template_msgs::Affordance affordance;
            if(pAffordance->Attribute("id")){
                affordance.id = std::atoi(pAffordance->Attribute("id"));
            }else{
                ROS_DEBUG("Affordance ID not found, setting to %d",aff_idx);
                affordance.id = aff_idx;
                aff_idx++;
            }

            TiXmlElement* pPose=pAffordance->FirstChildElement("pose");
            if(!pPose){
                ROS_ERROR("Template ID: %d does not contain a  pose in affordance id: %s, skipping template",template_type,pAffordance->Attribute("id"));
                continue;
            }else{
                ROS_DEBUG("Getting poses for affordance %d", affordance.id);
                for(pPose; pPose; pPose=pPose->NextSiblingElement("pose")){

                    double qx,qy,qz,qw;
                    std::string xyz = pPose->Attribute("xyz");
                    std::istringstream iss(xyz);
                    std::string word;
                    std::vector<std::string> tokens;
                    while ( iss >> word ) tokens.push_back( word );

                    pPose->QueryDoubleAttribute("qx",&qx);
                    pPose->QueryDoubleAttribute("qy",&qy);
                    pPose->QueryDoubleAttribute("qz",&qz);
                    pPose->QueryDoubleAttribute("qw",&qw);

                    geometry_msgs::PoseStamped waypoint;

                    waypoint.header.frame_id    = "/world";
                    waypoint.header.stamp       = ros::Time::now();
                    waypoint.pose.position.x    = std::atof(tokens[0].c_str());
                    waypoint.pose.position.y    = std::atof(tokens[1].c_str());
                    waypoint.pose.position.z    = std::atof(tokens[2].c_str());
                    waypoint.pose.orientation.x = qx;
                    waypoint.pose.orientation.y = qy;
                    waypoint.pose.orientation.z = qz;
                    waypoint.pose.orientation.w = qw;
                    affordance.waypoints.push_back(waypoint);
                    ROS_DEBUG("Getting %d waypoints", (int)affordance.waypoints.size());
                }
                ROS_DEBUG("Finished getting poses");


                if(pAffordance->Attribute("name"))
                    affordance.name = pAffordance->Attribute("name");
                else{
                    ROS_DEBUG("Affordance ID: %d has no name attribute, setting to aff_%d", affordance.id, affordance.id);
                    affordance.name = "aff_" + boost::to_string(affordance.id);
                }

                if(pAffordance->Attribute("type"))
                    affordance.type = pAffordance->Attribute("type");
                else{
                    ROS_DEBUG("Affordance ID: %d has no type attribute, setting to no_type", affordance.id);
                    affordance.type = "no_type";
                }

                if(pAffordance->Attribute("axis"))
                    affordance.axis = pAffordance->Attribute("axis");
                else{
                    ROS_DEBUG("Affordance ID: %d has no axis attribute, setting to no_axis", affordance.id);
                    affordance.axis = "no_axis";
                }

                if(pAffordance->Attribute("pitch"))
                    affordance.pitch = std::atof(pAffordance->Attribute("pitch"));
                else{
                    ROS_DEBUG("Affordance ID: %d has no pitch attribute, setting to 0", affordance.id);
                    affordance.pitch = 0.0;
                }

                if(pAffordance->Attribute("displacement")){
                    if(affordance.type == "circular")
                        affordance.displacement = (std::atof(pAffordance->Attribute("displacement"))) * 0.0174532925; // Template library in deg, msg in rad;
                    else
                        affordance.displacement = std::atof(pAffordance->Attribute("displacement"));
                }else{
                    ROS_DEBUG("Affordance ID: %d has no displacement attribute, setting to zero", affordance.id);
                    affordance.displacement = 0.0;
                }

                if(pAffordance->Attribute("keeporientation")){
                    if(std::string(pAffordance->Attribute("keeporientation")) == std::string("true"))
                        affordance.keep_orientation = true;
                    else
                        affordance.keep_orientation = false;

                }else{
                    ROS_DEBUG("Affordance ID: %d has no keeporientation attribute, setting to true", affordance.id);
                    affordance.keep_orientation     = true;
                }

                object_template.affordances.insert(std::pair<unsigned int,vigir_object_template_msgs::Affordance>(affordance.id, affordance));
            }
        }

        boost::recursive_mutex::scoped_lock lock(object_template_map_mutex_);

        object_template_map_.insert(std::pair<unsigned int,VigirObjectTemplate>(template_type,object_template));
        ROS_INFO(" Inserting Object template type: %d with id: %d, name %s and mesh path: %s, aff.name: %s, aff.displacement: %f", object_template_map_[template_type].type
                                                                                             , object_template_map_[template_type].id
                                                                                             , object_template_map_[template_type].name.c_str()
                                                                                             , object_template_map_[template_type].path.c_str()
                                                                                             , object_template_map_[template_type].affordances[1].name.c_str()
                                                                                             , object_template_map_[template_type].affordances[1].displacement);
    }

    ROS_INFO("OT Database loaded");
}

void ObjectTemplateServer::gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans){
    geometry_msgs::Vector3Stamped direction = trans.direction;
    tf::Transform template_T_hand, vec_in, vec_out;
    ROS_INFO("receiving trans distance: %f; dx: %f, dy: %f, dz: %f", trans.desired_distance, direction.vector.x, direction.vector.y, direction.vector.z);
    float norm = sqrt((direction.vector.x * direction.vector.x) +(direction.vector.y * direction.vector.y) +(direction.vector.z * direction.vector.z));
    if(norm != 0){
        direction.vector.x /= norm;
        direction.vector.y /= norm;
        direction.vector.z /= norm;
    }else{
        ROS_INFO("Norm is ZERO!");
        direction.vector.x = 0 ;
        direction.vector.y = -1;
        direction.vector.z = 0 ;
    }

    direction.vector.x *= -trans.desired_distance;
    direction.vector.y *= -trans.desired_distance;
    direction.vector.z *= -trans.desired_distance;

    ROS_INFO("setting trans; dx: %f, dy: %f, dz: %f", direction.vector.x, direction.vector.y, direction.vector.z);

    template_T_hand.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    template_T_hand.setOrigin(tf::Vector3(0,0,0));

    vec_in.setOrigin(tf::Vector3(direction.vector.x,direction.vector.y,direction.vector.z));
    vec_in.setRotation(tf::Quaternion(0,0,0,1));

    vec_out = template_T_hand * vec_in;

    ROS_INFO("setting result; dx: %f, dy: %f, dz: %f", vec_out.getOrigin().getX(), vec_out.getOrigin().getY(), vec_out.getOrigin().getZ());

    pose.position.x += vec_out.getOrigin().getX();
    pose.position.y += vec_out.getOrigin().getY();
    pose.position.z += vec_out.getOrigin().getZ();
}

bool ObjectTemplateServer::templateInfoSrv(vigir_object_template_msgs::GetTemplateInfo::Request& req,
                                      vigir_object_template_msgs::GetTemplateInfo::Response& res)
{
    boost::recursive_mutex::scoped_lock lock_template_list(template_list_mutex_);

    /*Fill in the blanks of the response "res"
     * with the info of the template id in the request "req"
    */
    ROS_INFO("Template Info Service called for type: %d", req.template_type);

    if(object_template_map_.size() > 0){

        res.template_type_information.template_type  = object_template_map_[req.template_type].type;
        res.template_type_information.type_name      = object_template_map_[req.template_type].name;
        res.template_type_information.mass           = object_template_map_[req.template_type].mass;
        res.template_type_information.center_of_mass = object_template_map_[req.template_type].com;
        res.template_type_information.b_min          = object_template_map_[req.template_type].b_min;
        res.template_type_information.b_max          = object_template_map_[req.template_type].b_max;

        for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it2  = object_template_map_[req.template_type].grasps.begin();
                                                                 it2 != object_template_map_[req.template_type].grasps.end();
                                                                 ++it2){
            res.template_type_information.grasps.push_back(it2->second);
            ROS_INFO("Filling grasp ID: %s", it2->second.id.c_str());
        }
        for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it2  = object_template_map_[req.template_type].stand_poses.begin();
                                                                         it2 != object_template_map_[req.template_type].stand_poses.end();
                                                                         ++it2){
            res.template_type_information.stand_poses.push_back(it2->second);
            ROS_INFO("Filling stand pose ID: %d", it2->second.id);
        }
        for (std::map<unsigned int,vigir_object_template_msgs::Affordance>::iterator it2  = object_template_map_[req.template_type].affordances.begin();
                                                                         it2 != object_template_map_[req.template_type].affordances.end();
                                                                         ++it2){
            res.template_type_information.affordances.push_back(it2->second);
            ROS_INFO("Filling affordance ID: %d", it2->second.id);
        }
        for (std::map<unsigned int,vigir_object_template_msgs::Usability>::iterator it2  = object_template_map_[req.template_type].usabilities.begin();
                                                                         it2 != object_template_map_[req.template_type].usabilities.end();
                                                                         ++it2){
            res.template_type_information.usabilities.push_back(it2->second);
            ROS_INFO("Filling usability ID: %d", it2->second.id);
        }
        return true;
    }
    ROS_ERROR("OBJECT LIBRARY IS EMPTY!");
    return false;
}

bool ObjectTemplateServer::graspInfoSrv(vigir_object_template_msgs::GetGraspInfo::Request& req,
                                   vigir_object_template_msgs::GetGraspInfo::Response& res)
{
    boost::recursive_mutex::scoped_lock lock(object_template_map_mutex_);

    /*Fill in the blanks of the response "res"
     * with the info of the template id in the request "req"
    */
    ROS_INFO("Executing service to get grasp info from: %s", req.grasp_id.c_str());
    if(object_template_map_.size() > 0){
        for (std::map<unsigned int,moveit_msgs::Grasp>::iterator it2  = object_template_map_[req.template_type].grasps.begin();
                                                                 it2 != object_template_map_[req.template_type].grasps.end();
                                                                 ++it2){
          if((it2->second).id == req.grasp_id){
            res.grasp_information.grasps.push_back(it2->second);
          }
        }
        for (std::map<unsigned int,vigir_object_template_msgs::StandPose>::iterator it2  = object_template_map_[req.template_type].stand_poses.begin();
                                                                         it2 != object_template_map_[req.template_type].stand_poses.end();
                                                                         ++it2){
            res.grasp_information.stand_poses.push_back(it2->second);
        }
        return true;
    }
    return false;
}


int ObjectTemplateServer::worldPoseTransform(const geometry_msgs::PoseStamped& template_pose,const geometry_msgs::Pose& input_pose, geometry_msgs::PoseStamped& target_pose)
{
    tf::Transform template_T_wrist;
    tf::Transform world_T_template;
    tf::Transform world_T_wrist;

    world_T_template.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    world_T_template.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );
    template_T_wrist.setRotation(tf::Quaternion(input_pose.orientation.x,input_pose.orientation.y,input_pose.orientation.z,input_pose.orientation.w));
    template_T_wrist.setOrigin(tf::Vector3(input_pose.position.x,input_pose.position.y,input_pose.position.z) );

    world_T_wrist = world_T_template * template_T_wrist;

    tf::Quaternion tg_quat;
    tf::Vector3    tg_vector;
    tg_quat   = world_T_wrist.getRotation();
    tg_vector = world_T_wrist.getOrigin();

    target_pose.pose.orientation.w = tg_quat.getW();
    target_pose.pose.orientation.x = tg_quat.getX();
    target_pose.pose.orientation.y = tg_quat.getY();
    target_pose.pose.orientation.z = tg_quat.getZ();

    target_pose.pose.position.x = tg_vector.getX();
    target_pose.pose.position.y = tg_vector.getY();
    target_pose.pose.position.z = tg_vector.getZ();

    target_pose.header.frame_id = "/world";
    target_pose.header.stamp    = template_pose.header.stamp;
    return 0;
}

int ObjectTemplateServer::poseTransform(geometry_msgs::Pose& first_pose, geometry_msgs::Pose& second_pose, geometry_msgs::Pose& target_pose)
{
    tf::Transform output_transform;
    tf::Transform first_transform;
    tf::Transform second_transform;

    first_transform.setRotation(tf::Quaternion(first_pose.orientation.x,first_pose.orientation.y,first_pose.orientation.z,first_pose.orientation.w));
    first_transform.setOrigin(tf::Vector3(first_pose.position.x,first_pose.position.y,first_pose.position.z) );

    second_transform.setRotation(tf::Quaternion(second_pose.orientation.x,second_pose.orientation.y,second_pose.orientation.z,second_pose.orientation.w));
    second_transform.setOrigin(tf::Vector3(second_pose.position.x,second_pose.position.y,second_pose.position.z) );

    output_transform = first_transform * second_transform;

    tf::Quaternion output_quat;
    tf::Vector3    output_vector;
    output_quat   = output_transform.getRotation();
    output_vector = output_transform.getOrigin();

    target_pose.position.x    = output_vector.getX();
    target_pose.position.y    = output_vector.getY();
    target_pose.position.z    = output_vector.getZ();
    target_pose.orientation.x = output_quat.getX();
    target_pose.orientation.y = output_quat.getY();
    target_pose.orientation.z = output_quat.getZ();
    target_pose.orientation.w = output_quat.getW();
    return 0;
}

// transform endeffort to palm pose used by GraspIt
int ObjectTemplateServer::staticTransform(geometry_msgs::Pose& palm_pose, tf::Transform gp_T_hand)
{
    tf::Transform o_T_hand;    //describes hand in object's frame
    tf::Transform o_T_pg;       //describes palm_from_graspit in object's frame

    o_T_pg.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    o_T_pg.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

    o_T_hand = o_T_pg * gp_T_hand;

    tf::Quaternion hand_quat;
    tf::Vector3    hand_vector;
    hand_quat   = o_T_hand.getRotation();
    hand_vector = o_T_hand.getOrigin();

    palm_pose.position.x    = hand_vector.getX();
    palm_pose.position.y    = hand_vector.getY();
    palm_pose.position.z    = hand_vector.getZ();
    palm_pose.orientation.x = hand_quat.getX();
    palm_pose.orientation.y = hand_quat.getY();
    palm_pose.orientation.z = hand_quat.getZ();
    palm_pose.orientation.w = hand_quat.getW();
    return 0;
}


void ObjectTemplateServer::updateStandPose(vigir_object_template_msgs::TemplateStandPoseUpdate::ConstPtr msg)
{
    boost::recursive_mutex::scoped_lock lock_object_template_map(object_template_map_mutex_);
    VigirObjectTemplateMapIter iter = object_template_map_.find(msg->template_type);
    if(iter == object_template_map_.end())
    {
        return;
    }
    std::map<unsigned int, vigir_object_template_msgs::StandPose>::iterator g_iter = iter->second.stand_poses.find(msg->pose.id);
    if(g_iter == iter->second.stand_poses.end())
    {
        return;
    }

    g_iter->second = msg->pose;

}

void ObjectTemplateServer::updateAffordance(vigir_object_template_msgs::TemplateAffordanceUpdate::ConstPtr msg)
{
    boost::recursive_mutex::scoped_lock lock_object_template_map(object_template_map_mutex_);
    VigirObjectTemplateMapIter iter = object_template_map_.find(msg->template_type);
    if(iter == object_template_map_.end())
    {
        return;
    }
    std::map<unsigned int, vigir_object_template_msgs::Affordance>::iterator g_iter = iter->second.affordances.find(msg->affordance.id);
    if(g_iter == iter->second.affordances.end())
    {
        return;
    }

    g_iter->second = msg->affordance;

}

void ObjectTemplateServer::updateGrasp(vigir_object_template_msgs::TemplateGraspUpdate::ConstPtr msg)
{
    boost::recursive_mutex::scoped_lock lock_object_template_map(object_template_map_mutex_);
    VigirObjectTemplateMapIter iter = object_template_map_.find(msg->template_type);
    if(iter == object_template_map_.end())
    {
        return;
    }
    std::map<unsigned int, moveit_msgs::Grasp>::iterator g_iter = iter->second.grasps.find(std::atoi(msg->grasp.id.c_str()));
    if(g_iter == iter->second.grasps.end())
    {
        return;
    }

    g_iter->second = msg->grasp;

}

}

PLUGINLIB_DECLARE_CLASS (vigir_object_template_server, ObjectTemplateServer, object_template_server::ObjectTemplateServer, nodelet::Nodelet);
