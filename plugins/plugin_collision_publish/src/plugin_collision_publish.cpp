#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <std_msgs/String.h>  // Include the ROS message header

namespace gazebo {

class ContactPlugin : public WorldPlugin {
public:
    ContactPlugin() : WorldPlugin() {}

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
       
        // Initialize the plugin
        std::cout << "Collision Info Plugin Started " << std::endl;

        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "gz_collision_info", ros::init_options::NoSigintHandler);
        }

        // Create a ROS node handle
        this->rosNode.reset(new ros::NodeHandle());

        // Create a Gazebo transport node
        this->gzNode = transport::NodePtr(new transport::Node());
        this->gzNode->Init();

        // Subscribe to the Gazebo Contacts topic
        this->contactSub = this->gzNode->Subscribe("~/physics/contacts", &ContactPlugin::OnContacts, this);

        // Create the ROS publisher
        this->collisionPub = this->rosNode->advertise<std_msgs::String>("/gazebo/collision/info", 10);
    }

    // Callback function for incoming contact messages
    void OnContacts(ConstContactsPtr &contactsMsg) {
        for (int i = 0; i < contactsMsg->contact_size(); ++i) {
            const msgs::Contact &contact = contactsMsg->contact(i);

            // Extract model and link names from collision names
            std::string modelName1 = GetModelNameFromLink(contact.collision1());
            std::string modelName2 = GetModelNameFromLink(contact.collision2());

           
            // Publish collision information
            std_msgs::String collisionInfoMsg;
            collisionInfoMsg.data = modelName1 + "-" + modelName2;
            this->collisionPub.publish(collisionInfoMsg);
        }
    }

private:
    ros::NodeHandlePtr rosNode;
    transport::NodePtr gzNode;
    transport::SubscriberPtr contactSub;
    ros::Publisher collisionPub;

    std::string GetModelNameFromLink(const std::string &linkName) {
        size_t lastLink = linkName.rfind("::link");
        if (lastLink != std::string::npos) {
            return linkName.substr(0, lastLink);
        }
        return linkName;
    }
};

GZ_REGISTER_WORLD_PLUGIN(ContactPlugin)

}  // namespace gazebo

