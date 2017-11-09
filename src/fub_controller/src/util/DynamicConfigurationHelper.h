#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>

namespace fub {
namespace controller {
namespace util {

/** A helper class to use dynamic_reconfigure. By inheriting from this class
 ** and calling setupConfiguration, a dynamic_reconfigure server and callback
 ** will be setup, and the latest configuration stored in mConfig.
 **
 ** @tparam ConfigClass  The dynamic_reconfigure configuration class
 **
 ** @ingroup @@
 */
template<class ConfigClass>
class DynamicConfigurationHelper
{
protected:
    /** Sets up the configuration helper.
     **
     ** @param name               Name of the configuration
     ** @param privateNodeHandle  Node handle with private namespace
     */
    virtual void setupConfiguration(std::string const & name, ros::NodeHandle & privateNodeHandle)
    {
        // create nodehandle with given sub-namespace
        mNh = ros::NodeHandle(privateNodeHandle.getNamespace() + "/" + name);

        // configure configuration server
        mConfigServer = boost::make_shared<dynamic_reconfigure::Server<ConfigClass> >(mNh);
        typename dynamic_reconfigure::Server<ConfigClass>::CallbackType f;
        f = boost::bind(&DynamicConfigurationHelper::configurationCallback, this, _1, _2);
        mConfigServer->setCallback(f);
    }

    /** Callback for updated configuration parameters via dynamic_reconfigure.
     **
     ** @param config  The configuration structure with the updated values
     ** @param level   The OR-ed value of all parameter level values
     */
    virtual void configurationCallback(ConfigClass &config, uint32_t level)
    {
        mConfig = config;
    }

protected:
    /// dynamic reconfigure service
    boost::shared_ptr<dynamic_reconfigure::Server<ConfigClass> > mConfigServer;

    ConfigClass mConfig;

private:
    ros::NodeHandle mNh;
};

}
}
}
