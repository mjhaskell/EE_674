#include "path_manager/pathmanager.hpp"

PathManager::PathManager() :
    m_nh{ros::NodeHandle()},
    m_nh_private{"~"}
{

}

PathManager::~PathManager()
{
}

