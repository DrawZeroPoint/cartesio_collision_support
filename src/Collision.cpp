#include "Collision.h"
#include <boost/make_shared.hpp>
#include <utility>
#include <OpenSoT/utils/collision_utils.h>


using namespace XBot::Cartesian;
using namespace XBot::Cartesian::collision;

const int default_max_pairs = 15;

namespace
{
  int get_size(YAML::Node node)
  {
    if (auto n = node["max_pairs"]) {
      return n.as<int>();
    }

    if(auto n = node["pairs"]) {
      return n.size();
    }

    return default_max_pairs;
  }
}

CollisionTaskImpl::CollisionTaskImpl(YAML::Node node, const Context::ConstPtr& context):
    TaskDescriptionImpl(node, context, "collision_avoidance", get_size(node)),
    bound_scaling_(1.0),
    min_dist_(0.001)  // note: smth > 0 to avoid singular min distance segment
{

  if (auto n = node["pairs"]) {
    for(auto p : n) {
      pairs_.push_back(p.as<std::pair<std::string, std::string>>());
    }
  }

  if (auto n = node["robot_collision_links"]) {
    for (auto p : n) {
      robot_links_.push_back(p.as<std::string>());
    }
  }

  if (auto n = node["env_collision_links"]) {
    for (auto p : n) {
      env_links_.push_back(p.as<std::string>());
    }
  }

  if (auto n = node["bound_scaling"]) {
    bound_scaling_ = n.as<double>();
  }

  if(auto n = node["distance_threshold"]) {
    min_dist_ = n.as<double>();
  }

  if (auto n = node["collision_urdf_path"]) {
    // parse path via shell
    auto urdf_path = XBot::Utils::computeAbsolutePathShell(n.as<std::string>());

    // construct model shared ptr
    auto urdf_mdl = new urdf::Model;
    urdf_model_ptr_.reset(urdf_mdl);

    // if could not init, destroy it
    if (!urdf_mdl->initFile(urdf_path)) {
      Logger::error("Could not load collision urdf from file '%s'\n",
                    urdf_path.c_str());
      urdf_model_ptr_.reset();
    }
  }

  if (auto n = node["collision_srdf_path"]) {
    // parse path via shell
    auto srdf_path = XBot::Utils::computeAbsolutePathShell(n.as<std::string>());

    // construct model shared ptr
    auto srdf_mdl = new srdf::Model;
    srdf_model_ptr_.reset(srdf_mdl);

    // get urdf either from collision or from model interface
    const urdf::ModelInterface* urdf = urdf_model_ptr_.get();

    if (!urdf) {
      urdf = &(context->model()->getUrdf());
    }

    // if cannot init, destroy
    if (!srdf_mdl->initFile(*urdf, srdf_path)) {
      Logger::error("Could not load collision srdf from file '%s'\n",
                    srdf_path.c_str());
      srdf_model_ptr_.reset();
    }
  }
}

void CollisionTaskImpl::setLinkPairDistances(const std::list<LinkPairDistance>& distance_list)
{
  link_pair_distance_list_ = distance_list;
}

const std::list<LinkPairDistance>& CollisionTaskImpl::getLinkPairDistances()
{
  return link_pair_distance_list_;
}

bool CollisionTaskImpl::validate()
{
  return bound_scaling_ <= 1.0 &&
         bound_scaling_ > 0 &&
         min_dist_ >= 0.0;
}

double CollisionTaskImpl::getBoundScaling() const
{
  return bound_scaling_;
}

double CollisionTaskImpl::getDistanceThreshold() const
{
  return min_dist_;
}

std::list<std::pair<std::string, std::string> > CollisionTaskImpl::getWhiteList() const
{
  return pairs_;
}

std::vector<std::string> CollisionTaskImpl::getRobotWhiteList() const
{
  return robot_links_;
}

std::list<std::string> CollisionTaskImpl::getEnvironmentWhiteList() const
{
  return env_links_;
}

urdf::ModelConstSharedPtr CollisionTaskImpl::getCollisionUrdf() const
{
  return urdf_model_ptr_;
}

srdf::ModelConstSharedPtr CollisionTaskImpl::getCollisionSrdf() const
{
  return srdf_model_ptr_;
}

void CollisionTaskImpl::registerWorldUpdateCallback(WorldUpdateCallback f)
{
  _world_upd_cb.push_back(f);
}

void CollisionTaskImpl::worldUpdated(const moveit_msgs::PlanningSceneWorld& psw)
{
  for(auto& fn : _world_upd_cb) {
    fn(psw);
  }
}

OpenSotCollisionConstraintAdapter::OpenSotCollisionConstraintAdapter(const ConstraintDescription::Ptr& ci_task,
                                                                     Context::ConstPtr context):
    OpenSotConstraintAdapter(ci_task, std::move(context))
{
  ci_collision_task_ = std::dynamic_pointer_cast<CollisionTaskImpl>(ci_task);
  if(!ci_collision_task_) throw std::runtime_error("Provided task description "
                                                   "does not have expected type 'CollisionTask'");
}

OpenSoT::OptvarHelper::VariableVector OpenSotCollisionConstraintAdapter::getRequiredVariables() const
{
  return {};
}

ConstraintPtr OpenSotCollisionConstraintAdapter::constructConstraint()
{
  Eigen::VectorXd q;
  _model->getJointPosition(q);

  std::string base_link = "base_link";
  std::map<std::string, boost::shared_ptr<fcl::CollisionObjectd>> env_collisions;

  //computer_ = boost::make_shared<ComputeLinksDistance>(const_cast<XBot::ModelInterface&>(*_model));

  opensot_collision_ptr_ = boost::make_shared<CollisionConstrSoT>(
      q,
      const_cast<XBot::ModelInterface&>(*_model),
      base_link,
      ci_collision_task_->getRobotWhiteList(),
      env_collisions
  );

  // set parameters
  opensot_collision_ptr_->setBoundScaling(ci_collision_task_->getBoundScaling());
  opensot_collision_ptr_->setLinkPairThreshold(ci_collision_task_->getDistanceThreshold());
  opensot_collision_ptr_->setDetectionThreshold(0.05);  // hardcoded!

  // set whitelist if available
  //auto whitelist = ci_collision_task_->getWhiteList();
  //if (!whitelist.empty()) {
  //opensot_collision_ptr_->setCollisionWhiteList(whitelist);
  //}

  // set link-env collisions
  //auto env_whitelist = ci_collision_task_->getEnvironmentWhiteList();
  //if (!env_whitelist.empty()) {
  //  opensot_collision_ptr_->setLinksVsEnvironment(env_whitelist);
  //}

  // register world update function
  //auto on_world_upd = [this](const moveit_msgs::PlanningSceneWorld& psw)
  //{
  //  opensot_collision_ptr_->setWorldCollisions(psw);
  //};
  //ci_collision_task_->registerWorldUpdateCallback(on_world_upd);

  return opensot_collision_ptr_;
}

void OpenSotCollisionConstraintAdapter::update(double time, double period)
{
  OpenSotConstraintAdapter::update(time, period);

  ci_collision_task_->setLinkPairDistances(opensot_collision_ptr_->getLinkDistances(0.05));
}

void OpenSotCollisionConstraintAdapter::processSolution(const Eigen::VectorXd &solution)
{

}


CollisionRos::CollisionRos(const TaskDescription::Ptr& task, const RosContext::Ptr& context):
    TaskRos(task, context)
{
  ci_collision_task_ = std::dynamic_pointer_cast<CollisionTaskImpl>(task);

  if (!ci_collision_task_) throw std::runtime_error(
        "Provided task description does not have expected type 'CollisionTask'"
    );

  auto nh = ros::NodeHandle(context->nh().getNamespace() + "/" + task->getName());

  _ps = std::make_unique<Planning::PlanningSceneWrapper>(ci_collision_task_->getModel(),
                                                         ci_collision_task_->getCollisionUrdf(),
                                                         ci_collision_task_->getCollisionSrdf(),
                                                         nh);
  _ps->startGetPlanningSceneServer();
  _ps->startMonitor();


  _visualize_distances = nh.param("visualize_distances", true);

  _world_upd_srv = nh.advertiseService("apply_planning_scene",
                                       &CollisionRos::apply_planning_scene_service,
                                       this);

  registerType("Collision");

  _vis_pub = nh.advertise<visualization_msgs::Marker>( "collision_distances", 0 );

}

void CollisionRos::setVisualizeDistances(const bool flag)
{
  _visualize_distances = flag;
}

bool CollisionRos::apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request &req,
                                                moveit_msgs::ApplyPlanningScene::Response &res)
{
  // for visualization purposes (e.g. rviz/PlanningScene)
  _ps->applyPlanningScene(req.scene);

  // notify collision avoidance constraint that
  // world geometry has changed
  ci_collision_task_->worldUpdated(req.scene.world);

  res.success = true;

  return true;
}

void XBot::Cartesian::collision::CollisionRos::run(ros::Time time)
{
  _ps->update();
  if(_visualize_distances)
  {
    std::list<LinkPairDistance> distance_list = ci_collision_task_->getLinkPairDistances();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "ci/world";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    marker.color.a = 1.;
    marker.scale.x = 0.005;
    marker.scale.y = 0.;
    marker.scale.z = 0.;

    for (const auto& data : distance_list) {
      auto k2p = [](const KDL::Vector &k)->geometry_msgs::Point{
        geometry_msgs::Point p;
        p.x = k[0]; p.y = k[1]; p.z = k[2];
        return p;
      };

      // closest point on first link
      marker.points.push_back(k2p(data.getLink_T_closestPoint().first.p));
      // closest point on second link
      marker.points.push_back(k2p(data.getLink_T_closestPoint().second.p));
    }
    _vis_pub.publish(marker);
  }
}


CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionConstraint)
CARTESIO_REGISTER_TASK_PLUGIN(CollisionTaskImpl, CollisionTask)
CARTESIO_REGISTER_ROS_API_PLUGIN(CollisionRos, CollisionConstraint)
CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(OpenSotCollisionConstraintAdapter, CollisionConstraint)


