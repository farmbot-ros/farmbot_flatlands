#ifndef RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
#define RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreColourValue.h>
#include <OgreVector3.h>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <rclcpp/rclcpp.hpp>

#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool.hpp>

#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/objects/line.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace rviz_polygon_tool {

// Declare polygon tool as subclass of rviz_common::Tool.
class PolygonTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PolygonTool();
  virtual ~PolygonTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;


private:
  // User input.
  void clickLeft(const rviz_common::ViewportMouseEvent& event);
  void clickRight(const rviz_common::ViewportMouseEvent& event);

  // Action.
  void createVertex(const Ogre::Vector3& position);
  void deleteVertex(const Ogre::Vector3& position);
  void nextVertex();
  void clearAll();
  void publishPolygon();

  geometry_msgs::msg::Polygon polygon_;
  std::vector<geometry_msgs::msg::Point32>::iterator vertex_selection_;
  double altitude_;

  // Rendering
  void renderPolygon();
  Ogre::SceneNode* polygon_node_;

  // Sphere currently displayed.
  Ogre::SceneNode* moving_vertex_node_;
  rviz_rendering::Shape* sphere_;

  // ROS messaging
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
};

}  // namespace rviz_polygon_tool

#endif  // RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
