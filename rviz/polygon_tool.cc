// Include the header file for the PolygonTool class.
#include "polygon_tool.h"

// Include OgrePlane for defining planes in 3D space.
#include <OgrePlane.h>

// Include various RViz common headers for display context, rendering panels, and mouse events.
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

// Include RViz rendering utilities for geometry operations.
#include <rviz_rendering/geometry.hpp>

// Include pluginlib macros for exporting the class as a plugin.
#include <pluginlib/class_list_macros.hpp>

// Include Qt's key event handling.
#include <QKeyEvent>

// Define the namespace for the PolygonTool to avoid naming conflicts.
namespace rviz_polygon_tool {

// Define constant colors using Ogre's ColourValue.
// These are used for rendering different parts of the polygon.
const Ogre::ColourValue kRed = Ogre::ColourValue(1.f, 0.f, 0.f, 1.0);        // Fully opaque red.
const Ogre::ColourValue kGreen = Ogre::ColourValue(0.f, 1.f, 0.f, 1.0);      // Fully opaque green.
const Ogre::ColourValue kBlue = Ogre::ColourValue(0.f, 0.f, 1.f, 1.0);       // Fully opaque blue.
const Ogre::ColourValue kPink = Ogre::ColourValue(1.f, 0.f, 1.f, 1.0);       // Fully opaque pink.
const Ogre::ColourValue kYellow = Ogre::ColourValue(1.f, 1.f, 0.f, 1.0);     // Fully opaque yellow.
const Ogre::ColourValue kTransparent = Ogre::ColourValue(0.f, 0.f, 0.f, 0.0); // Fully transparent.

// Define constant scales for points and deletion tolerance.
const float kPtScale = 0.5;    // Scale for rendering points (vertices).
const float kDeleteTol = 0.5;  // Tolerance distance for deleting a vertex.

// Define default altitude for the polygon in the Z-axis.
const double kDefaultAltitude = 0.0;

// Constructor for the PolygonTool class.
PolygonTool::PolygonTool()
    : Tool(), // Initialize the base Tool class.
      polygon_(geometry_msgs::msg::Polygon()), // Initialize an empty polygon message.
      vertex_selection_(polygon_.points.begin()), // Initialize vertex selection iterator.
      altitude_(kDefaultAltitude), // Set initial altitude.
      polygon_node_(nullptr), // Initialize polygon scene node to null.
      moving_vertex_node_(nullptr), // Initialize moving vertex scene node to null.
      sphere_(nullptr) // Initialize sphere shape to null.
{
  shortcut_key_ = 'p'; // Set the shortcut key for activating the tool to 'p'.
}

// Destructor for the PolygonTool class.
PolygonTool::~PolygonTool() {}

// Initialization function called when the tool is initialized.
void PolygonTool::onInitialize()
{
  // Create a child scene node for moving vertices under the root scene node.
  moving_vertex_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create a spherical shape to represent the moving vertex.
  sphere_ = new rviz_rendering::Shape(rviz_rendering::Shape::Sphere, scene_manager_, moving_vertex_node_);

  // Initially hide the moving vertex node.
  moving_vertex_node_->setVisible(false);

  // Create a child scene node for the polygon under the root scene node.
  polygon_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Make the polygon node visible.
  polygon_node_->setVisible(true);

  // Obtain the raw ROS node from the display context.
  auto raw_node = context_->getRosNodeAbstraction().lock();

  // Check if the ROS node was successfully obtained.
  if (!raw_node)
  {
    throw std::runtime_error("Failed to lock node"); // Throw an error if the node couldn't be locked.
  }

  // Get the underlying raw ROS node.
  auto node = raw_node->get_raw_node();

  // Create a publisher for publishing the polygon data on the "polygon" topic with a queue size of 10.
  polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("polygon", 10);
}

// Activate the tool, making relevant nodes visible.
void PolygonTool::activate()
{
  // If the moving vertex node exists, make it visible.
  if (moving_vertex_node_)
  {
    moving_vertex_node_->setVisible(true);
  }

  // If the polygon node exists, make it visible.
  if (polygon_node_)
  {
    polygon_node_->setVisible(true);
  }
}

// Deactivate the tool, hiding relevant nodes.
void PolygonTool::deactivate()
{
  // If the moving vertex node exists, hide it.
  if (moving_vertex_node_)
  {
    moving_vertex_node_->setVisible(false);
  }

  // If the polygon node exists, hide it.
  if (polygon_node_)
  {
    polygon_node_->setVisible(false);
  }
}

// Handle mouse events within the viewport.
int PolygonTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
  // If there's no moving vertex node, request a render.
  if (!moving_vertex_node_)
  {
    return Render;
  }

  Ogre::Vector3 intersection; // Variable to store the intersection point.

  // Define a plane for the polygon on the XY plane (Z=0).
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  // Attempt to get the intersection point on the plane from the mouse position.
  if (rviz_rendering::getPointOnPlaneFromWindowXY(
        event.panel->getRenderWindow(), polygon_plane, event.x, event.y, intersection))
  {
    // Hide the moving vertex node while processing.
    moving_vertex_node_->setVisible(false);

    // Set the position of the moving vertex node to the intersection point.
    moving_vertex_node_->setPosition(intersection);

    // If the left mouse button was released, handle a left click.
    if (event.leftUp())
    {
      clickLeft(event);
    }
    // If the right mouse button was released, handle a right click.
    else if (event.rightUp())
    {
      clickRight(event);
    }
  }
  else
  {
    // If the mouse is not over the plane, hide the moving vertex node.
    moving_vertex_node_->setVisible(false);
  }

  return Render; // Request a render after processing the event.
}

// Handle left mouse button clicks.
void PolygonTool::clickLeft(const rviz_common::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection; // Variable to store the intersection point.

  // Define a plane for the polygon on the XY plane (Z=0).
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  // Attempt to get the intersection point on the plane from the mouse position.
  if (rviz_rendering::getPointOnPlaneFromWindowXY(
        event.panel->getRenderWindow(), polygon_plane, event.x, event.y, intersection))
  {
    createVertex(intersection); // Create a new vertex at the intersection point.
  }

  renderPolygon(); // Update the polygon visualization.
}

// Handle right mouse button clicks.
void PolygonTool::clickRight(const rviz_common::ViewportMouseEvent& event)
{
  Ogre::Vector3 intersection; // Variable to store the intersection point.

  // Define a plane for the polygon on the XY plane (Z=0).
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  // Attempt to get the intersection point on the plane from the mouse position.
  if (rviz_rendering::getPointOnPlaneFromWindowXY(
        event.panel->getRenderWindow(), polygon_plane, event.x, event.y, intersection))
  {
    deleteVertex(intersection); // Attempt to delete a vertex near the intersection point.
  }

  renderPolygon(); // Update the polygon visualization.
}

// Create a new vertex at the specified position.
void PolygonTool::createVertex(const Ogre::Vector3& position)
{
  // Create a new Point32 message for the vertex.
  geometry_msgs::msg::Point32 new_point;
  new_point.x = position.x; // Set the X coordinate.
  new_point.y = position.y; // Set the Y coordinate.
  // Note: Z coordinate is not set as it's assumed to be on the XY plane.

  // Insert the new point into the polygon at the current vertex selection.
  vertex_selection_ = polygon_.points.insert(vertex_selection_, new_point);
}

// Delete a vertex near the specified position.
void PolygonTool::deleteVertex(const Ogre::Vector3& position)
{
  // Iterate through all vertices in the polygon.
  for (auto v = polygon_.points.begin(); v != polygon_.points.end(); ++v)
  {
    // Create a Vector3 for the vertex position (Z=0).
    Ogre::Vector3 pt(static_cast<double>(v->x), static_cast<double>(v->y), 0.0);

    // Check if the vertex is within the deletion tolerance distance.
    if ((position - pt).length() < kDeleteTol)
    {
      vertex_selection_ = v; // Select the vertex to be deleted.
      vertex_selection_ = polygon_.points.erase(v); // Erase the vertex from the polygon.

      // If the iterator is at the end, wrap around to the beginning.
      if (vertex_selection_ == polygon_.points.end())
        vertex_selection_ = polygon_.points.begin();

      break; // Exit the loop after deleting one vertex.
    }
  }
}

// Render the polygon by visualizing its vertices and edges.
void PolygonTool::renderPolygon()
{
  // If there's no polygon node, exit the function.
  if (!polygon_node_)
  {
    return;
  }

  // Remove all existing children from the polygon node to clear previous visualizations.
  polygon_node_->removeAllChildren();  // Clear polygon visualization.

  const Ogre::ColourValue& c = kBlue; // Set the default color for rendering.

  // Iterate through all vertices in the polygon.
  for (auto v = polygon_.points.begin(); v != polygon_.points.end(); ++v)
  {
    // Render each vertex as a sphere.
    rviz_rendering::Shape* sphere = new rviz_rendering::Shape(rviz_rendering::Shape::Sphere, scene_manager_, polygon_node_);
    sphere->setColor(c); // Set the color of the sphere.
    sphere->setScale(Ogre::Vector3(kPtScale)); // Set the scale of the sphere.

    // Set the position of the sphere based on the vertex coordinates and altitude.
    Ogre::Vector3 p(static_cast<double>(v->x), static_cast<double>(v->y), altitude_);
    sphere->setPosition(p);

    // If this vertex is currently selected, change its color to green.
    if (v == vertex_selection_) sphere->setColor(kGreen);

    // Render edges between consecutive vertices as lines.
    if (polygon_.points.size() < 2) continue; // Skip if there are fewer than 2 points.

    // Determine the previous vertex to create an edge.
    auto v_prev = v == polygon_.points.begin() ? std::prev(polygon_.points.end()) : std::prev(v);

    // Get the starting point of the line.
    Ogre::Vector3 start(static_cast<double>(v_prev->x), static_cast<double>(v_prev->y), altitude_);

    // Create a new line between the previous vertex and the current vertex.
    rviz_rendering::Line* line = new rviz_rendering::Line(scene_manager_, polygon_node_);
    line->setColor(c); // Set the color of the line.
    line->setPoints(start, p); // Define the start and end points of the line.

    // If this vertex is currently selected, change the line color to green.
    if (v == vertex_selection_) line->setColor(kGreen);
  }
}

// Handle key events within the render panel.
int PolygonTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel)
{
  (void)panel; // Suppress unused parameter warning.

  // If the 'v' key is pressed, move to the next vertex.
  if (event->text() == "v")
  {
    nextVertex();
  }
  // If the 'c' key is pressed, clear all vertices.
  else if (event->text() == "c")
  {
    clearAll();
  }
  // If the Enter or Return key is pressed, publish the polygon.
  else if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
  {
    publishPolygon();
  }

  renderPolygon(); // Update the polygon visualization after handling the key event.
  return Render; // Request a render.
}

// Move the vertex selection to the next vertex in the polygon.
void PolygonTool::nextVertex()
{
  vertex_selection_ = std::next(vertex_selection_); // Move to the next vertex.

  // If the iterator reaches the end, wrap around to the beginning.
  if (vertex_selection_ == polygon_.points.end())
    vertex_selection_ = polygon_.points.begin();
}

// Clear all vertices from the polygon and reset selections.
void PolygonTool::clearAll()
{
  polygon_ = geometry_msgs::msg::Polygon(); // Reset the polygon to an empty state.
  vertex_selection_ = polygon_.points.begin(); // Reset the vertex selection iterator.
  altitude_ = kDefaultAltitude; // Reset the altitude to default.
}

// Publish the current polygon to the ROS topic.
void PolygonTool::publishPolygon()
{
  // Create a PolygonStamped message.
  geometry_msgs::msg::PolygonStamped msg;

  msg.header.stamp = rclcpp::Clock().now(); // Set the current time as the timestamp.
  msg.header.frame_id = context_->getFixedFrame().toStdString(); // Set the frame ID to the fixed frame.
  msg.polygon = polygon_; // Assign the polygon data.

  polygon_pub_->publish(msg); // Publish the message to the "polygon" topic.

  // Log an informational message indicating that the polygon has been published.
  RCLCPP_INFO(rclcpp::get_logger("PolygonTool"), "Publishing polygon");
}

}  // namespace rviz_polygon_tool

// Export the PolygonTool class as a plugin so that RViz can recognize and load it.
PLUGINLIB_EXPORT_CLASS(rviz_polygon_tool::PolygonTool, rviz_common::Tool)
