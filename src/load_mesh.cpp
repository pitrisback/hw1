#include "ros/ros.h"
#include "std_msgs/String.h"
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/Mesh.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
// #include <geometric_shapes> // this includes all

int main(int argc, char** argv)
{
    // https://answers.ros.org/question/172720/problem-loading-mesh-with-createmeshfromresource/
    // http://docs.ros.org/hydro/api/geometric_shapes/html/classshapes_1_1Mesh.html
    
    std::string mesh_file = "file:///home/ros/ros_ws/src/internal/ar_arena/meshes/hexagon.dae";
    // std::string mesh_file = "file:///home/ros/ros_ws/src/internal/ar_arena/meshes/octagon_big.dae";
    shapes::Mesh* m = shapes::createMeshFromResource(mesh_file);
    ROS_INFO("Loaded mesh with %d vertex_count", m->vertex_count);
    ROS_INFO("Loaded mesh with %d triangle_count", m->triangle_count);

    for(uint i = 0; i < m->triangle_count; i++) {
        ROS_INFO("Triangle %d : ( %d, %d, %d ),",
                i,
                m->triangles[3*i],
                m->triangles[3*i+1],
                m->triangles[3*i+2]
            );
    }
    for(uint i = 0; i < m->vertex_count; i++) {
        ROS_INFO("Vertex %d : ( %f, %f, %f ),",
                i,
                m->vertices[3*i],
                m->vertices[3*i+1],
                m->vertices[3*i+2]
            );
    }
    // moveit_msgs::CollisionObject co;
    // shape_msgs::Mesh co_mesh;
    // shapes::ShapeMsg co_mesh_msg = co_mesh;
    // shapes::constructMsgFromShape(m,co_mesh_msg);
    // co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

}

