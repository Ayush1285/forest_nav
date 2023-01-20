#include <mrs_msgs/Path.h>
#include <mrs_msgs/Reference.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/PathSrv.h>




int main(int argc, char** argv){

    mrs_msgs::Path path;
    int x = 3, y = 3, z = 3;
    path.fly_now = true;
    path.use_heading = false;
    path.stop_at_waypoints = false;
    path.override_constraints = false;
    //for(int i = 0;  i < 20; i++){
    mrs_msgs::Reference refpos;
    refpos.position.x = 3;
    refpos.position.y = 0;
    refpos.position.z = z;
    path.points.push_back(refpos);
    mrs_msgs::Reference refpos1;
    refpos1.position.x = 4.5;
    refpos1.position.y = 1.5;
    refpos1.position.z = z;
    path.points.push_back(refpos1);
    mrs_msgs::Reference refpos2;
    refpos2.position.x = 6;
    refpos2.position.y = 3;
    refpos2.position.z = z;
    path.points.push_back(refpos2);
    mrs_msgs::Reference refpos3;
    refpos3.position.x = 6;
    refpos3.position.y = 9;
    refpos3.position.z = z;
    path.points.push_back(refpos3);
    mrs_msgs::Reference refpos4;
    refpos4.position.x = 9;
    refpos4.position.y = 12;
    refpos4.position.z = z;
    path.points.push_back(refpos4);
    mrs_msgs::Reference refpos5;
    refpos5.position.x = 13;
    refpos5.position.y = 14;
    refpos5.position.z = z;
    path.points.push_back(refpos5);
    /*mrs_msgs::Reference refpos6;
    refpos6.position.x = 0;
    refpos6.position.y = 0;
    refpos6.position.z = z;
    path.points.push_back(refpos6);
    //x = x + 1;
    //y = y + 1;
    //}*/
    ros::init(argc, argv, "testfly");
    ros::NodeHandle nh;
    mrs_msgs::Reference refpos0;
    refpos0.position.x = 0;
    refpos0.position.y = 0;
    refpos0.position.z = 3;
    ros::ServiceClient flytoorigin = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/uav1/control_manager/reference");
    mrs_msgs::ReferenceStampedSrv originsrv;
    originsrv.request.reference = refpos0;
    bool flag0 = flytoorigin.call(originsrv);
    ros::Duration(5.0).sleep();
    ros::ServiceClient srvfly = nh.serviceClient<mrs_msgs::PathSrv>("/uav1/trajectory_generation/path");
    //ros::Publisher  path_pub = nh.advertise<mrs_msgs::Path>("/uav1/trajectory_generation/path",10);
    //path_pub.publish(path)
    
    mrs_msgs::PathSrv srv;
    srv.request.path = path;
    std::cout<<"started flying service" << std::endl;
    bool flag1 = srvfly.call(srv);
    std::cout<<"returned from flying service" << std::endl;
    ros::Duration(2.5).sleep();

    /*mrs_msgs::Path path1;
    path1.fly_now = true;
    path1.use_heading = false;
    path1.stop_at_waypoints = false;
    path1.override_constraints = false;
    //for(int i = 0;  i < 20; i++){
    mrs_msgs::Reference refpos6;
    refpos6.position.x = 1;
    refpos6.position.y = 0;
    refpos6.position.z = z;
    path1.points.push_back(refpos6);
    mrs_msgs::Reference refpos7;
    refpos7.position.x = -1;
    refpos7.position.y = 1.5;
    refpos7.position.z = z;
    path1.points.push_back(refpos7);
    mrs_msgs::Reference refpos8;
    refpos8.position.x = -4;
    refpos8.position.y = 3;
    refpos8.position.z = z;
    path1.points.push_back(refpos8);

    mrs_msgs::PathSrv srv1;
    srv1.request.path = path1;
    std::cout<<"started flying service" << std::endl;
    bool flag2 = srvfly.call(srv1);
    std::cout<<"returned from flying service" << std::endl;*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*ros::init(argc, argv, "testfly");
    ros::NodeHandle nh;    

    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "/map";
    geometry_msgs::Point32 point0;
    point0.x = 1;
    point0.y = -1;
    geometry_msgs::Point32 point1;
    point1.x = 1;
    point1.y = 1;
    geometry_msgs::Point32 point2;
    point2.x = -1;
    point2.y = 1;
    geometry_msgs::Point32 point3;
    point3.x = -1;
    point3.y = -1;
    polygon.polygon.points.push_back(point0);
    polygon.polygon.points.push_back(point1);
    polygon.polygon.points.push_back(point2);
    polygon.polygon.points.push_back(point3);

    ros::Publisher poly_pub = nh.advertise<geometry_msgs::PolygonStamped>("/polygon",5);
    while(ros::ok()){
        poly_pub.publish(polygon);
    }*/
    

    ros::spin();

}