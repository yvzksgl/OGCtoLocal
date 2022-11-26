#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kml/base/file.h"
#include "kml/base/version.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node{
public:
    PathPublisher() : Node("path_publisher"){
        this->declare_parameter("kml_file_path","data/LocalizationAssignmentTestRoute.kml");
        this->timer_=create_wall_timer(500ms,std::bind(&PathPublisher::path_pub,this));
        this->pub=this->create_publisher<nav_msgs::msg::Path>("path",10);
        PathPublisher::kml_reader();
        PathPublisher::extract_coordinates();
        PathPublisher::global_to_local();
    }
private:
    kmlengine::KmlFilePtr kml_reader(){
        std::string file_path;        
        this->get_parameter("kml_file_path",file_path);
        std::string file_content;
        if(kmlbase::File::ReadFileToString(file_path,&file_content)){
            return kmlengine::KmlFile::CreateFromString(file_content);
        }
        return NULL;
    }
    kmldom::CoordinatesPtr extract_coordinates(){
        kmlengine::KmlFilePtr kml_file=kml_reader();
        kmldom::PlacemarkPtr placemark0 = AsPlacemark(kml_file->GetObjectById("0C11468A89207A6E17BE"));
        kmldom::LineStringPtr linestring = AsLineString(placemark0->get_geometry());
        kmldom::CoordinatesPtr coordinates =linestring->get_coordinates();
        // size_t cooardinates_array_size=coordinates->get_coordinates_array_size();
        // coordinates->Parse(coordinates->get_char_data())
        /* for(size_t i=0;i<cooardinates_array_size;++i){
            kmlbase::Vec3 coordinates_array= coordinates->get_coordinates_array_at(i);
            RCLCPP_INFO(this->get_logger(),"altitude: %f latitude: %f longtitude: %f",coordinates_array.get_altitude(),coordinates_array.get_latitude(),coordinates_array.get_longitude());
        } */

        return coordinates;        
    }

    std::vector<geometry_msgs::msg::PoseStamped> global_to_local(){
        using namespace GeographicLib;
        kmldom::CoordinatesPtr coordinates= extract_coordinates();
        const double lat0 = coordinates->get_coordinates_array_at(0).get_latitude();
        const double lon0 = coordinates->get_coordinates_array_at(0).get_longitude();
        const double alt0 = coordinates->get_coordinates_array_at(0).get_altitude();
        Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
        LocalCartesian proj(lat0,lon0,alt0,earth);
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        for(size_t i=0;i<coordinates->get_coordinates_array_size();++i){
            kmlbase::Vec3 coordinates_array= coordinates->get_coordinates_array_at(i);
            double lat=coordinates_array.get_latitude();
            double lon=coordinates_array.get_longitude();
            double alt=coordinates_array.get_altitude();
            double x,y,z;
            proj.Forward(lat,lon,alt,x,y,z);
            auto pose_msg=geometry_msgs::msg::PoseStamped();
            pose_msg.pose.position.x=x;
            pose_msg.pose.position.y=y;
            pose_msg.pose.position.z=z;
            poses.push_back(pose_msg);
        }
        return poses;
    }
    void path_pub(){
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id="map";
        path_msg.set__poses(global_to_local());
        pub->publish(path_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}