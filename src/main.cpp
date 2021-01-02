#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

//path: for the file we will be using
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
	//$(is): input file stream , initialized by $(path), 
    //options: 
	//std::ios::binary - reading the path as binary data, at the end - 
	//std::ios::ate    - will imidiately seek to the end of the input stream
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
	//telg(): determin the size of the input stream
    auto size = is.tellg();
	
	//$(contents): vector of bytes, initialized at $(size)
    std::vector<std::byte> contents(size);    
    
	//sek back to the begining of the input stream
    is.seekg(0);

	//read all the input stream $(is) into the $(contents) vector
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    //std::move- when we done , we will return the contents vector
	// allow you to return the content of this vector to pointer or reference
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
	//parse command arguments: -f <file name> -> $(osm_data_file)
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
 
	// read content of file $(osm_data_file) to $(osm_data)
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
	float start_x;
	std::cout << "Please enter the start point x coordinate: ";
	std::cin >> start_x;
	float start_y;
	std::cout << "Please enter the start point y coordinate: ";
	std::cin >> start_y;
	float end_x;
	std::cout << "Please enter the end point x coordinate: ";
	std::cin >> end_x;
	float end_y;
	std::cout << "Please enter the end point y coordinate: ";
	std::cin >> end_y;
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
	// args: $(model) start point x,y, end point x,y 
    RoutePlanner route_planner{model, start_x,start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

	//io2nd code to diaplay the results
    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
