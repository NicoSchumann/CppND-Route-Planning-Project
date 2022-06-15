#include <cstdlib>
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



///
/// Prints usage hints.
///
static void PrintHelp( std::ostream &ostr = std::cout )
{
    ostr << "You need to input the start coordinates and the goal coordinates.\n"
        << "Valid coordinates are in range from [0...100].\n"
        << "These could be inputted as floating point values.\n"
        << "Example:\n"
        << "         10  8.0  57.3  90\n\n";
}
        
///
/// Reads start and goal coordinates. By default, from standard input (console).
///
static void ReadInput(float &start_x, float &start_y, float &end_x, float &end_y,
                       std::istream &istr = std::cin 
                     ) 
{
    if ( !(istr >> start_x)  ||  (start_x < 0 || start_x > 100)  ){
        PrintHelp();
        std::exit(EXIT_FAILURE);
    }
    if ( !(istr >> start_y)  ||  (start_y < 0 || start_y > 100)  ) {
        PrintHelp();
        std::exit(EXIT_FAILURE);
    }
    if ( !(istr >> end_x)  ||  (end_x < 0 || end_x > 100)  ) {
        PrintHelp();
        std::exit(EXIT_FAILURE);
    }
    if ( !(istr >> end_y)  ||  (end_y < 0 || end_y > 100)  ) {
        PrintHelp();
        std::exit(EXIT_FAILURE);
    }
}



///
/// Tries to read an openstreetmap data file from passed program arguments.
///
static void HandleProgArgs(int &argc, const char **argv ,std::string &osm_data_file)
{
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
}

///
/// Reads in OpenStreetMapFile.
///
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}


///
/// Tries to read in openstreetmap data.
///
static void ReadOSMData( std::string &osm_data_file, std::vector<std::byte> & osm_data)
{
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
}

///
/// Displays an OpenStreetMap model
///
static int ShowSolution( RouteModel &model)
{
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    return display.begin_show();
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file_name = "";
    HandleProgArgs(argc, argv, osm_data_file_name);
    
    std::vector<std::byte> osm_data;
    ReadOSMData(osm_data_file_name, osm_data );
     
    float start_x, start_y, end_x, end_y;  ///< Values for start and goal of the path to search.    
    ReadInput(start_x, start_y, end_x, end_y);

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Shows the result of the search.
    return ShowSolution(model);
}
