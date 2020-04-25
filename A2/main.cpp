#include <iostream>
#include <iomanip>
#include <chrono>

#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"
#include "cell.h"



using Clock = std::chrono::steady_clock;

void setSensorProperties(std::vector<RangerInterface *> &rangers);

// DEBUG functions
void outputRawData(std::vector<std::vector<double>> data, std::vector<RangerInterface *> sensors);
void outputCellData(std::vector<Cell*> cells);

// Time between each data output (ms)
const double LOOP_PERIOD = 1000;

int main () {

    // Define some objects
    Laser laser1;
    Sonar sonar1, sonar2;
    RangerFusion fusion;

    // ----------------- CREATE CELL AND SENSOR LIST -------------------
    // -----------------------------------------------------------------
    std::vector<Cell *> cells;
    std::vector<RangerInterface *> rangers = {&sonar1, &sonar2, &laser1};



    // -----------------OUTPUT SENSOR FIXED PARAMETERS-----------------
    // ----------------------------------------------------------------
    std::cout << "Sonar 1:" << sonar1.getFixedParameters() << std::endl;
    std::cout << "Sonar 2:" << sonar2.getFixedParameters() << std::endl;
    std::cout << "Laser 1:" << laser1.getFixedParameters() << std::endl;


    // Set sensor properties
    setSensorProperties(rangers);


    // -------------------------------GENERATE CELLS----------------------------------
    // -------------------------------------------------------------------------------
    int num_cells;

    std::cout << "\nEnter number of Cells: ";

    while(!(std::cin >> num_cells)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Enter integer value" << std::endl;
    }

    // Print out verification
    std::cout << "\033[F";
    std::cout << "Number of Cells: " << num_cells << std::setw(20) << std::left << " OK" << std::endl;
    
    // Allocate cells
    cells.resize(num_cells);
    for(auto &cell : cells) {
        cell = new Cell();
    }

    // ------------------------GENERATE CELL DISTRIBTION-------------------------
    // --------------------------------------------------------------------------

    // Find the maximum range of any sensor
    double max_range = 0;
    for(auto ranger : rangers) {
        if(ranger->getMaxRange() > max_range) {
            max_range = ranger->getMaxRange();
        }
    }

    // Give cells random locations, within max range
    int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::uniform_real_distribution<double> xdist(-max_range, max_range);
    std::uniform_real_distribution<double> ydist(0, max_range);
    for(auto cell : cells) {
        cell->setCentre(xdist(gen), ydist(gen));
    }
    


    //------------------------PERFORM FUSION----------------------
    //------------------------------------------------------------
    

    // Set sensors and cells for fusion
    fusion.setRangers(rangers);
    fusion.setCells(cells);
    
    // Time points to manage data output frequency
    Clock::time_point now = Clock::now();
    Clock::time_point last_output = Clock::now();

    // Keep track of the iteration
    int output_num = 0;

    // Heading
    std::cout << "\nCell States:" << std::endl;
    


    while(1) {
        // Current Time
        now = Clock::now();

        // If it is time to fuse and output again
        if(std::chrono::duration_cast<std::chrono::milliseconds>(now - last_output).count() > LOOP_PERIOD) {
            
            // Increment counter
            ++output_num;

            // Generate data and fuse
            fusion.grabAndFuseData();
            
            // Interation number
            std::cout << "Iteration: " << output_num << ":" << "\t";
            
            // Output cell states
            for(auto c : cells) {
                std::cout << c->getState() << "\t";
            }
            std::cout << std::endl;
         
            // Update output time
            last_output = Clock::now();
        }   
    }

    // Delete cells
    for(auto &cell : cells) {
        delete cell;
    }
  
    return 0;
}


void setSensorProperties(std::vector<RangerInterface *> &rangers) {

    unsigned int point_number = 1;
    unsigned int cone_number = 1;

    std::string sensor_name;

    for(auto sensor : rangers) {

        // Set sensor name for output
        if(sensor->getSensingMethod() == POINT) {
            // Laser type
            sensor_name = ("Laser " + std::to_string(point_number));
            point_number++;

        }else if(sensor->getSensingMethod() == CONE) {
            // Sonar type
            sensor_name = ("Sonar " + std::to_string(cone_number));
            cone_number++;

        }else {
            sensor_name = "Unknown Sensor";
        }
        
        int offset;
        std::cout << "Enter " << sensor_name << " Offset: ";
    
        while(!(std::cin >> offset)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Enter integer value: ";
        }
        sensor->setOffset(offset);
        std::cout << "\033[F";
        std::cout << sensor_name << " Offset: " << offset << std::setw(20) << std::left << " OK" << std::endl;


        int fov;
        std::cout << "Enter " << sensor_name << " FOV: ";

        while(!(std::cin >> fov)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Enter integer value: ";
        }
        if(sensor->setFieldOfView(fov)) {
            std::cout << "\033[F";
            std::cout << sensor_name <<  " FOV: " << fov << std::setw(20) << std::left << " OK" << std::endl;
        }else {
            std::cout << "\033[F";
            std::cout << sensor_name << " FOV: " << fov << " INVALID. USING DEFAULT(" << sensor->getFieldOfView() << std::setw(20) << std::left<< ")" << std::endl;
        }

        int resolution;
         std::cout << "Enter " << sensor_name << " Resolution: ";
        while(!(std::cin >> resolution)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Enter integer value: ";
        }
        if(sensor->setAngularResolution(resolution)) {
            std::cout << "\033[F";
            std::cout << sensor_name << " Resolution: " << resolution << std::setw(20) << std::left << " OK" << std::endl;
        }else {
            std::cout << "\033[F";
            std::cout << sensor_name << " Resolutin: " << resolution << " INVALID. USING DEFAULT(" << sensor->getAngularResolution() << std::setw(20) << std::left<< ")" << std::endl;
        }

        // To make each sensor clear
        std::cout << "---------------------------------------" << std::endl;
    }
}



// THIS IS JUST A DEBUG FUNCTION
// outputs raw data for matlab sim
void outputRawData(std::vector<std::vector<double>> data, std::vector<RangerInterface *> sensors) {
    int name_counter = 0;
    std::vector<std::string> names = {"sonar_data1", "sonar_data2", "laser_data"};


    for(auto &sensor : sensors) {
        if(sensor->getSensingMethod() == CONE) {
            std::cout << names.at(name_counter) << " = [" << std::endl;
            std::cout << data.at(name_counter).front() << ", " << sensor->getFieldOfView()  << ", " << sensor->getOffset() << ";\n]" << std::endl;
        }else {
            std::cout << names.at(name_counter) << " = [" << std::endl;
            for(int i = 0; i < data.at(2).size(); ++i) {
                std::cout << data.at(2).at(i) << ", " << sensor->getOffset() + (int)sensor->getAngularResolution() * i << ";" << std::endl;
            }
            std::cout << "]" << std::endl;
        }
        
        
        name_counter++;
        if (name_counter > 3) {
            return;
        }
    }
}

// THIS IS JUST A DEBUG FUNCTION
// outputs raw data for matlab sim
void outputCellData(std::vector<Cell*> cells) {
    // Output Cell positions for testing
    std::cout << "cell_data = [" << std::endl;
    for(std::vector<Cell *>::iterator it = cells.begin(); it < cells.end(); ++it) {
        double x, y;
        (*it)->getCentre(x, y);
        std::cout << x << ", " << y << ";" << std::endl;
    }
    std::cout << "]" << std::endl;
}