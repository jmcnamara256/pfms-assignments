/*! @file
 *  @brief Main entry point for assignment 3.
 *  @author {Jesse McNamara : 12905749}
 *  @date {15/05/2020}
*/
#include <thread>
#include <iostream>
#include "simulator.h"
#include "fatcontroller.h"

int main(void)
{
    // Create a pointer to a simulator
    Simulator* sim = new Simulator();

    // Give it to the fat controller
    FatController fatty(sim);

    // Join the sim
    // Other threads are joined in the their respective owners destructors
    fatty.begin().join();

    // Clean up
    delete sim;

    return 0;
}
