#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include <cmath>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include "vector2.h"

// Define a line as a start and end point
/**
 * @brief A 2d line, defined by 2 points (Vector2).
 * 
 */
struct line {
  Vector2 A;
  Vector2 B;
};


// The 'centre' reference for sensors is offset from zero angle (positive x axis)
const double ANGULAR_OFFSET = 90.0;
const int CELL_NUM_SIDES = 4;


/**
 * @brief Generates the fusion of rangers(sensors) and cells.
 * 
 */
class RangerFusion: public RangerFusionInterface
{
public:
  //Default constructor should set all RangerFusion attributes to a default value
  RangerFusion();

  // Accepts container of rangers - as per requirement C1 
  void setRangers(std::vector<RangerInterface*> rangers);

  // Accepts container of cells - as per requirement C2 
  void setCells(std::vector<Cell*> cells);

  // Grab data and fuse - as per requirement C3
  void grabAndFuseData();

  // Returns a container of raw data range readings - as per requirement C5 
  std::vector<std::vector<double>> getRawRangeData();

  
  
private:
  // This is to cater for getRawRangeData (which generates the raw data))
  std::vector<std::vector<double>> data_;

  // Object containers
  std::vector<RangerInterface*> rangers_;
  std::vector<Cell*> cells_;

  // Cell geometry functions to detect sensor collision
  bool cellIntersectPoint(Cell * c, Vector2 p) ;
  bool cellIntersectCone(Cell * c, Vector2 A, Vector2 B, Vector2 C);
  
  // Methods perform basic geometric checks
  bool pointInTriangle(Vector2 p, Vector2 A, Vector2 B, Vector2 C);
  bool lineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4);
  
  // Utility functions
  std::vector<line> getCellEdge(Cell * c);
  std::vector<Vector2> getCellVertex(Cell * c);
  void triangleToVertex(double range, double theta, double fov,  Vector2& A, Vector2& B, Vector2& C);
  void lineToVertex(double range, double theta, Vector2& A, Vector2& B);
  void wrapAngle(double& theta);
};


#endif // RANGERFUSION_H
