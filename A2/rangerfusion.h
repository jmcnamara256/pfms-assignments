#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include <cmath>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"
#include "vector2.h"

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
  void getCellEdge(Cell * c, int side, Vector2& p1, Vector2& p2);
  Vector2 getCellVertex(Cell * c, int vertex);
  void triangleToVertex(double range, double theta, double fov,  Vector2& A, Vector2& B, Vector2& C);
  void lineToVertex(double range, double theta, Vector2& A, Vector2& B);
  void wrapAngle(double& theta);
};

#endif // RANGERFUSION_H
