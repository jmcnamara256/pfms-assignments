#include "rangerfusion.h"


/**
 * @brief Construct a new Ranger Fusion:: Ranger Fusion object
 * 
 */
RangerFusion::RangerFusion() {};

/**
 * @brief Sets the list of sensors.
 * 
 * @param rangers 
 */
void RangerFusion::setRangers(std::vector<RangerInterface*> rangers) {
    rangers_ = rangers;
}

/**
 * @brief Sets the list of cells.
 * 
 * @param cells 
 */
void RangerFusion::setCells(std::vector<Cell*> cells) {
    cells_ = cells;
}



/**
 * @brief Combines randomly generated sensor data with provided container of cells. Generates a 'fusion' of the data based on collision conditions.
 * New random sensor data is generated.
 * 
 */
void RangerFusion::grabAndFuseData() {

    std::vector<double> sensor_data;

    // Clear old data, resize
    data_.clear();
    data_.resize(rangers_.size());


    // keep this data for when the fusion occurs, so that new data isnt used.
    for(int i = 0; i < data_.size(); ++i){
        sensor_data.clear();
        sensor_data = rangers_.at(i)->generateData();
        
        data_.at(i).clear();
        data_.at(i) = sensor_data;
    }
    
    // This shouldn't happen
    if(rangers_.size() != data_.size()) {
        // Something has gone wrong
        return;
    }

    

    // Iterators for data and sensors
    auto it_r = rangers_.begin();
    auto it_d = data_.begin();

    // for each cell, check each sensor.
    // if the cell becomes occupied, we can break
    for(auto cell : cells_) {

        // Cells should all be unknown to start with
        cell->setState(UNKNOWN);
        
        //check each sensor
        // Iterator for Ranger and Data vectors
        std::vector<RangerInterface *>::iterator it_r = rangers_.begin();
        std::vector<std::vector<double>>::iterator it_d = data_.begin();

        // Iterate through both containers together
        for(;it_r < rangers_.end() && it_d < data_.end(); ++it_r, ++it_d) {

            // If the cell is already occuped there is no reason to test more sensors against it
            if(cell->getState() == OCCUPIED) {
                break;
            }
            
            // For each sonar
            if((*it_r)->getSensingMethod() == CONE) {
                // Angle for the centre of the sensor
                // Relative to 0 (positive x-axis)
                double theta = ANGULAR_OFFSET + (*it_r)->getOffset();
                double fov = (*it_r)->getFieldOfView();

                // Because data and rangers have equal number of elements, we can access both with our counter i
                // The radar data is a single value, so use front()
                double range = it_d->front();

                // Convert the sensor triangle to 3 verticies
                Vector2 A,B,C;
                triangleToVertex(range, theta, fov, A, B, C);

                // Check cell collisions
                cellIntersectCone(cell, A, B, C);


            // If the sensor is a laser
            }else if((*it_r)->getSensingMethod() == POINT) {
                
                // The laser has multiple beams
                // For this cell, we must check each beam, iterate through them

                // Counter for which beam we are on
                // Used to calculate the beam angle
                int j = 0;
                std::vector<double>::iterator it = it_d->begin();

                // For each beam
                for(; it < it_d->end(); ++it, ++j) {


                    // The angle of the current beam relative to zero (+ve x-axis)
                    double theta = ANGULAR_OFFSET + (*it_r)->getOffset() - (*it_r)->getFieldOfView() * 0.5 + int((*it_r)->getAngularResolution()) * j;

                    // Get the start/end of the beam
                    Vector2 A,B;
                    lineToVertex(*it, theta, A, B);
                
                    // Cell is hit
                    // We only test the end, A isnt needed
                    if(cellIntersectPoint(cell, B)) {
                        cell->setState(OCCUPIED);

                        // If this cell is occupied, we dont need to test any more beams
                        // Go to the next sensor
                        break;
                    }
                    
                    // The other case is free, where the cell intersects the line
                    // Check each edge of the cell with the beam
                    for(auto edge : getCellEdge(cell)) {

                        // If the beam intersects edge
                        if(lineIntersection(A, B, edge.A, edge.B)) {
                            cell->setState(FREE);

                            // If there is an intersection, we dont need to test any more edges of the cell
                            break;
                        }
                    }





                }
            }
        }
    }
}

/**
 * @brief Returns the raw data from any sensors in the ranger container.
 * The raw data is updated every time a new fusion is requested. The raw data will match the preceeding fusion if it is called between fusions.
 * 
 * @return std::vector<std::vector<double>> 
 */
std::vector<std::vector<double>> RangerFusion::getRawRangeData() {
    return data_;
}

/**
 * @brief Returns true if point is contained within a cell.
 * 
 * @param c Pointer to cell object.
 * @param p Vector2 point in x,y coordinates.
 * @return true 
 * @return false 
 */
bool RangerFusion::cellIntersectPoint(Cell * c, Vector2 p) {
    // The cell state could be directly set here
    // The function is more flexible if the return is used instead
    // cell center and half side length
    double cx, cy;
    double s;

    // assign variables
    c->getCentre(cx, cy);
    s = c->getSide()* 0.5;

    // x coordinate is between square sides
    // y coordinate is between top and bottom
    if (p.x >= cx - s && p.x <= cx + s && p.y >= cy - s && p.y <= cy + s) {
        return true;
    }else {
        return false;
    }
}



/**
 * @brief Returns true if 2 line segments intersect.
 *
 * @param p1 Line segment 1 start.
 * @param p2 Line segment 1 end.
 * @param p3 Line segment 2 start.
 * @param p4 Line segment 2 end.
 * @return true 
 * @return false 
 */
bool RangerFusion::lineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4){
    // This function uses the vector dot-product method outlined in Graphics Gems 3 by David Kirk
    Vector2 A,B,C;

    A = p2 - p1;
    B = p3 - p4;
    C = p1 - p3;

    double BxA = Vector2::cross(B,A);
    double AxC = Vector2::cross(A,C);
    double CxB = Vector2::cross(C,B);

    // If both are == 0, collinear
    if(BxA == 0 && AxC == 0) {

        // Collinear & intersecting if the intervals overlap.
        // Check x and y intervals 
        return ((p3.x - p1.x <= 0) != (p3.x - p2.x <= 0))
            || ((p3.y - p1.y <= 0) != (p3.y - p2.y <= 0));
    }

    if(BxA == 0) {

        // Lines are parallel non intersecting
        return false;
    }

    double alpha = CxB / BxA;
    if(alpha >= 0 && alpha <= 1) {
        double beta = AxC / BxA;

        if (beta >= 0 && beta <= 1) {
            return true;
        }
    }

    return false;
}


/**
 * @brief Checks a triangluar (conical) region for hit/miss/unknown cell state.
 * 
 * @param c Cell to check.
 * @param A Origin point of triangle. Must be opposite sensing side.
 * @param B 2nd point of triangle.
 * @param C 3rd point of triangle.
 * @return true 
 * @return false 
 */
bool RangerFusion::cellIntersectCone(Cell * c, Vector2 A, Vector2 B, Vector2 C) {
    // For simplicity, assume that A is the sensor origin.
    // When using this function, A should be the the vertex opposite the sensing edge of the triangle.
    // Or a 0,0 vector
    
    // Test hit case first
    // Check each cell edge with the BC (sensing) edge of the sonar triangle
    for(auto edge : getCellEdge(c)) {
        // Check if sonar sensing line intersects
        if(lineIntersection(edge.A, edge.B, B, C)) {
            c->setState(OCCUPIED);
            return true;
        }
    }

    // Test hit case 2
    // Cell fully encloses sonar sensing end
    // This would only apply for large cells
    if(cellIntersectPoint(c, B) || cellIntersectPoint(c, C)) {
        c->setState(OCCUPIED);
        return true;
    }

    // check for intersection between cell edges and AC/AB
    for(auto edge : getCellEdge(c)) {
        // If the cell edge intersects either side of the sonar cone
        if(lineIntersection(edge.A, edge.B, A, B) || lineIntersection(edge.A, edge.B, A, C)) {
            c->setState(FREE);
            return true;
        }
    }

    // Check if any cell verticies are contained within the triangle
    // This check applies to the case where the cell is fully contained in the sonar cone
    for(auto vertex : getCellVertex(c)) {
        if(pointInTriangle(vertex, A, B, C)) {
            c->setState(FREE);
            return true;
        }
    }

    return false;
}

/**
 * @brief Returns true if the point p is contained within triangle ABC.
 * 
 * @param p Point to test.
 * @param A Triangle vertex 1.
 * @param B Triangle vertex 2.
 * @param C Triangle vertex 3.
 * @return true 
 * @return false 
 */
bool RangerFusion::pointInTriangle(Vector2 P, Vector2 A, Vector2 B, Vector2 C) {
    // Uses barycentric coordinates to evaluate if a point is contained by a triangle
    // We use point A as our reference for the coordinates
    // v0 and v1 are the unit vectors of the coordinate system
    // v2 is the vector from A to P

    Vector2 v0 = C - A;
    Vector2 v1 = B - A;
    Vector2 v2 = P - A;

    // Calculate the dot-products
    double dot00, dot01, dot02, dot11, dot12;
    dot00 = Vector2::dot(v0, v0);
    dot01 = Vector2::dot(v0, v1);
    dot02 = Vector2::dot(v0, v2);
    dot11 = Vector2::dot(v1, v1);
    dot12 = Vector2::dot(v1, v2);

    // Calculate position vectors a and w
    double den = 1 / (dot00 * dot11 - dot01 * dot01);
    double w = (dot11 * dot02 - dot01 * dot12) * den;
    double a = (dot00 * dot12 - dot01 * dot02) * den;

    // If either vector is less than 0, the point is out the back of the triangle
    // If a + w > 1, the point is past the end
    return (w >= 0 && a >= 0 && a + w <= 1);
}

/**
  * @brief Returns the end points of the line segment forming 1 side of the cell.
 *         Which side is determined by the integer argument between 0 and 3.
 * 
 * @param c Pointer to cell.
 * @param side Side number (0-3) .
 * @param p1 End point 1.
 * @param p2 End point 2.
 */
std::vector<line> RangerFusion::getCellEdge(Cell * c) {
    
    // container
    std::vector<line> edges;

    // 4 Corners
    Vector2 p1, p2, p3, p4;

    // Center of cell
    double xc, yc, half_side;
    c->getCentre(xc, yc);

    half_side = c->getSide() * 0.5;

    // Top right
    p1.x = xc + half_side;
    p1.y = yc + half_side;
   
    // Bottom right
    p2.x = p1.x;
    p2.y = yc - half_side;
    
    // Bottom left
    p3.x = xc - half_side;
    p3.y = p2.y;

    // Top left
    p4.x = p3.x;
    p4.y = p1.y;

    edges = {{p1, p2}, {p2, p3}, {p3, p4}, {p4, p1}};

    return edges;
}

/**
 * @brief Returns the vertex of the cell specified by an integer 0-3.
 * 
 * @param c Cell used.
 * @param vertex Vertex, from top right proceeding counter-clockwise.
 * @return Vector2 
 */
std::vector<Vector2> RangerFusion::getCellVertex(Cell * c) {

    // container for the corners
    std::vector<Vector2> corners;

    // 4 Corners
    Vector2 p1, p2, p3, p4;

    // Center of cell
    double xc, yc, half_side;
    c->getCentre(xc, yc);

    half_side = c->getSide() * 0.5;

    // Top right
    p1.x = xc + half_side;
    p1.y = yc + half_side;
   
    // Bottom right
    p2.x = p1.x;
    p2.y = yc - half_side;
    
    // Bottom left
    p3.x = xc - half_side;
    p3.y = p2.y;

    // Top left
    p4.x = p3.x;
    p4.y = p1.y;

    corners = {p1, p2, p3, p4};

    return corners;
}

/**
 * @brief Returns the 3 verticies of the given triangle
 * 
 * @param range The perpendicular length of the triangle from the origin.
 * @param theta The absolute angle from 0, to the centerline of the triangle.
 * @param fov The angular width of the triangle.
 * @param A Base vertex (origin).
 * @param B 2nd vertex.
 * @param C 3rd vertex.
 */
void RangerFusion::triangleToVertex(double range, double theta, double fov,  Vector2& A, Vector2& B, Vector2& C) {
    // The angles of the two sides of the sonar reading.
    // They are offset by half the FOV from the center
    double theta_b = M_PI * (theta - fov * 0.5) / 180;
    double theta_c = M_PI * (theta + fov * 0.5) / 180;
    double side = range / cos(M_PI*fov * 0.5 / 180);

    // Check the angles are within bounds
    wrapAngle(theta_b);
    wrapAngle(theta_c);


    // Polar -> cartesian
    double bx = side * cos(theta_b);
    double by = side * sin(theta_b);
    
    // Polar -> cartesian
    double cx = side * cos(theta_c);
    double cy = side * sin(theta_c);

    // Set the verticies
    A = Vector2::zero();
    B = Vector2(bx, by);
    C = Vector2(cx, cy);
}


/**
 * @brief Returns 2 verticies representing a line given in polar coordinates.
 * 
 * @param range Length of line from origin.
 * @param theta Angle of Line.
 * @param A Start vertex (origin).
 * @param B End vertex.
 */
void RangerFusion::lineToVertex(double range, double theta, Vector2& A, Vector2& B) {
    theta = M_PI * theta / 180;
    wrapAngle(theta);
    double bx = range * cos(theta);
    double by = range * sin(theta);

    A = Vector2::zero();
    B = Vector2(bx, by);
}

/**
 * @brief Wrap the angle between -PI and PI.
 * 
 * @param theta Angle to be constrained.
 */
void RangerFusion::wrapAngle(double& theta) {
    if(std::abs(theta) > M_PI) {
        theta = atan2(sin(theta), cos(theta));
    }
}