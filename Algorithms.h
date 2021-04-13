// 1761414855B69983BD8035097EFBD312EB0527F0

/*

    A utility file containing the declaration of the class algorithms..

*/

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <limits>

using namespace std;

const double INF = numeric_limits<double>::infinity();

class Algorithms {
public:
    // Read and process command line options.
    void getOptions(int argc, char** argv);
    
    // Reads input describing the locations where pickups and/or deliveries occur.
    void readInput();
    
    // Process which algorithm to use based on the mode given.
    void processAlgorithm();
    
    // ----------------------------------------------------------------------------
    //                                     MST
    // ----------------------------------------------------------------------------
    
    // Process that creates a minimum spanning tree using linear search.
    void mstAlgorithm();
    
    // Print out the results of MST.
    void printMST();
    
    // ----------------------------------------------------------------------------
    //                                   FASTTSP
    // ----------------------------------------------------------------------------
    
    // Process that creates a close-to-optimal Hamiltonian Cycle using arbitrary insertion.
    void fasttspAlgorithm();
    
    // Print out the results of FASTTSP.
    void printFASTTSP();
    
    // ----------------------------------------------------------------------------
    //                                   OPTTSP
    // ----------------------------------------------------------------------------
    
    // Process that creates a optimal Hamiltonian Cycle using genPerms especially.
    void opttspAlgorithm();
    
    // Print out the results of OPTTSP.
    void printOPTTSP();
    
    
private:
    
    // ----------------------------------------------------------------------------
    //                              Member variables
    // ----------------------------------------------------------------------------
    
    // Helper variables to determine if a MST can be created.
    bool isBorder = 0;
    bool isMedical = 0;
    bool isNormal = 0;
    double totalWeight = 0;
    
    // Number of locations given.
    int numLocations;
    
    // Enumerated variable for the algorithm to be used.
    enum class Mode : char {MST, FASTTSP, OPTTSP};
    Mode mode;
    
    // Enumerated variable for what area each location is in.
    enum class Location : char {Normal, Border, Medical};
    
    // Variable for locations.
    struct coordinate {
        int x;
        int y;
        Location location;
    };
    
    // List of locations for dones on the map.
    vector<coordinate> droneLocations;
    
    struct Prim {
        bool isVisited = 0;
        double minEdgeWeight = INF;
        int precedingVertex;
    };
    
    // Prim table used for MST algorithm.
    vector<Prim> primTable;
    
    // Vector for TSP location order.
    vector<int> partialTour;
    
    // ----------------------------------------------------------------------------
    //                              Helper Functions
    // ----------------------------------------------------------------------------
    
    // Helper function to set mode and check if mode argument is valid.
    void setMode(const string& modeInput) {
        if (modeInput == "MST") {
            mode = Mode::MST;
        }
        else if (modeInput == "FASTTSP") {
            mode = Mode::FASTTSP;
        }
        else if (modeInput == "OPTTSP") {
            mode = Mode::OPTTSP;
        }
        else if (modeInput == "") {
            cerr << "Error: No mode specified\n";
            exit(1);
        }
        else {
            cerr << "Error: Invalid mode\n";
            exit(1);
        }
    }
 
    // Helper function to determine what part of campus location is in.
    Location categorizeLocation(int x, int y) {
        // If both x and y are negative (apart of the 3rd quadrant), in medical campus.
        if (x < 0 && y < 0) {
            isMedical = 1;
            return Location::Medical;
        }
        // If at least one is positive (not the 3rd quadrant), in normal campus.
        else if (x > 0 || y > 0) {
            isNormal = 1;
            return Location::Normal;
        }
        // Else, the coordinate is located on the border. MST can definitely be created.
        else {
            isBorder = 1;
            return Location::Border;
        }
    }
    
    // Helper function that calculates distance between two points. (MST)
    double calculateDistance(const coordinate& A, const coordinate& B) {
        
        // If points are not adacent, return infinity.
        if (A.location == Location::Normal && B.location == Location::Medical) {
            return INF;
        }
        else if (A.location == Location::Medical && B.location == Location::Normal) {
            return INF;
        }
        
        // Calculate distance normally if points are adjacent.
        else {
            return sqrt((static_cast<double>(B.x - A.x) * static_cast<double>(B.x - A.x))
                        + (static_cast<double>(B.y - A.y) * static_cast<double>(B.y - A.y)));
        }
        
    }
    
    // Helper function that readjusts total weight in MST algorithm.
    void calculateTotalWeight(const int w, const double distance) {
        if (primTable[w].minEdgeWeight != INF) {
            totalWeight -= primTable[w].minEdgeWeight;
            totalWeight += distance;
        }
        else {
            totalWeight += distance;
        }
    }
    
    // Helper function that determines if an MST can be constructed.
    void checkMSTPossible() {
        if (isNormal && isMedical && !isBorder) {
            cerr << "Cannot construct MST\n";
            exit(1);
        }
    }
    
    // Helper function that calculates distance between two points. (TSP)
    double calculateCost(const coordinate& A, const coordinate& B) {
        return sqrt((static_cast<double>(B.x - A.x) * static_cast<double>(B.x - A.x))
                    + (static_cast<double>(B.y - A.y) * static_cast<double>(B.y - A.y)));
    }
    
    // Helper function that calculates change in cost. (TSP)
    double calculateNewCost(const coordinate& i, const coordinate& j, const coordinate& k) {
        return calculateCost(i, k) + calculateCost(j, k) - calculateCost(i, j);
    }
};
