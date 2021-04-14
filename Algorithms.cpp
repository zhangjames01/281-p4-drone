// 1761414855B69983BD8035097EFBD312EB0527F0

/*

    A utility file containing the member function definitions of the class Algorithms.

*/

#include "Algorithms.h"
#include <getopt.h>

// Read and process command line options.
void Algorithms::getOptions(int argc, char** argv) {

    int option_index = 0, option = 0;
    
    // Don't display getopt error messages about options.
    opterr = false;

    // Use getopt to find command line options.
    struct option longOpts[] = {{"mode",  required_argument, nullptr, 'm' },
                                {"help",  no_argument,       nullptr, 'h' },
                                {nullptr, 0,                 nullptr, '\0'}};
    
    while ((option = getopt_long(argc, argv, "hm:", longOpts, &option_index)) != -1) {
        switch (option) {
            case 'm':
                // Sets the mode of the program to the command argument.
                // Mode must be "MST", "FASTTSP", or "OPTTSP".
                // Mode cannot be empty or invalid. Will print error message and exit.
                setMode(optarg);
                break;

            case 'h':
                // Print a short description of this program and its arguments.
                cout << "Find the path for drones according to mode. "
                << "Valid modes are 'MST', 'FASTTSP', and 'OPTTSP'.\n";
                exit(0);
                break;
                
            default:
                // Print an error message if command is invalid and exit 1.
                cerr << "Error: Invalid command line option\n";
                exit(1);
                break;
        }
    }
}

// Reads input describing the locations where pickups and/or deliveries occur.
void Algorithms::readInput() {
    // Read in number of locations.
    cin >> numLocations;
    // Reserve the vector to number of locations.
    droneLocations.reserve(numLocations);
    coordinate temp;
    
    // While a coordinate is being read in.
    while (cin >> temp.x) {
        cin >> temp.y;
        // Set the type of location the coordinate is.
        temp.location = categorizeLocation(temp.x, temp.y);
        // Insert into vector.
        droneLocations.emplace(droneLocations.end(), temp);
    }
}

// Process which algorithm to use based on the mode given.
void Algorithms::processAlgorithm() {
    switch (mode) {
        case Mode::MST:
            checkMSTPossible();
            mstAlgorithm();
            printMST();
            break;
            
        case Mode::FASTTSP:
            fasttspAlgorithm();
            printFASTTSP();
            break;
            
        case Mode::OPTTSP:
            opttspAlgorithm();
            printOPTTSP();
            break;
    }
}

    // ----------------------------------------------------------------------------
    //                                     MST
    // ----------------------------------------------------------------------------

// Process that creates a minimum spanning tree using linear search.
void Algorithms::mstAlgorithm() {
    // Resize the prim table to number of locations.
    primTable.resize(numLocations);
    
    // Set starting vertex 0.
    primTable[0].minEdgeWeight = 0;
    
    int timesTrue = 0;
    int currentVertex = 0;
    double minDistance = INF;
    
    // Loop until every vertex has been visited.
    while (timesTrue < numLocations) {
        minDistance = INF;
        
        // From the set of unvisited vertices, choose the vertex k having the
        // smallest distance to current vertex.
        for (int k = 0; k < numLocations; ++ k) {
            if (primTable[k].isVisited == 0) {
                if (primTable[k].minEdgeWeight < minDistance) {
                    minDistance = primTable[k].minEdgeWeight;
                    currentVertex = k;
                }
            }
        }
        
        // Set current vertex visited.
        primTable[currentVertex].isVisited = 1;
        ++ timesTrue;
        
        // For each vertex w adjacent to curent vertex.
        for (int w = 0; w < numLocations; ++ w) {
            minDistance = calculateDistance(droneLocations[currentVertex], droneLocations[w]);
            // If it has not been visited.
            if (primTable[w].isVisited == 0) {
                // It's distance is smaller than (current,w).
                if (minDistance < primTable[w].minEdgeWeight) {
                    // Process total weight meanwhile.
                    calculateTotalWeight(w, minDistance);
                    // Change it's min distance and preceding vertex.
                    primTable[w].minEdgeWeight = minDistance;
                    primTable[w].precedingVertex = currentVertex;
                }
            }
        }
    }
}

// Print out the results of MST.
void Algorithms::printMST() {
    cout << totalWeight << "\n";
    
    for (int i = 1; i < numLocations; ++ i) {
        if (i < primTable[i].precedingVertex) {
            cout << i << " " << primTable[i].precedingVertex << "\n";
        }
        else {
            cout << primTable[i].precedingVertex << " " << i << "\n";
        }
    }
}

    // ----------------------------------------------------------------------------
    //                                   FASTTSP
    // ----------------------------------------------------------------------------

// Process that creates a close-to-optimal Hamiltonian Cycle using arbitrary insertion.
void Algorithms::fasttspAlgorithm() {
    // Initialization: Start with a partial tour of three cities.
    partialTour.push_back(0);
    partialTour.push_back(1);
    partialTour.push_back(2);
    
    double minCost = INF;
    double newCost = 0;
    
    // Selection: Arbitrarily select a city to add to the partial tour.
    for (int k = 3; k < numLocations; ++ k) {
        // Record the minimum change in cost of each edge in the partial tour.
        minCost = calculateNewCost(droneLocations[partialTour[0]], droneLocations[partialTour[1]], droneLocations[k]);
        uint32_t indexInserting = 1;
        
        // Insertion: For each edge in the partial tour, calculate the change in cost if node k were inserted in between.
        for (size_t m = 1; m < partialTour.size(); ++ m) {
            newCost = calculateNewCost(droneLocations[partialTour[m]], droneLocations[partialTour[(m + 1) % partialTour.size()]], droneLocations[k]);
            
            // Insertion: If change in cost is smaller, keep track of new potential edge to insert.
            if (newCost <= minCost) {
                minCost = newCost;
                indexInserting = static_cast<uint32_t>((m + 1));
            }
        }
        // Insertion: Insert at index between i and j.
        partialTour.insert(partialTour.begin() + indexInserting, k);
    }
}

// Print out the results of FASTTSP.
void Algorithms::printFASTTSP() {
    // Calculate total weight of cycle.
    calculateTotalWeight();
    
    // Print out total weight.
    cout << totalWeight << "\n";
    
    // Print out each city in partial tour.
    for (int i = 0; i < numLocations; ++ i) {
        cout << partialTour[i] << " ";
    }
    cout << "\n";
}

    // ----------------------------------------------------------------------------
    //                                   OPTTSP
    // ----------------------------------------------------------------------------

// Process that creates a optimal Hamiltonian Cycle using genPerms especially.
void Algorithms::opttspAlgorithm() {
    // Process Distance Matrix.
    processDistanceMatrix();
    // Find upper bound.
    fasttspAlgorithm();
    calculateTotalWeight();
    upperBound = totalWeight;
    bestPath = partialTour;
   
    
    totalWeight = 0; // Reset total weight after assigning upper bound
    size_t permLength = 1;
    genPerms(permLength);
    
    
    totalWeight = 0;
    for (int i = 0; i < numLocations; ++ i) {
        totalWeight += distanceMatrix[bestPath[i]][bestPath[(i + 1) % numLocations]];
    }
}


void Algorithms::genPerms(size_t permLength) {
    if (permLength == partialTour.size()) {
      // Add weight of last edge.
      totalWeight += distanceMatrix[partialTour[permLength - 1]][0];
      if (totalWeight < upperBound) {
          upperBound = totalWeight;
          bestPath = partialTour;
      }
      // Subtract weight of last edge.
      totalWeight -= distanceMatrix[partialTour[permLength - 1]][0];
      return;
    }
   
    if (!promising(permLength)) {
        return;
    }

    for (size_t i = permLength; i < partialTour.size(); ++ i) {
        swap(partialTour[permLength], partialTour[i]);
        totalWeight += distanceMatrix[partialTour[permLength]][partialTour[permLength - 1]];
        
        genPerms(permLength + 1);
        
        totalWeight -= distanceMatrix[partialTour[permLength]][partialTour[permLength - 1]];
        swap(partialTour[permLength], partialTour[i]);
  }
}


// Print out the results of OPTTSP.
void Algorithms::printOPTTSP() {
    // Print out total weight.
    cout << totalWeight << "\n";
    
    // Print out each city in partial tour.
    for (int i = 0; i < numLocations; ++ i) {
        cout << bestPath[i] << " ";
    }
    cout << "\n";
}
