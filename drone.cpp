//  1761414855B69983BD8035097EFBD312EB0527F0
//  drone.cpp
//  p4-drone
//
//  Created by James Zhang on 4/7/21.
//

#include "xcode_redirect.hpp"
#include "Algorithms.h"
#include <iomanip>

// ----------------------------------------------------------------------------
//                                  Driver
// ----------------------------------------------------------------------------

int main(int argc, char** argv) {
    ios_base::sync_with_stdio(false); // Speeds up I/O.
    xcode_redirect(argc, argv); // Xcode redirect.
    
    cout << setprecision(2); // Always show 2 decimal places.
    cout << fixed; // Disable scientific notation for large numbers.
    
    
    Algorithms omg;
    
    omg.getOptions(argc, argv);
    omg.readInput();
    omg.processAlgorithm();
    return 0;
}
