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
                // Turns on verbose output mode.
                mode = *optarg;
                break;

            case 'h':
                // Print a short description of this program and its arguments.
                cout << "Find the path for drones according to mode. Valid modes are 'MST', 'FASTTSP', and 'OPTTSP'.\n";
                exit(0);
                break;
        }
    }
    
    // Print an error message if no mode was specified and exit 1.
    if (mode == "") {
        cout << "Error: No mode specified\n";
        exit(1);
    }
}
