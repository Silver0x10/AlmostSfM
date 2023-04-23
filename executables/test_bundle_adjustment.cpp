#include <iostream>
#include <string>
// #include <vector>
// #include "utils.h"
#include "init_translations.h"

using namespace std;
using namespace pr;

int main (int argc, char** argv) {
    string dataset_path = "../dataset_and_info/input_BA.txt";
    vector<Camera> cameras = load_data(dataset_path);

    cout << "Test BundleAdjustment" << endl;

    // bundle_adjustment(cameras, landmarks);

    return 0;

}