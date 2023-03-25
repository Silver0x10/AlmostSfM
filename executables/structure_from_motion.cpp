#include <iostream>
#include <string>
#include <vector>
#include "utils.h"
#include "init_translations.h"

using namespace std;
using namespace pr;

int main (int argc, char** argv) {
    string dataset_path = "../dataset_and_info/dataset.txt";
    vector<Camera> cameras = load_data(dataset_path);


    init_translations(cameras);

    return 0;

}