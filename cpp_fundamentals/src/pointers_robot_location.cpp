#include <iostream>
#include <vector>
#include <memory>

using namespace std;

// This function now takes a const reference to a vector<int>
// It avoids unnecessary copying while maintaining the safety and simplicity of use.
void processSensorData(const vector<int>& data) {
    cout << "Processing sensor data:" << endl;
    for (int d : data) {
        cout << d << " ";
    }
    cout << endl;
}

int main() {
    // Using make_unique to dynamically allocate a vector of integers
    auto sensorData = make_unique<vector<int>>(initializer_list<int>{10, 20, 30, 40, 50});

    // Pass a reference to the vector to the function.
    // This is safe and does not involve manual memory management.
    processSensorData(*sensorData);

    // No need to manually delete the allocated vector;
    // unique_ptr automatically takes care of that.

    return 0;
}
