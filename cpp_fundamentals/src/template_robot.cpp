// Demonstrates the use of templates for a class method in C++ to process different types of data.
// The Robot class has a template method that accepts two parameters of potentially different types.
// This example showcases how templates provide flexibility and reduce the need for overloading methods.
// Author: Robotisim

#include <iostream>
using namespace std;

class Robot {
public:
    // Template function to process data of arbitrary types
    template <typename T, typename U>
    void processData(T data1, U data2) {
        cout << "Processing data: " << data1 << " and " << data2 << endl;
    }
    // void processData(int data1,int data2) {
    //     cout << "Processing integer data: " << data1+data2 << endl;
    // }
    // void processData(float data1,float data2) {
    //     cout << "Processing float data: " << data1+data2 << endl;
    // }
    // void processData(const string& data1, const string& data2) {
    //     cout << "Processing string data: " << data1 + " " + data2 << endl;
    // }
};

int main() {
    Robot myRobot;
    // Demonstrates the versatility of the template function
    myRobot.processData(5, 15); // Processes two integers
    myRobot.processData(3.14, 6.98f); // Processes a double and a float
    myRobot.processData("mobile", "robot"); // Processes two C-style string literals
    myRobot.processData("mobile", 3.41f); // Processes a C-style string literal and a float
    myRobot.processData('c', 1); // Processes a char and an integer

    return 0;
}
