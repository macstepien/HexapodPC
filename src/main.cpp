#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <memory>
#include "tcpconnector.h"
#include "view.h"
#include "robotcontroler.h"

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
    bool usingTCP = true;
    unique_ptr<TCPConnector> connector;
    TCPStream* stream;

    /*unique_ptr<TCPConnector> connector1;
    TCPStream* stream1;*/

    int len;
    char line[64];

    if(argc < 2)
    {
    	cout << "Tryb bez połączenia z hexapodem"<< endl;
    	usingTCP = false;
    }
    else
    {
    	cout << "Łączenie z " << argv[1] << ":8080" << endl;
    	connector = make_unique<TCPConnector>();
   	 	stream = connector->connect(argv[1], 8080);

        /*cout << "Łączenie z " << argv[1] << ":8081" << endl;
        connector1 = make_unique<TCPConnector>();
        stream1 = connector1->connect(argv[1], 8081);*/
    }

    View view1(1000, Point3f(0,0,0), Point3f(0, -300, 0));

    char key = 'm';

    Point3f robotPosition(0, 17, 100);
    Point3f robotAngles(0,0,0);
    float robotWidth = 11.8;
    float robotLength = 36.5;
    Point3f robotLegLenghts(3.7, 5.8, 16.3);

    float walkStep = 5;
    float rotStep = 0.3;
    float sMoveStep = 1;
    float sRotStep = 0.05;

    RobotControler rob(walkStep, rotStep, sMoveStep, sRotStep, robotPosition, robotAngles, robotWidth, robotLength, robotLegLenghts);

    ///Tryby:
    ///1 - stanie w miejscu i ruch translacyjny
    ///2 - stanie w miesjscu i obroty
    ///3 - poruszanie siê manualne
    ///4 - poruszanie siê automatyczne
    ///5 - automatyczne z poprawioną płynnością
    ///6 - automatyczne z płynnym chodzeniem do przodu i zakręcaniem
        
	while(key != 27)
    {
        rob.control(key,view1);
        view1.update(key, rob.getRobot());
        key = waitKey(10);

        if (usingTCP && stream)
        	stream->send(&key,sizeof(key));

        /*if (usingTCP && stream1)
        {

            len = stream1->receive(line, sizeof(line));
            line[len] = '\0';
            cout << line;
        }*/
    }
    
    return 0;
}
