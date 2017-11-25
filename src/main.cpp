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
    

    bool walking = false;


    ///Tryby:
    ///1 - stanie w miejscu i ruch translacyjny
    ///2 - stanie w miesjscu i obroty
    ///3 - poruszanie siê manualne
    ///4 - poruszanie siê automatyczne
    ///5 - automatyczne z poprawioną płynnością
    ///6 - automatyczne z płynnym chodzeniem do przodu i zakręcaniem

    int mode = 7;
    int direction = -1;
    int xMode = 0;
    int yMode = 0;
    int rotMode = 0;
        
	while(key != 27)
    {
        switch(key)
        {
            case 'W':
                direction = 0;
                break;
            case 'S':
                direction = 1;
                break;
            
            case 'A':
                direction = 2;
                break;
            case 'D':
                direction = 3;
                break;
            
            case 'Q':
                direction = 5;
                break;
            case 'E':
                direction = 4;
                break;

            case 'R':
                rob.restart();
            
            default:
                direction = -1;
        }

        switch(mode)
        {
            case 1:
                rob.moveBase(direction);
                break;
            case 2:
                rob.rotateBase(direction);
                break;
            case 3:
                xMode = 0;
                yMode = 0;
                rotMode = 0;
                break;
            case 4:
                xMode = 1;
                yMode = 1;
                rotMode = 1;
                break;
            case 5:
                xMode = 1;
                yMode = 2;
                rotMode = 1;
                break;
            case 6:
                xMode = 3;
                yMode = 3;
                rotMode = 2;
                break;
            case 7:
                xMode = 3;
                yMode = 4;
                rotMode = 2;
                break;

        }
        if(mode >= 3 && mode <= 7)
        {
            switch(key)
            {
                case 'W':
                case 'S':
                    rob.walk(yMode,direction,view1);
                    break;

                case 'A':
                case 'D':
                    rob.walk(xMode,direction,view1);
                    break;
                
                case 'Q':
                case 'E':
                    rob.rotate(rotMode,direction-4,view1);
                    break;
            }
        }
        else if(mode == 8)
        {
        	cout << "Podaj wspolrzedne: ";
        	Point2f pkt;
        	cin >> pkt.x >> pkt.y;
        	rob.walkToPoint(pkt, view1);
        	mode = 7;
        }
        else if(mode == 9)
        {
        	rob.showoff(view1);
        	mode = 7;
        }

        switch(key)
        {
            case '1':
                mode = 1;
                break;
            case '2':
                mode = 2;
                break;
            case '3':
                mode = 3;
                break;
            case '4':
                mode = 4;
                break;
            case '5':
                mode = 5;
                break;
            case '6':
                mode = 6;
                break;
            case '7':
            	mode = 7;
            	break;
            case '8':
            	mode = 8;
            	break;
            case '9':
            	mode = 9;
            	break;
        }

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
