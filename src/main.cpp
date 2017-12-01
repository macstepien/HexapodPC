#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <ncurses.h>
#include "TCP/tcpconnector.h"
#include "GUI/gui.h"
#include "Robot/robotcontroler.h"
#include "Gamepad/gamepad.h"

using namespace std;
using namespace cv;

struct voltage
{
    string voltageStr;
    mutex own;
};

void battery(bool communication, char* adres, bool* end, voltage* v)
{
    if(!communication)
        return;

    int len;
    char line[64];

    unique_ptr<TCPConnector> connector1;
    TCPStream* stream1;
    cout << "Łączenie z " << adres << ":8081" << endl;
    connector1 = make_unique<TCPConnector>();
    stream1 = connector1->connect(adres, 8081);
    while(stream1 && !(*end))
    {
        len = stream1->receive(line, sizeof(line));
        line[5] = '\0'; // liczba w formacie X.XXX - 3 miejsca po przecinku
        //cout << line << endl;
        unique_lock<mutex> lck(v->own);
        v->voltageStr = line;
    }
}

static char getChar(){
 	
 	static int mode;
    
    float XX,YY;
    GamepadStickNormXY((GAMEPAD_DEVICE)0, STICK_LEFT,&XX, &YY);
    int x,y;
    x=(int)XX;
    y=(int)YY;

    if(mode == 0)
    {
	    if ( XX == -1){
	        return 'a';
	    }
	    if ( XX == 1){
	        return 'd';
	    }
	    if ( YY == 1){
	        return 'w';
	    }
	    if ( YY == -1){
	        return 's';
	    }
	}
	
    GamepadStickNormXY((GAMEPAD_DEVICE)0, STICK_RIGHT,&XX, &YY);
    x=(int)XX;
    y=(int)YY;

    if(mode == 0)
    {
	    if ( XX == -1){
	        return 'q';
	    }
	    if ( XX == 1){
	        return 'e';
	    }
	}

    bool button;
    
    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )5);
    if ( button == true) {
        return 'r';
    }

    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )4);
    if ( button == true){
        if(mode == 0)
        	mode = 1;
        else
        	mode = 0;
        return 'p';
    }

    if(mode == 1)
	{
		button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )15);
		if ( button == true){
		    return 'w';
		}
		    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )12);
		if ( button == true){
		    return 's';
		}
		button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )14);
		if ( button == true){
		    return 'a';
		}
		button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )13);
		if ( button == true){
		    return 'd';
		}
		button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )8);
		if ( button == true){
		    return 'q';
		}
		button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )9);
		if ( button == true){
		    return 'e';
		}
	}
	else
	{
		button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )12);
	    if ( button == true){
	        return '1';
	    }
	    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )13);
	    if ( button == true ){
	        return '2';
	    }
	    
	    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )14);
	    if ( button == true){
	        return '5';
	    }
	    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )15);
	    if ( button == true){
	        return '6';
	    }

	    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )8);
	    if ( button == true){
	        return '7';
	    }
	    button = GamepadButtonTriggered((GAMEPAD_DEVICE)0,  (GAMEPAD_BUTTON )9);
	    if ( button == true){
	        return 'z';
	    }

	}

    return 'm';
 
}

int main(int argc, char** argv)
{
    bool usingTCP = true;
    unique_ptr<TCPConnector> connector;
    TCPStream* stream;

    int len;
    char line[64];
    bool end = false;

    voltage v;

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
    }

    thread t1 {battery, argc>=2, argv[1], &end, &v};

    GUI view1(1000, Point3f(0,0,0), Point3f(0, -300, 0));

    char keyPad = 'm';
    char keyKeyboard = 'm';

    Point3f robotPosition(0, 17, 100);
    Point3f robotAngles(0,0,0);
    float robotWidth = 11.8;
    float robotLength = 36.5;
    Point3f robotLegLenghts(3.7, 5.8, 16.3);

    float walkStep = 5;
    float rotStep = 0.3;
    float sMoveStep = 1;
    float sRotStep = 0.05;

    int delayShort = 10;
    float stepHeight = 4;

    RobotControler rob(walkStep, rotStep, sMoveStep, sRotStep, delayShort, stepHeight, robotPosition, robotAngles, robotWidth, robotLength, robotLegLenghts);

    ///Tryby:
    ///1 - stanie w miejscu i ruch translacyjny
    ///2 - stanie w miesjscu i obroty
    ///3 - poruszanie siê manualne
    ///4 - poruszanie siê automatyczne
    ///5 - automatyczne z ruchem nogi po paraboli
    ///6 - automatyczne bez wracania do pozycji początkowej
    ///8 - chodzenie do punktu
    ///9 - tryb pokazowy

    ///Pad:
    /// Y - automatyczne bez wracania do pozycji początkowej
    /// X - automatyczne z ruchem nogi po paraboli
    /// A - 
    /// B - 

    //initscr();
    cbreak();
    noecho();
    timeout(1);
    unsigned int microseconds;
    microseconds = 100000;
    GamepadInit();

    int mode = 1;
        
	while(keyKeyboard != 27)
    {
        
        GamepadUpdate();
 
        if (!GamepadIsConnected((GAMEPAD_DEVICE)0)) {
            printf("%d) n/a\n", (GAMEPAD_DEVICE)0);
            return 0;
        }
 
        keyPad=getChar();

        /*if(keyPad == '1' || keyPad == '2')
        	mode = 2;
        else if(keyPad == '3' || keyPad == '4' || keyPad == '5' || keyPad == '6' || keyPad == '7' || keyPad == '8' || keyPad == '9')
        	mode = 1;*/

        rob.control(keyPad,view1);

        if(usingTCP)
        {
            unique_lock<mutex> lck(v.own);
            view1.setVoltage(v.voltageStr);
        }
        
        view1.update(keyKeyboard, rob.getRobot());

        keyKeyboard = waitKey(10);

        if(usingTCP && stream)
        	stream->send(&keyPad,sizeof(keyPad));

        /*if(key == '8')
        {
            Point pkt;
            cout << "Podaj wspolrzedne: ";
            cin >> pkt.x >> pkt.y;
            rob.walkToPoint(pkt, view1);
            if(usingTCP && stream)
            {
                char p[2];
                p[1] = pkt.x;
                p[2] = (pkt.x >> 8);
                stream->send(p,sizeof(p));

                p[1] = pkt.y;
                p[2] = (pkt.y >> 8);
                stream->send(p,sizeof(p));
            }
        }*/
    }

    end = true;
    t1.join();
    
    return 0;
}
