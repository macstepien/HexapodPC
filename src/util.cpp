#include "util.h"
#include "Gamepad/gamepad.h"
#include <memory>
#include "TCP/tcpconnector.h"
#include <iostream>

using namespace cv;
using namespace std;

cv::Point3f rotate3D(cv::Point3f pt, cv::Point3f angles)
{
    Mat Rx = (Mat_<float>(3,3) << 1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
    Mat Ry = (Mat_<float>(3,3) << cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
    Mat Rz = (Mat_<float>(3,3) << cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);

    Mat R = Rz*Ry*Rx;

    Mat P1 = (Mat_<float>(3,1) << pt.x, pt.y, pt.z);

    Mat P11 = R*P1;

    Point3f p1 = Point3f(P11.at<float>(0,0), P11.at<float>(0,1), P11.at<float>(0,2));

    return p1;
}

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

char getChar(){
 	
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