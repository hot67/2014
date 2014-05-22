
#ifndef _CHEESYVISIONSERVER_H_
#define _CHEESYVISIONSERVER_H_

#include "WPILib.h"
#include "JoystickWrapper.h"
#include "DriveWrapper.h"
#include "ArmWrapper.h"
#include "CameraHandler.h"
#include "VisionThread.h"
#include "Defines.h"
#include <cmath>
#include "networktables2/stream/SocketServerStreamProvider.h"





class CheesyVisionServer: private JankyTask
{

    static CheesyVisionServer *_instance;
    
    
    CheesyVisionServer(int port = 1180);
    int _listenPort;
    
    int _leftCount;
    int _rightCount;
    int _totalCount;
    bool _curLeftStatus;
    bool _curRightStatus;
    bool _counting;
    
    double _lastHeartbeatTime;
    bool _listening;
public:
    static CheesyVisionServer *GetInstance();
    bool HasClientConnection();
    void SetPort(int port){_listenPort = port;}
    virtual void Run();
    void StartListening(){_listening = true; Start();}
    void StopListening(){_listening = false; Pause();}
    
    void Reset();
    void UpdateCounts(bool left, bool right);
    void StartSamplingCounts(){_counting = true;}
    void StopSamplingCounts(){_counting = false;}

    bool GetLeftStatus(){return _curLeftStatus;}
    bool GetRightStatus(){return _curRightStatus;}
    int GetLeftCount(){return _leftCount;}
    int GetRightCount(){return _rightCount;}
    int GetTotalCount(){return _totalCount;}


};


#endif //CHEESYVISIONSERVER_H_
