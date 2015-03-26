#include "arduino-pc.h"

class CameraControl:Arduino-PC
{
    public:
    void Camera_setup()
    {
        srvCamRot.attach(PIN_CAM_ROTATE);
        srvCamRot.writeMicroseconds(CAM_ROTATE_STOP);
        camRotateCCW();
        srvCamPitch.attach(PIN_CAM_PITCH);
        camPitch(125);

    }

    void camRotateCW()
    {
        srvCamRot.writeMicroseconds(CAM_ROTATE_CW);
    }

    void camRotateCCW()
    {
        srvCamRot.writeMicroseconds(CAM_ROTATE_CCW);
    }

    void camPitch(angle)
    {
        srvCamPitch.write(angle);
    }
    protected:
    int angle;
}
