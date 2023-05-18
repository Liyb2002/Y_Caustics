#ifndef BASICCAMERA_H
#define BASICCAMERA_H

#include "camera.h"

class BasicCamera : public Camera
{
public:
    BasicCamera(){}
    BasicCamera(Eigen::Vector3f position, Eigen::Vector3f direction, Eigen::Vector3f up, float heightAngle, float aspect);

    virtual Eigen::Matrix4f getViewMatrix() const;
    virtual Eigen::Matrix4f getScaleMatrix() const;

    Eigen::Vector3f getOrigin() const {
        return m_position;
    }

    Eigen::Vector3f m_u; // up
    Eigen::Vector3f m_s; // right
    Eigen::Vector3f m_f; // look

private:
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_direction;
    Eigen::Vector3f m_up;

    float m_heightAngle;
    float m_aspectRatio;
};

#endif // BASICCAMERA_H
