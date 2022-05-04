#ifndef TASK_H
#define TAsk_H
#include <string>


// #include <tf2_ros/buffer.h>

class TASK
{
    public:
        TASK();
        ~TASK() {};
        void update();
        void isActive();

    private:
    void setDesiredPose();
    void getDesiredPose();
    void getJacobian();
    void getError();

       
};

class Configuration : public TASK
{
    public:
        Configuration();
        ~Configuration() {};

    private:
    void update();
    void isActive();
};

class JointLimits : public TASK
{
    public:
        JointLimits();
        ~JointLimits() {};

    private:
    void update();
    void isActive();
       
};

#endif