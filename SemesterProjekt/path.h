#ifndef PATH_H
#define PATH_H

#include <vector>

class Path
{
public:
    Path()
    {

        jointPoses.reserve(reservedMem);
        jointVel.reserve(reservedMem);
        toolPose.reserve(reservedMem);
        toolVel.reserve(reservedMem);
        elapsedTime.reserve(reservedMem);
    }

    Path(const Path& path){
        jointPoses = path.getJointPoses();
        jointVel = path.getJointVel();
        toolPose = path.getToolPose();
        toolVel = path.getToolVel();
        elapsedTime = path.getElapsedTime();

    }


    Path(Path&& path){
        jointPoses = std::move(path.jointPoses);
        jointVel = std::move(path.jointVel);
        toolPose = std::move(path.toolPose);
        toolVel = std::move(path.toolVel);

    }


    Path & operator = (const Path & p) {
        jointPoses = p.jointPoses;
        jointVel = p.jointVel;
        toolPose = p.toolPose;
        toolVel = p.toolVel;
        elapsedTime = p.elapsedTime;

        return *this;
    }

    Path & operator = (Path&& path){
        jointPoses = std::move(path.jointPoses);
        jointVel = std::move(path.jointVel);
        toolPose = std::move(path.toolPose);
        toolVel = std::move(path.toolVel);

        return *this;

    }


    void addPoint (const std::vector<double> &jointPosesValue, const std::vector<double> &jointVelValue, const std::vector<double> &toolPoseValue, const std::vector<double> &toolVelValue)
    {
        jointPoses.push_back(jointPosesValue);
        jointVel.push_back(jointVelValue);
        toolPose.push_back(toolPoseValue);
        toolVel.push_back(toolVelValue);
    }

    std::vector<std::vector<double> > getJointPoses() const {return jointPoses;}
    void addJointPose(const std::vector<double> &value)     {jointPoses.push_back(value);}

    std::vector<std::vector<double> > getJointVel() const   {return jointVel;}
    void addJointVel(const std::vector<double> &value)      {jointVel.push_back(value);}

    std::vector<std::vector<double> > getToolPose() const   {return toolPose;}
    void addToolPose(const std::vector<double> &value)      {toolPose.push_back(value);}

    std::vector<std::vector<double> > getToolVel() const    {return toolVel;}
    void addToolVel(const std::vector<double> &value)       {toolVel.push_back(value);}

    std::vector<double> getElapsedTime() const              {return elapsedTime;}
    void addElapsedTime(const double &value)                {elapsedTime.push_back(value);}

private:
    std::vector<std::vector<double>> jointPoses, jointVel, toolPose, toolVel;
    std::vector<double> elapsedTime;
    const unsigned int reservedMem = 80;
};

#endif // PATH_H


