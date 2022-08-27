#include "Trajectory.hpp"

namespace ryan{

TrajectoryPoint::TrajectoryPoint(double leftP, double rightP, double leftV, double rightV, double leftA, double rightA): 
leftPosition(leftP), rightPosition(rightP), leftVelocity(leftV), leftAcceleration(leftA), rightVelocity(rightV), rightAcceleration(rightA){}

std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt){
    os << pt.leftPosition << " " << pt.rightPosition << " " << pt.leftVelocity << " " << pt.rightVelocity << " " << pt.leftAcceleration << " " << pt.rightAcceleration;
    return os;
}

Trajectory::Trajectory(const std::initializer_list<TrajectoryPoint>& iPath):
path(iPath)
{}

TrajectoryPoint Trajectory::operator[](int index) const{
    if(index < 0 || index >= path.size()){
        return {0, 0, 0, 0, 0, 0};
    }
    else{
        return path[index];
    }
}

int Trajectory::size() const{
    return path.size();
}

}