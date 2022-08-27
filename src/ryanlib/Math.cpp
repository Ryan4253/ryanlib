#include "Math.hpp"

namespace ryan{

double Math::deadband(double value, double db){
    if(abs(value) < db){
        return 0;
    }
    else{
        return value;
    }
}

// vel (ft/s) -> vel(in/s) -> rps(wheel) -> rps -> rpm
double Math::ftpsToRPM(double ftps, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset){
    return ftps * 12 / (scale.wheelDiameter.convert(okapi::inch) * M_PI) * gearset.ratio * 60;
}

// rpm -> rps -> rps (wheel) -> vel (in/s) -> vel (ft/s)
double Math::rpmToFtps(double rpm, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset){
    return rpm / 60 / gearset.ratio * (scale.wheelDiameter.convert(okapi::inch) * M_PI) / 12;
}

double Math::ftToTick(double ft, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset){
    return ft * 12 / (scale.wheelDiameter.convert(okapi::inch)*M_PI) * (gearset.ratio) * scale.tpr;
}

double Math::tickToFt(double tick, okapi::ChassisScales scale, okapi::AbstractMotor::GearsetRatioPair gearset){
    return (tick/scale.tpr)/(gearset.ratio)*(scale.wheelDiameter.convert(okapi::inch)*M_PI)/12;
}

okapi::QAngle Math::rescale180(okapi::QAngle angle){
    return rescale180(angle.convert(okapi::degree)) * okapi::degree;
}

double Math::rescale180(double angle){
    return angle - 360.0 * std::floor((angle + 180.0) * (1.0 / 360.0));
}

okapi::QAngle Math::rescale360(okapi::QAngle angle){
    return rescale360(angle.convert(okapi::degree)) * okapi::degree;
}

double Math::rescale360(double angle){
    return angle - 360.0 * (std::floor(angle * (1.0 / 360.0)));
}

std::pair<double, double> Math::quadraticFormula(double a, double b, double c){
    double discriminant = b * b - 4 * a * c;

    if(discriminant < 0){
        throw std::runtime_error("Quadratic Formula: No Real Solution!");
    }
    else if(abs(discriminant) <= 0.001){
        return {-b / 2 * a, -b / 2 * a};
    }
    else{
        double x1 = (-b + sqrt(discriminant)) / (2 * a);
        double x2 = (-b - sqrt(discriminant)) / (2 * a);

        return {x1, x2};
    }
}

int Math::signum(double value){
    return (value > 0) - (value < 0);
}

}

