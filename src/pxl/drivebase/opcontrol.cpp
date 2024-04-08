
#include "pxl/drivebase/drivebase.hpp"

namespace pxl {
pxl::Controller::Controller(joystickCurveParams throttleParams, joystickCurveParams turnParams)
    : throttleParams(throttleParams), turnParams(turnParams) {}

float Controller::joystickCurve(float val, joystickCurveParams params) {
    float g_x = std::fabs(val) - params.deadzone;
    float i_x = std::pow(params.curve, (g_x - params.scale)) * g_x * pxl::sgn(val);

    float result = 0.0f;

    if ((-params.scale <= val && val <= -params.deadzone) || (params.deadzone <= val && val <= params.scale)) {
        result = ((params.scale - params.minOutput) / params.scale) * params.scale + params.minOutput * pxl::sgn(val);
    }

    return result;
}

void Drivebase::tank(float left, float right, std::function<float(float, float)>curveFunc) {

    drivetrain.leftMotors->move(curveFunc(left,controller.throttleParams.curve));
    drivetrain.rightMotors->move(curveFunc(right,controller.throttleParams.curve));
}

}  // namespace pxl