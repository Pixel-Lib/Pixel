#include "pxl/drivebase/drivebase.hpp"
#include "pxl/parametrics/pose.hpp"
#include "pxl/util.hpp"

namespace pxl {
    void Drivebase::Boomerang(float x, float y, float theta, float dlead, float timeout, std::shared_ptr<boomerangParams> boomerangParams, bool async) {
    mutex.take(TIMEOUT_MAX);
    if (async) {
        pros::Task task([&]() { Boomerang(x,y,theta,timeout,dlead,boomerangParams,false); });
        pros::delay(10);
        return;
    }

    Pose targetPose = Pose(x,y,degToRad(theta));

    float linearError;
    float angularError;

    Timer localTimeout(timeout);
    localTimeout.start();
    linearController.timerStart();
    angularController.timerStart();
    while (!localTimeout.isDone()
           || !linearController.getExit(linearError) && !angularController.getExit(angularError)) {
        float distance = this->odom.getPose().distance(targetPose);
           Pose carrot(targetPose.x - distance * cos(theta) * dlead,
			         targetPose.y - distance * sin(theta) * dlead, theta);
    
    }  
}}