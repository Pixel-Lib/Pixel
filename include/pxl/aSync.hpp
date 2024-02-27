#pragma once
#include "pros/rtos.hpp"
#include "pxl/parametrics/pose.hpp"
#include "pxl/util.hpp"
#include <functional>
#include "pxl/parametrics/coord.hpp"

//Macro for implicit creatiion of LAMBDAs 
#define LAMBDA(func) [](){func;}

namespace pxl {
 
 enum action_Point{
  pose,
  point,
  error,
  sensed,
 };

 class aSync{

  protected:
  pros::Task asyncTask;
  

 };

 struct ActionFuncTuple{
   std::function<void()> func;
   action_Point actionPoint;
   bool called;
   ActionFuncTuple(std::function<void()> func, action_Point actionPoint, bool called):
   func(func),actionPoint(actionPoint), called(called){}

   
 };

};// namespace pxl