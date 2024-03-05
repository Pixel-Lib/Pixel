#pragma once
#include <cstdint>

namespace pxl {

 // Flags used for standstill calculations
 enum movement_Type {
   lateral,
   turn
 };

 struct stand_still{
    public:
    void setStandStill(movement_Type type, uint8_t maxCycles, float maxStep);

    private:
    //Maximum distance to add a new count for SS
    float maxStepDistance_t = 2;
    float maxStepTurn_d = 0.2;

    float SSMaxCount_lateral = 10;
    float SSMaxCount_turn = 10;
     
    bool SSActive = true;
    bool SSActive_t = true;
    bool updateStandstill(movement_Type type, bool& standStill, float error, float lastError, uint8_t& standStillCount);
 };
 struct parallel_line_cross{

 };

 struct global_timeOuts{
   /**
     * @brief function that ends movemnet after n time 
     * 
     * @param timeOut_S maximum time a movement is allocated in seconds 
     *
     */
    void timeOut(float timeOut_S);

    void early_exit_range(float distance = 1 , float timeout = 500);
    
 };

 class exit_conditions {
     public:
     exit_conditions(stand_still mainExit, global_timeOuts timeOut);
     exit_conditions(parallel_line_cross mainExit, global_timeOuts timeOut);
    
     protected:
         /**
          * boolean to indicate if the robot has settled or not
          */
         bool isSettled;

     private:

 };

}  // namespace pxl