#include "myprints.h"


const char* getStateName(robinsun_competition state)
{
   switch (state) 
   {
       case CALIBRATE:      return "CALIBRATE";
       case WAIT_FOR_START1:return "WAIT_FOR_START";
       case BLOCKS_FRONT:   return "BLOCKS_FRONT";
       case CABINS:         return "CABINS";
       case BLOCKS_DUNE_1:  return "BLOCKS_DUNE_1";
       case BLOCKS_DUNE_2:  return "BLOCKS_DUNE_2";
       case BLOCKS_CABINS:  return "BLOCKS_CABINS";
       case FISH:           return "FISH";
       case PARASOL:        return "PARASOL";
       case STOP:           return "STOP";
       default:             return "UNKNOWN STATE...";
   }
}

const char* getClampState(clampstate_t state)
{
   switch (state) 
   {
       case UNCLAMPED:  return "UNCLAMPED";
       case CLAMPED:    return "CLAMPED";
       case OPEN:       return "OPEN";
       case CLOSED:     return "CLOSED";
       default:         return "UNKNOWN STATE...";
   }
}

const char* getProgress(robinsun_competition state, int current_progress)
{
    switch(state)
    {
       case CALIBRATE:      
           return   "NOT IMPLEMENTED LOL";
       case WAIT_FOR_START1:
           return   "REMOVE THE JUMPER MAYBE ?";
       case BLOCKS_FRONT:   
           switch((blocks_front_t) current_progress)
           {
               case GOTO_BF:                return "Going to starting point...";
               case WAIT_FOR_POSITION_BF:   return "Waiting for starting point";
               case TURN_BF:                return "Turn towards blocks";
               case FORWARD_BF:             return "Go to the blocks";
               case CLAMP_BF:               return "Clamp the blocks";
               case PUSH_BF:                return "Push forward";
               case UNCLAMP_BF:             return "Unclamp...";
               case MOVE_BACK_BF:           return "Move back (last)";
               default:                     return "Unknown progression.";
           }
       case CABINS:
           switch((cabins_close_t) current_progress)
           {
               case GOTO_C:                 return "Going to starting point...";
               case WAIT_FOR_POSITION_C:    return "Waiting for starting point";
               case TURN_C:                 return "Turn back to cabins";
               case BACKWARDS_C:            return "Go backwards";
               case SWITCHES_C:             return "Switches pushed (last)";
               default:                     return "Unknown progression.";
           }
       case BLOCKS_DUNE_1:
           switch((blocks_dune_1_t) current_progress)
           {
               case GOTO_BD1:               return "Going to starting point...";
               case WAIT_FOR_POSITION_BD1:  return "Waiting for starting point";
               case TURN_BD1:               return "Turn to the dune";
               case FORWARD_BD1:            return "Go to the dune";
               case CLAMP_BD1:              return "Clamp the blocks (fingers crossed)";
               case MOVE_BACK_BD1:          return "Move backwards";
               case BRING_BD1:              return "Bring the blocks back";
               case UNCLAMP_BD1:            return "Unclamp the blocks";
               case MOVE_BACK_BD1_END:      return "Move backwards (last)";
               default:                     return "Unknown progression";
           }
       case BLOCKS_DUNE_2:  return "NOT IMPLEMENTED LOL";
       case BLOCKS_CABINS:
           switch((blocks_cabins_t) current_progress)
           {
               case GOTO_BC:                return "Going to starting point...";
               case WAIT_FOR_POSITION_BC:   return "Waiting for starting point";
               case TURN_BC:                return "Turn to face the blocks";
               case FORWARD_BC:             return "Go to the blocks";
               case CLAMP_BC:               return "Clamp 'em";
               case BACKWARDS_BC:           return "Go backwards";
               case BRING_BC:               return "Bring them back";
               case UNCLAMP_BC:             return "Unclamp the blocks";
               case MOVE_BACK_BC:           return "Move backwards (last)";
               default:                     return "Unknown progression";
           }
       case FISH:           return "NOT IMPLEMENTED LOL";
       case PARASOL:        return "You can't CI me";
       case STOP:           return "What did you expect ?";
       default:             return "YOU GUESSED IT...";
    }
}

