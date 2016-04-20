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

const char* getProgress(robinsun_competition state, int current_progress)
{
    switch(state)
    {
       case CALIBRATE:      
           return   "CONFIGURE AND CALIBRATE";
       case WAIT_FOR_START1:
           return   "REMOVE THE JUMPER MAYBE ?";
       case BLOCKS_FRONT:   
           switch((blocks_front_t) current_progress)
           {
               case GOTO_BF:                return "Going to starting point...";
               case WAIT_FOR_POSITION_BF:   return "Waiting for starting point";
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
               case BACKWARDS_C:            return "Closing first cabin.";
               case SECOND_C:               return "Taking position for second cabin.";
               case BACKWARDS_C1:           return "Closing second cabin.";
               case SWITCHES_C:             return "Closed both cabins.";
               default:                     return "Unknown progression.";
           }
       case BLOCKS_DUNE_1:
           switch((blocks_dune_1_t) current_progress)
           {
               case GOTO_BD1:               return "Going to starting point...";
               case WAIT_FOR_POSITION_BD1:  return "Waiting for starting point";
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
       case FISH:
           switch((fish_catch_t) current_progress)
           {
               case GOTO_FC:                return "Going to starting point...";
               case WAIT_FOR_POSITION_FC:   return "Waiting for starting point.";
               case POSITION_FC:            return "Taking position for maneuver";
               case GO_CATCH_FC:            return "Maneuvring backwards";
               case BRING_OUT_FC:           return "Bringing out the plate";
               case LOWER_FC:               return "Lowering the plate";
               case SWEEP_FC:               return "Sweeping across ";
               case GO_UP_FC:               return "Bring the plate back up";
               case GO_NET_FC:              return "Going to the net";
               case BRING_IN_FC:            return "Letting go of the fish";
               default:                     return "Unknown progression";
           }
       case PARASOL:        return "You can't CI me";
       case STOP:           return "Time has run out...";
       default:             return "I have no idea what I'm doing...";
    }
}

