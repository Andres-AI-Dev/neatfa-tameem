// Fixed GetFloorColor implementation that actually renders food
#include "iAnt_loop_functions.h"

CColor iAnt_loop_functions::GetFloorColor(const CVector2& position) {
    // Check if this position is near the nest
    if((position - NestPosition).Length() < NestRadius) {
        return CColor::GRAY50;  // Gray nest area
    }

    // Check if this position is near any food item
    for(size_t i = 0; i < FoodList.size(); i++) {
        if((position - FoodList[i]).Length() < FoodRadius) {
            // Return color based on the food coloring list or default to black
            if(i < FoodColoringList.size()) {
                return FoodColoringList[i];
            }
            return CColor::BLACK;  // Default food color
        }
    }

    // Check if near pheromone trail
    for(size_t i = 0; i < Pheromones.size(); i++) {
        if((position - Pheromones[i].Location).Length() < 0.05) {
            // Fade pheromone color based on strength
            int green = 255 * (Pheromones[i].Strength / 1.0);
            return CColor(0, green, 0);  // Green pheromone trail
        }
    }

    return CColor::WHITE;  // Default floor color
}