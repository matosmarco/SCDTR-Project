#include "luminaire.h"

Luminaire::Luminaire(): 

node_id{0}, reference{0.0}, current_u{0.0}, low_bound{0.0}, high_bound{50.0},
    state{LuminaireState::OFF}, feedback_on{true} // Init
{}

// Reference state update
void Luminaire::updateReference() {
    switch (state) {
        case LuminaireState::OFF:  
            reference = 0.0; 
            break;
        case LuminaireState::LOW:  
            reference = low_bound; 
            break;
        case LuminaireState::HIGH: 
            reference = high_bound; 
            break;
    }
}

int Luminaire::getId() const {
    return node_id;
}

void Luminaire::setId(int new_id) {
    node_id = new_id;
}
