#include "FSM.h"
#include <iostream>
using namespace std;

// Interpolation linéaire :
// Doit retourner la valeur dans l'intervalle [x1,x2] qui correspond au placement de y1 dans l'intervalle [0,y2]
float linear_interpolate (float x1, float x2, float y1, float y2) {
    /// ============ TODO PARTIE I ============= ///
    return x1 + ((y1/y2)*(x2-x1));
    //return x1*(1-y1/y2) + x2*y1/y2;
    /// ======================================== ///
}

float cubic_interpolate (float x1, float x2, float y1, float y2) {
    float t = y1/y2;
    return (1-t)*(1-t)*(1-t) * x1 + 3*(1-t)*(1-t) * t * (x1 + (x2-x1)*3/7) + 3*(1-t)* t*t * (x1 + (x2-x1)*5/7) + t*t*t * x2;
}

FSM::FSM() { }

void FSM::update(double Dt, bool lc, bool rc, std::vector<float> currentAnglesLocal, std::vector<float> currentAnglesGlobal) {
    // Mise à jour du temps écoulé dans l'état
    m_timeInState += Dt;
    // L'état courant
    State s = m_states[m_currentState];
    // Si la machine a un seul état on reste dedans
    if (m_nbStates==1) return;
    // Récupération des appuis courants
    ContactState footContact = NONE;
    if (lc && !rc) footContact = LEFT_SUPPORT;
    else if (!lc && rc) footContact = RIGHT_SUPPORT;
    else if (lc && rc) footContact = DOUBLE_SUPPORT;
    else if (!lc && !rc) footContact = NO_SUPPORT;
    // Transition si durée écoulée ou si appui correct
    if ((s.transitionOnTimeOrContact && (m_timeInState > s.transitionTime))
        || (!s.transitionOnTimeOrContact && (footContact == s.transitionContact))) {
        // Remise à zéro du temps écoulé
        m_timeInState = 0.0;
        // Récupération des angles courants en tant que point de départ de l'état (animation fluide)
        for (unsigned int i=0;i<s.targetAngles.size();i++) {
                if (m_states[s.nextState].targetLocal[i]) m_anglesAtTransition[i] = currentAnglesLocal[i];
                else m_anglesAtTransition[i] = currentAnglesGlobal[i];
        }
        // Passage à l'état suivant
        m_currentState = s.nextState;
    }
}

std::vector<float> FSM::getCurrentTargetAngles() const {
    // Poses clés non interpolées si 1 état ou transition directe ou basé appui
    if (m_nbStates==1 || !m_states[m_currentState].transitionOnTimeOrContact || m_states[m_currentState].transitionTime==0.0) return m_states[m_currentState].targetAngles;

    // Poses clés interpolées sinon
    std::vector<float> targetAnglesInterpolated;
    float y1 = m_timeInState; // durée écoulée depuis le début de l'état courant
    float y2 = m_states[m_currentState].transitionTime;  // durée max avant transition
    for (unsigned int i=0; i<m_states[m_currentState].targetAngles.size();i++) {
        float x1 = m_anglesAtTransition[i]; // l'angle au début de l'état
        float x2 = m_states[m_currentState].targetAngles[i]; // la cible courante
        if (x1==x2) targetAnglesInterpolated.push_back(x1); // pas d'interpolation si identiques
        else targetAnglesInterpolated.push_back(linear_interpolate(x1,x2,y1,y2)); // interpolation linéaire sinon
    }
    return targetAnglesInterpolated;
}

FSM_Stand::FSM_Stand() {
    // La machine à états finis pour un mouvement stable debout
    // 1 état, toutes les articulation à zéro dans le monde

    // La machine à états finis pour un mouvement stable debout
    // 1 état, toutes les articulation à zéro dans le monde
    m_nbStates = 1;
    m_currentState = 0;
    State s;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 0;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.0;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(false);  //TRONC
    m_states.push_back(s);
}

FSM_Walk::FSM_Walk() {
    /// ============ TODO PARTIE I ============= ///
    m_nbStates = 7;
    m_currentState = 0;
    State s;
    float  t = 0.2;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = t;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);


    for (unsigned int i=0;i<s.targetAngles.size();i++)
         m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);



    s.ID = 1;
    s.nextState = 2;
    s.contactState = RIGHT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.2;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-1.6); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.4); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);



    s.ID = 2;
    s.nextState = 3;
    s.contactState = RIGHT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.4;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.9); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);



    s.ID = 3;
    s.nextState = 4;
    s.contactState = LEFT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.4;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.3); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(-0.2); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);



    s.ID = 4;
    s.nextState = 5;
    s.contactState = LEFT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.2;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(-1.6); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.4); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);



    s.ID = 5;
    s.nextState = 6;
    s.contactState = LEFT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.4;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.9); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);



    s.ID = 6;
    s.nextState = 1;
    s.contactState = RIGHT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 0.4;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.3); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(-0.2); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(-0.1); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);

   /// ========================================= ///
}



FSM_Jump::FSM_Jump() {
    /// ============ TODO PARTIE I ============= ///
    m_nbStates = 4;
    m_currentState = 0;
    State s;
    float  t = 0.3;
    // ETAT 0 //
    s.ID = 0;
    s.nextState = 1;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = t;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);


    for (unsigned int i=0;i<s.targetAngles.size();i++)
         m_anglesAtTransition.push_back(m_states[m_currentState].targetAngles[i]);



    s.ID = 1;
    s.nextState = 2;
    s.contactState = DOUBLE_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = t;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.5); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.3); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.3); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);



    s.ID = 2;
    s.nextState = 3;
    s.contactState = RIGHT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 2*t;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(-0.785); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(-0.785); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(1.57); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(1.57); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);



    s.ID = 3;
    s.nextState = 0;
    s.contactState = LEFT_SUPPORT;
    s.transitionOnTimeOrContact = true;
    s.transitionTime = 2*t;
    s.transitionContact = NONE;
    s.targetAngles.clear(); s.targetLocal.clear();
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //CHEVILLE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //GENOU_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_GAUCHE
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //HANCHE_DROIT
    s.targetAngles.push_back(0.0); s.targetLocal.push_back(true);  //TRONC
    m_states.push_back(s);


   /// ========================================= ///
}
