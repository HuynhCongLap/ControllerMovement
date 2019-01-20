#ifndef FSM_H
#define FSM_H

#include <vector>

enum ContactState {LEFT_SUPPORT,RIGHT_SUPPORT,DOUBLE_SUPPORT,NO_SUPPORT,NONE};  // Les differents états d'appui possibles

struct State {
    unsigned int ID;                    // Identificateur de l'état
    std::vector<float> targetAngles;    // Angle clé pour chaque articulation
    std::vector<bool> targetLocal;      // Vrai si l'angle clé est local au parent, faux sinon (global)
    unsigned int nextState;             // Identificateur de l'état suivant
    ContactState contactState;          // L'état des appuis des pieds au sol
    bool transitionOnTimeOrContact;     // Vrai si la transition est sur la durée, faux si sur le changement d'appui
    float transitionTime;               // La durée de transition (si basé durée)
    ContactState transitionContact;     // Les appuis de transition (si basé appui)
};

class FSM {

public:

	// Mise à jour de l'état si condition remplie
	void update(double Dt, bool lc, bool rc, std::vector<float> currentAnglesLocal, std::vector<float> currentAnglesGlobal);

	// Retourne les cibles et information local/global
	std::vector<float> getCurrentTargetAngles() const;
	std::vector<bool> getCurrentTargetLocal() const {return m_states[m_currentState].targetLocal;}

	// Retourne l'ID de l'état
	unsigned int getID() const {return m_states[m_currentState].ID;}

protected:

	FSM();      // Construit la machine avec ses états

public:
    unsigned int            m_nbStates;             // Le nombre d'états
    unsigned int            m_currentState;         // L'indice de l'état courant
    std::vector<State>      m_states;               // Les états
    double                  m_timeInState;          // Temps écoulé dans l'état
    std::vector<float>      m_anglesAtTransition;   // Les angles au moment de la dernière transition

};

class FSM_Stand : public FSM { // La machine à états finis pour un mouvement stable debout
public:
	FSM_Stand();
};

class FSM_Walk : public FSM { // La machine à états finis pour un mouvement de marche
public:
	FSM_Walk();
};

#endif
