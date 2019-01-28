#ifndef FSM_H
#define FSM_H

#include <vector>

enum ContactState {LEFT_SUPPORT,RIGHT_SUPPORT,DOUBLE_SUPPORT,NO_SUPPORT,NONE};  // Les differents �tats d'appui possibles

struct State {
    unsigned int ID;                    // Identificateur de l'�tat
    std::vector<float> targetAngles;    // Angle cl� pour chaque articulation
    std::vector<bool> targetLocal;      // Vrai si l'angle cl� est local au parent, faux sinon (global)
    unsigned int nextState;             // Identificateur de l'�tat suivant
    ContactState contactState;          // L'�tat des appuis des pieds au sol
    bool transitionOnTimeOrContact;     // Vrai si la transition est sur la dur�e, faux si sur le changement d'appui
    float transitionTime;               // La dur�e de transition (si bas� dur�e)
    ContactState transitionContact;     // Les appuis de transition (si bas� appui)
};

class FSM {

public:

	// Mise � jour de l'�tat si condition remplie
	void update(double Dt, bool lc, bool rc, std::vector<float> currentAnglesLocal, std::vector<float> currentAnglesGlobal);

	// Retourne les cibles et information local/global
	std::vector<float> getCurrentTargetAngles() const;
	std::vector<bool> getCurrentTargetLocal() const {return m_states[m_currentState].targetLocal;}

	// Retourne l'ID de l'�tat
	unsigned int getID() const {return m_states[m_currentState].ID;}

protected:

	FSM();      // Construit la machine avec ses �tats

public:
    unsigned int            m_nbStates;             // Le nombre d'�tats
    unsigned int            m_currentState;         // L'indice de l'�tat courant
    std::vector<State>      m_states;               // Les �tats
    double                  m_timeInState;          // Temps �coul� dans l'�tat
    std::vector<float>      m_anglesAtTransition;   // Les angles au moment de la derni�re transition

};

class FSM_Stand : public FSM { // La machine � �tats finis pour un mouvement stable debout
public:
	FSM_Stand();
};

class FSM_Walk : public FSM { // La machine � �tats finis pour un mouvement de marche
public:
	FSM_Walk();
};


class FSM_Jump : public FSM { // La machine � �tats finis pour un mouvement de saut
public:
	FSM_Jump();
};

#endif
