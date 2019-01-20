#include "Biped.h"
#include <iostream>
using namespace std;

Biped::Biped (b2World* world) : m_world (world), m_hasFallen(false) { // Constructeur

		// Creation des corps rigides
		// ==========================

		// Proprietes communes
		b2BodyDef bodyDef;
		bodyDef.fixedRotation = false;
		bodyDef.allowSleep = false;
		bodyDef.awake = true;
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearDamping = 0.01f;
		bodyDef.angularDamping = 0.01f;
        b2PolygonShape shape;
		b2FixtureDef fixture;
		fixture.shape = &shape;
		fixture.filter.groupIndex = -1; // same group and don't collide

		// PIED GAUCHE
		bodyDef.position.Set(0.05f,0.05f); // 5cm au dessus du sol
		m_bodies[PIED_GAUCHE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.1f, 0.05f); // boite de 20cm x 10cm
        fixture.density = 5.0f;
        fixture.friction = 0.999; // friction grande pour les pieds
        fixture.userData = (void*)PIED_GAUCHE;
		m_bodies[PIED_GAUCHE]->CreateFixture(&fixture);

		// PIED DROIT
		bodyDef.position.Set(0.05f,0.05f); // 5cm au dessus du sol
		m_bodies[PIED_DROIT] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.1f, 0.05f); // boite de 20cm x 10cm
        fixture.density = 5.0f;
        fixture.friction = 0.999; // friction grande pour les pieds
        fixture.userData = (void*)PIED_DROIT;
		m_bodies[PIED_DROIT]->CreateFixture(&fixture);

		// JAMBE GAUCHE
		bodyDef.position.Set(0.0f,0.25f); // 15 cm au dessus de cheville
		m_bodies[JAMBE_GAUCHE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)JAMBE_GAUCHE;
		m_bodies[JAMBE_GAUCHE]->CreateFixture(&fixture);

		// JAMBE DROIT
		bodyDef.position.Set(0.0f,0.25f); // 15 cm au dessus de cheville
		m_bodies[JAMBE_DROIT] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)JAMBE_DROIT;
		m_bodies[JAMBE_DROIT]->CreateFixture(&fixture);

		// CUISSE GAUCHE
		bodyDef.position.Set(0.0f,0.55f); // 15 cm au dessus de genou
		m_bodies[CUISSE_GAUCHE] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)CUISSE_GAUCHE;
		m_bodies[CUISSE_GAUCHE]->CreateFixture(&fixture);

		// CUISSE DROIT
		bodyDef.position.Set(0.0f,0.55f); // 15 cm au dessus de genou
		m_bodies[CUISSE_DROIT] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.05f, 0.15f); // boite de 10cm x 30cm
        fixture.density = 5.0f;
        fixture.userData = (void*)CUISSE_DROIT;
		m_bodies[CUISSE_DROIT]->CreateFixture(&fixture);

		// TRONC
		bodyDef.position.Set(0.0f,1.0f); // 30 cm au dessus de hanche
		m_bodies[TRONC] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.1f, 0.3f); // boite de 20cm x 60cm
        fixture.density = 4.0f;
        fixture.userData = (void*)TRONC;
		m_bodies[TRONC]->CreateFixture(&fixture);

		// Creation des articulations
		// ==========================

		// Proprietes communes
		b2RevoluteJointDef jointDef;
		jointDef.lowerAngle = -0.5f *  b2_pi;
		jointDef.upperAngle = 0.5f *  b2_pi;
		jointDef.enableLimit = true;

		// CHEVILLE GAUCHE
		jointDef.Initialize(m_bodies[PIED_GAUCHE],m_bodies[JAMBE_GAUCHE],m_bodies[JAMBE_GAUCHE]->GetWorldCenter()+b2Vec2(0.0f,-0.15f));
		m_joints[CHEVILLE_GAUCHE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// CHEVILLE DROIT
		jointDef.Initialize(m_bodies[PIED_DROIT],m_bodies[JAMBE_DROIT],m_bodies[JAMBE_DROIT]->GetWorldCenter()+b2Vec2(0.0f,-0.15f));
		m_joints[CHEVILLE_DROIT] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// GENOU GAUCHE
		jointDef.Initialize(m_bodies[JAMBE_GAUCHE],m_bodies[CUISSE_GAUCHE],m_bodies[JAMBE_GAUCHE]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		jointDef.lowerAngle = 0.0;
		m_joints[GENOU_GAUCHE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// GENOU DROIT
		jointDef.Initialize(m_bodies[JAMBE_DROIT],m_bodies[CUISSE_DROIT],m_bodies[JAMBE_DROIT]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		m_joints[GENOU_DROIT] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);
		jointDef.lowerAngle = -0.5f * b2_pi;

		// HANCHE GAUCHE
		jointDef.Initialize(m_bodies[CUISSE_GAUCHE],m_bodies[TRONC],m_bodies[CUISSE_GAUCHE]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		m_joints[HANCHE_GAUCHE] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		// HANCHE DROIT
		jointDef.Initialize(m_bodies[CUISSE_DROIT],m_bodies[TRONC],m_bodies[CUISSE_DROIT]->GetWorldCenter()+b2Vec2(0.0f,0.15f));
		m_joints[HANCHE_DROIT] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0);

		m_currentAnglesLocal.push_back(0.0);
		m_currentAnglesGlobal.push_back(0.0); // pour l'angle du tronc

		// PD Controleurs
		// ==============

	    /// ============ TODO PARTIE II ============= ///
	    // Création des régulateurs PD :  m_PDControllers[articulation] = new PDController(gainKp,gainKv);


            m_PDControllers[0] = new PDController(0.1,0.1);
            m_PDControllers[1] = new PDController(0.1,0.1);

            m_PDControllers[2] = new PDController(0.8,0.8);
            m_PDControllers[3] = new PDController(0.8,0.8);

            m_PDControllers[4] = new PDController(0.9,0.85);
            m_PDControllers[5] = new PDController(0.9,0.85);

            m_PDControllers[6] = new PDController(0.7,0.7);

	    /// ========================================= ///

        // Finite State Machine
        // ====================
        /// ============ TODO PARTIE I ============= ///
        m_stateMachine = new FSM_Stand();
        //m_stateMachine = new FSM_Walk();
        /// ======================================== ///

}

Biped::~Biped() { // Destructor
    for (int i = 0; i <= NB_ARTICULATIONS; ++i) {
        if (m_PDControllers[i]!=NULL) {delete m_PDControllers[i]; m_PDControllers[i] = NULL;}
    }
    delete m_stateMachine;
}

void Biped::update(double Dt, bool lc, bool rc) {
    // Remise à zero des moments articulaires
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) m_motorTarget[j] = 0.0;

    // Mise à jour de la position et de la vitesse du COM
    computeCenterOfMass();

    if (!hasFallen()) { // Teste si le bipède est tombé
        // Mise à jour de l'état dans la machine si condition remplie
        m_stateMachine->update(Dt,lc,rc,m_currentAnglesLocal,m_currentAnglesGlobal);
        //m_currentAnglesLocal =  m_stateMachine->getCurrentTargetAngles();

        // Calcul des moments nécessaires au suivi des poses clés
        KeyPoseTracking();

        // Application des moments
        for (int j = 0; j <= NB_ARTICULATIONS; ++j)
        {
            float nextAngle = m_bodies[j]->GetAngle() +  m_bodies[j]->GetAngularVelocity() / 60.0;
            float totalRotation = m_motorTarget[j] - nextAngle;
            while ( totalRotation < - b2_pi ) totalRotation +=  b2_pi*2;
            while ( totalRotation >   b2_pi ) totalRotation -=  b2_pi*2;
            float desiredAngularVelocity = totalRotation * 60;
            float torque = m_bodies[j]->GetInertia() * desiredAngularVelocity / (1/60.0);
          //body->ApplyTorque( torque );
            m_bodies[j]->ApplyTorque(m_motorTarget[j]*9,true );
        }


    }
}

bool Biped::hasFallen() {
    // vrai si déja à terre (impossible de se remettre debout)
	if (m_hasFallen) return m_hasFallen;

	// détection que le bipède est tombé : le CdM du tronc est au niveau des genoux
	m_hasFallen = m_bodies[TRONC]->GetWorldCenter().y < 0.4;

	return m_hasFallen;
}

float Biped::sumTorque() const {
    // Retourne la somme des carrés des moments articulaires appliqués (pour l'optimisation)
    float sum = 0;
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) sum += m_motorTarget[j]*m_motorTarget[j];
    return sum;
}

float Biped::sumAngleVelocity() const {
    // Retourne la somme des vitesses angulaires absolues (pour l'optimisation)
    float sum = 0;
    for (int j = 0; j < NB_ARTICULATIONS; ++j) sum += fabs(m_joints[j]->GetJointSpeed());
    return sum;
}

//====================== PRIVATE ============================//

void Biped::computeCenterOfMass() {
    // Calcul de la position et de la vitesse du CdM du bipede dans le repere du monde
    float32 total_mass = 0.0f;
    m_velocityCOM = m_positionCOM;
    m_positionCOM.SetZero();
    for (int i = 0; i < NB_CORPS; ++i) {
        float32 massBody = m_bodies[i]->GetMass();
        b2Vec2 comBody = m_bodies[i]->GetWorldCenter();
        m_positionCOM += massBody * comBody;
        total_mass += massBody;
    }
    m_positionCOM = (1.0f / total_mass) * m_positionCOM;
    m_velocityCOM = m_positionCOM - m_velocityCOM;
}

void Biped::KeyPoseTracking () {
    // Récupération des cibles et de l'information local/global
    std::vector<float> targetAngles = m_stateMachine->getCurrentTargetAngles();
    std::vector<bool> targetLocal = m_stateMachine->getCurrentTargetLocal();

    // Pour toutes les articulations
    for (int j = 0; j <= NB_ARTICULATIONS; ++j) {
        /// ============ TODO PARTIE II ============= ///
        // Lire la cible pour l'articulation j dans targetAngles
          m_PDControllers[j]->setTarget(targetAngles[j]);
          m_currentAnglesLocal[j] = m_bodies[j]->GetAngle();
          m_motorTarget[j] = m_PDControllers[j]->compute(m_currentAnglesLocal[j]);
        // Affecter la cible au régulateur PD par setTarget

        // Mise à jour de m_currentAnglesLocal par b2RevoluteJoint::GetJointAngle() (attention au signe et attention pour j==NB_ARTICULATIONS pas d'angle local)

        // Mise à jour de m_currentAnglesGlobal par b2Body::GetTransform().q.GetAngle() avec l'équivalence d'indice d'articulation et de corps rigide (cf. énumérations)

        // Calcul du moment à ajouter dans m_motorTarget grâce au régulateur PD et en fonction de si la cible est locale ou globale

        /// ========================================= ///
    }
}
